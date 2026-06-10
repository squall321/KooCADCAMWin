// @lat: [[engine/skills#linear_rail_seat_compound]]

#include "linear_rail_seat_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::linear_rail_seat_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Compound feature defaults (DIN 645 / ISO 12090 LM rail mount, M5 fastener):
constexpr double kSlotWidthMm    = 12.0;
constexpr double kSlotDepthMm    = 4.0;
constexpr double kRowOffsetMm    = 9.0;   // each row's offset from rail centerline
constexpr double kPilotDiaMm     = 4.2;   // M5 metric coarse pilot
constexpr double kTapDepthMm     = 8.0;
constexpr double kChamferSizeMm  = 0.5;
constexpr double kHoleExtraDepth = 1.0;   // chip clearance below tap

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "linear_rail_seat_compound: length_mm must be > 0");
    }
    if (in.pitch_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "linear_rail_seat_compound: pitch_mm must be > 0");
    }

    if (in.pitch_mm > 0.0 && (in.pitch_mm < 10.0 || in.pitch_mm > 80.0)) {
        r.add("DFM-RAIL-PITCH", "warning",
              "linear_rail_seat_compound: pitch_mm " +
              std::to_string(in.pitch_mm) +
              " outside typical [10, 80] mm range (DIN 645 LM rail)");
    }

    if (in.length_mm > 0.0 && in.pitch_mm > 0.0 &&
        in.length_mm < in.pitch_mm + 2.0 * kRowOffsetMm) {
        r.add("DFM-RAIL-LEN", "error",
              "linear_rail_seat_compound: length_mm " +
              std::to_string(in.length_mm) +
              " too short to fit at least one full hole pitch");
    }

    // Row offset must clear slot wall: kRowOffsetMm - (kSlotWidthMm / 2) >= 2 mm
    constexpr double rowFromSlot = kRowOffsetMm - kSlotWidthMm / 2.0;
    if constexpr (rowFromSlot < 2.0) {
        r.add("DFM-RAIL-WALL", "error",
              "linear_rail_seat_compound: hole row sits " +
              std::to_string(rowFromSlot) +
              " mm from slot wall (< 2 mm) — compile-time defaults invalid");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "linear_rail_seat_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("linear_rail_seat_compound: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    constexpr double kOverhang = 0.05;
    const double topZ = zMax;

    // ── Sub-feature 1: longitudinal rail slot ──────────────────────────
    // Slot extends along +X starting at start_x_mm, centered on rail_centerline_mm
    // in Y, depth kSlotDepthMm into the material (downward).
    const gp_Pnt slotOrigin(
        in.start_x_mm,
        in.rail_centerline_mm - kSlotWidthMm / 2.0,
        topZ - kSlotDepthMm);
    const gp_Ax2 slotAx(slotOrigin, gp_Dir(0, 0, 1), gp_Dir(1, 0, 0));
    const TopoDS_Shape slotTool = pr::box(slotAx,
                                          in.length_mm,
                                          kSlotWidthMm,
                                          kSlotDepthMm + kOverhang);

    // ── Sub-feature 2: edge chamfer (modelled as a thin top-strip removal
    //    around the slot perimeter for slice-1).  We approximate the
    //    chamfer with a slightly larger box at the very top of the slot,
    //    extending kChamferSizeMm above the slot edge on each side. ──
    const gp_Pnt chamferOrigin(
        in.start_x_mm,
        in.rail_centerline_mm - kSlotWidthMm / 2.0 - kChamferSizeMm,
        topZ - kChamferSizeMm);
    const gp_Ax2 chamferAx(chamferOrigin, gp_Dir(0, 0, 1), gp_Dir(1, 0, 0));
    const TopoDS_Shape chamferTool = pr::box(chamferAx,
                                             in.length_mm,
                                             kSlotWidthMm + 2.0 * kChamferSizeMm,
                                             kChamferSizeMm + kOverhang);

    // ── Sub-feature 3+: 2 rows of M5 tapped holes at pitch_mm ──────────
    // Holes are positioned at start_x + pitch/2 + k·pitch (centered between slot ends).
    // We fit floor((length - pitch) / pitch) + 1 holes per row, at least 1.
    const int holesPerRow = std::max(1,
        static_cast<int>(std::floor((in.length_mm - in.pitch_mm) / in.pitch_mm)) + 1);
    const double rowHoleSpan = (holesPerRow - 1) * in.pitch_mm;
    const double firstHoleX  = in.start_x_mm + (in.length_mm - rowHoleSpan) / 2.0;

    std::vector<TopoDS_Shape> holeTools;
    holeTools.reserve(static_cast<size_t>(2 * holesPerRow));
    for (int row = 0; row < 2; ++row) {
        const double yPos = (row == 0)
            ? in.rail_centerline_mm + kRowOffsetMm
            : in.rail_centerline_mm - kRowOffsetMm;
        for (int k = 0; k < holesPerRow; ++k) {
            const double xPos = firstHoleX + k * in.pitch_mm;
            const gp_Pnt holeStart(xPos, yPos, topZ + kOverhang);
            const gp_Ax2 holeAx(holeStart, gp_Dir(0, 0, -1));
            const double holeDepth = kTapDepthMm + kHoleExtraDepth + kOverhang;
            holeTools.push_back(pr::cylinder(holeAx, kPilotDiaMm / 2.0, holeDepth));
        }
    }

    // Bundle all sub-feature tools (slot, chamfer, holes) into a single
    // cut for performance and to leave one coherent feature signature.
    std::vector<TopoDS_Shape> allTools;
    allTools.reserve(2 + holeTools.size());
    allTools.push_back(slotTool);
    allTools.push_back(chamferTool);
    for (const auto& t : holeTools) allTools.push_back(t);

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), allTools);

    // Sub-feature count: 1 slot + 1 chamfer + 2*holesPerRow holes.
    const int subFeatureCount = 2 + 2 * holesPerRow;

    json holePositions = json::array();
    for (int row = 0; row < 2; ++row) {
        const double yPos = (row == 0)
            ? in.rail_centerline_mm + kRowOffsetMm
            : in.rail_centerline_mm - kRowOffsetMm;
        for (int k = 0; k < holesPerRow; ++k) {
            holePositions.push_back({
                { "x", firstHoleX + k * in.pitch_mm },
                { "y", yPos },
            });
        }
    }

    json params = {
        { "entry_face_id",      *entryId },
        { "length_mm",          in.length_mm },
        { "rail_centerline_mm", in.rail_centerline_mm },
        { "pitch_mm",           in.pitch_mm },
        { "start_x_mm",         in.start_x_mm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  subFeatureCount },
        { "slot_count",        1 },
        { "chamfer_count",     1 },
        { "hole_count",        2 * holesPerRow },
        { "holes_per_row",     holesPerRow },
        { "rows",              2 },
        { "length_mm",         in.length_mm },
        { "pitch_mm",          in.pitch_mm },
        { "rail_centerline_mm", in.rail_centerline_mm },
        { "start_x_mm",        in.start_x_mm },
        { "slot_width_mm",     kSlotWidthMm },
        { "slot_depth_mm",     kSlotDepthMm },
        { "row_offset_mm",     kRowOffsetMm },
        { "thread_size",       "M5" },
        { "pilot_dia_mm",      kPilotDiaMm },
        { "tap_depth_mm",      kTapDepthMm },
        { "chamfer_size_mm",   kChamferSizeMm },
        { "hole_positions",    holePositions },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "end_mill;drill;tap;chamfer_mill";
    tooling.tool_dia_mm      = kSlotWidthMm;
    tooling.tool_length_mm   = std::max(kSlotDepthMm, kTapDepthMm) * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 4;
    tooling.cutting_speed_sfm = 500.0;
    tooling.feed_per_tooth_mm = 0.04;
    // Stock removal: slot box + per-hole pilots (chamfer negligible).
    const double slotVol = in.length_mm * kSlotWidthMm * kSlotDepthMm;
    const double holeVol = M_PI * (kPilotDiaMm/2.0)*(kPilotDiaMm/2.0)
                         * (kTapDepthMm + kHoleExtraDepth) * 2.0 * holesPerRow;
    tooling.stock_removed_mm3 = slotVol + holeVol;
    tooling.est_cycle_time_s  = std::max(5.0,
        in.length_mm * 0.1 + (2 * holesPerRow) * 3.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "end_mill" },
              { "purpose",   "rail slot" },
              { "tool_dia_mm", kSlotWidthMm },
              { "depth_mm",    kSlotDepthMm } },
            { { "tool_type", "chamfer_mill" },
              { "purpose",   "edge chamfer" },
              { "chamfer_size_mm", kChamferSizeMm } },
            { { "tool_type", "drill" },
              { "purpose",   "tap pilots" },
              { "tool_dia_mm", kPilotDiaMm },
              { "depth_mm",    kTapDepthMm + kHoleExtraDepth },
              { "pass_count",  2 * holesPerRow } },
            { { "tool_type", "tap" },
              { "thread_size", "M5" },
              { "tap_depth_mm", kTapDepthMm },
              { "pass_count",   2 * holesPerRow } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::linear_rail_seat_compound applied: len={} pitch={} subfeatures={}",
                  in.length_mm, in.pitch_mm, subFeatureCount);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay + geometric fallback) ──────────────────
//
// Geometric signature of a linear rail seat:
//   - 1 prismatic slot on the top face (planar bottom + 2 vertical walls).
//   - 2 rows of -Z axis cylindrical pilot holes, regularly spaced along X
//     (pitch_mm), symmetric about the slot Y centerline.
//
// Heuristic: scan for clustered small-diameter (≤ 3 mm radius) -Z
// cylindrical faces.  If we find ≥ 4 of them with Y values forming exactly
// TWO clusters and X values evenly spaced by a common pitch — match.

namespace {

std::vector<RecognizedFeature> geometric_fallback(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    if (wp.shape().IsNull()) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // Collect -Z cylindrical faces (pilot holes for tapped fasteners).
    struct CylXY { double x, y, r; };
    std::vector<CylXY> piloth;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cy = surf.Cylinder();
        const gp_Dir axDir = cy.Axis().Direction();
        if (std::abs(std::abs(axDir.Z()) - 1.0) > 1e-3) continue;
        if (cy.Radius() > 3.5) continue;  // M5 pilot = 2.1 mm radius
        const gp_Pnt p = cy.Axis().Location();
        piloth.push_back({ p.X(), p.Y(), cy.Radius() });
    }
    if (piloth.size() < 4) return out;

    // Bucket the Y values into two rows.
    std::vector<double> ys;
    for (const auto& h : piloth) ys.push_back(h.y);
    std::sort(ys.begin(), ys.end());
    const double yMid = (ys.front() + ys.back()) / 2.0;
    int nRow0 = 0, nRow1 = 0;
    double yRow0 = 0.0, yRow1 = 0.0;
    int nRow0c = 0, nRow1c = 0;
    for (const auto& h : piloth) {
        if (h.y < yMid) { ++nRow0; yRow0 += h.y; ++nRow0c; }
        else            { ++nRow1; yRow1 += h.y; ++nRow1c; }
    }
    if (nRow0 < 2 || nRow1 < 2) return out;
    if (nRow0 != nRow1) return out;
    yRow0 /= std::max(1, nRow0c);
    yRow1 /= std::max(1, nRow1c);

    // Look for a planar slot floor (Z-normal +Z, intermediate Z).
    bool slotFloorFound = false;
    double zSlotFloor = zMax;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(n.Z() - 1.0) > 1e-3) continue;
        const gp_Pnt c = wp.faceCenter(i);
        if (c.Z() < zMax - 1e-3 && c.Z() > zMin + 1e-3) {
            slotFloorFound = true;
            zSlotFloor = std::min(zSlotFloor, c.Z());
        }
    }
    if (!slotFloorFound) return out;

    // Pitch from sorted X values of row 0.
    std::vector<double> xRow0;
    for (const auto& h : piloth)
        if (h.y < yMid) xRow0.push_back(h.x);
    std::sort(xRow0.begin(), xRow0.end());
    double pitch = 0.0;
    if (xRow0.size() >= 2) {
        pitch = xRow0[1] - xRow0[0];
    }

    const double railCenterline = (yRow0 + yRow1) / 2.0;
    const double length_estimate = (xRow0.size() >= 2)
        ? (xRow0.back() - xRow0.front() + pitch)
        : (xMax - xMin);

    json recovered = {
        { "length_mm",          length_estimate },
        { "rail_centerline_mm", railCenterline },
        { "pitch_mm",           pitch },
        { "start_x_mm",         xRow0.empty() ? 0.0 : xRow0.front() - pitch * 0.5 },
    };
    json matched = {
        { "source",       "geometric_fallback" },
        { "pilot_count",  static_cast<int>(piloth.size()) },
        { "row0_count",   nRow0 },
        { "row1_count",   nRow1 },
        { "slot_floor_z", zSlotFloor },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.65, matched });
    return out;
}

}  // namespace

// Compound recognition: search the workpiece's feature history for a
// signature whose pattern.kind == kSkillId.  Confidence 1.0 when found
// via metadata replay (we trust the recorded params verbatim).  If no
// metadata is present (e.g. after STEP roundtrip), fall back to
// geometric_fallback() which detects the rail-slot + dual-row pattern.
std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",            "metadata_replay" },
            { "subfeature_count",  f.pattern.value("subfeature_count", 0) },
        };
        out.push_back(r);
    }
    if (out.empty()) {
        out = geometric_fallback(wp);
    }
    return out;
}

}  // namespace koocadcam::skill::linear_rail_seat_compound
