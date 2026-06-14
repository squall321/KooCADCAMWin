// @lat: [[engine/skills#pin_and_diamond_locating_set]]

#include "pin_and_diamond_locating_set.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::pin_and_diamond_locating_set {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// DFM cites ASME Y14.5 (3-2-1 location principle), ANSI B5.50 (fixture
// design), and Carr Lane "Jig and Fixture Handbook" for diamond-pin slot
// dimensions.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pin_dia_mm < 4.0) {
        r.add("DFM-LOCATE-DIA", "error",
              "pin_and_diamond_locating_set: pin_dia " +
              std::to_string(in.pin_dia_mm) +
              " mm < 4 mm (Carr Lane min for shear strength)");
    }
    if (in.pin_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "pin_and_diamond_locating_set: pin_dia " +
              std::to_string(in.pin_dia_mm) + " mm < 0.8 mm");
    }
    if (in.hole_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pin_and_diamond_locating_set: hole_depth must be > 0");
    }
    // Hole engagement ≥ 1.5 × pin Ø (Carr Lane).
    if (in.hole_depth_mm < 1.5 * in.pin_dia_mm) {
        r.add("DFM-LOCATE-ENGAGEMENT", "warning",
              "pin_and_diamond_locating_set: hole_depth " +
              std::to_string(in.hole_depth_mm) +
              " < 1.5 × pin_dia (Carr Lane recommendation)");
    }
    // Connection length ≥ 3 × pin Ø.
    const double dx = in.diamond_x_mm - in.round_x_mm;
    const double dy = in.diamond_y_mm - in.round_y_mm;
    const double L  = std::sqrt(dx * dx + dy * dy);
    if (L < 3.0 * in.pin_dia_mm) {
        r.add("DFM-LOCATE-SPACING", "error",
              "pin_and_diamond_locating_set: connection length " +
              std::to_string(L) + " < 3 × pin_dia (" +
              std::to_string(3.0 * in.pin_dia_mm) + ")");
    }
    if (L < 1e-6) {
        r.add("DFM-INPUT", "error",
              "pin_and_diamond_locating_set: round and diamond positions coincide");
    }
    // Slot extension must be > 0 (else this is two round pins, not a 3-2-1
    // set) and ≤ pin Ø (else too sloppy).
    if (in.diamond_slot_extension_mm < 0.2) {
        r.add("DFM-LOCATE-SLOT", "error",
              "pin_and_diamond_locating_set: diamond_slot_extension " +
              std::to_string(in.diamond_slot_extension_mm) +
              " < 0.2 mm minimum");
    }
    if (in.diamond_slot_extension_mm > in.pin_dia_mm) {
        r.add("DFM-LOCATE-SLOT", "error",
              "pin_and_diamond_locating_set: slot_extension " +
              std::to_string(in.diamond_slot_extension_mm) +
              " > pin_dia — too sloppy");
    }
    if (in.chamfer_mm < 0.3) {
        r.add("DFM-LOCATE-CHAMFER", "warning",
              "pin_and_diamond_locating_set: chamfer " +
              std::to_string(in.chamfer_mm) +
              " < 0.3 mm recommended lead-in");
    }
    // Plate thickness check.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness > 0.0 && thickness < in.hole_depth_mm + 1.0) {
        r.add("DFM-LOCATE-THICKNESS", "error",
              "pin_and_diamond_locating_set: plate thickness " +
              std::to_string(thickness) + " < hole_depth + 1 mm");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Chain (axis_dir = -Z):
//   1. CUT round-pin receptacle cylinder       at (round_x, round_y)
//   2. CUT diamond receptacle = capsule slot   at (diamond_x, diamond_y)
//      formed as 2 round end caps (pin_dia)
//      fused with a rectangular bar between, oriented along the round↔diamond
//      connection axis, total length = pin_dia + 2 · slot_extension.
//   3. CUT chamfer cone at round entry
//   4. CUT chamfer cone at diamond entry (over each end cap)

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pin_and_diamond_locating_set DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError(
        "pin_and_diamond_locating_set: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    if (std::abs(adir.X()) > 1e-6 || std::abs(adir.Y()) > 1e-6) {
        throw SkillError(
            "pin_and_diamond_locating_set: only axis-aligned axis_dir supported");
    }
    const double entryZ = (adir.Z() < 0) ? zMax : zMin;
    const double cutLen = in.hole_depth_mm + 2.0 * kOverhang;

    // Helper: produce a cylindrical cutter at (cx, cy, entryZ).
    auto makeCyl = [&](double cx, double cy, double r) {
        const gp_Pnt p(cx, cy, entryZ - adir.Z() * kOverhang);
        const gp_Ax2 ax(p, adir);
        return pr::cylinder(ax, r, cutLen);
    };

    const double rPin = in.pin_dia_mm / 2.0;
    const double dx = in.diamond_x_mm - in.round_x_mm;
    const double dy = in.diamond_y_mm - in.round_y_mm;
    const double L  = std::sqrt(dx * dx + dy * dy);
    const double ux = dx / L;   // unit vector round→diamond
    const double uy = dy / L;
    // Perpendicular in-plane unit vector
    const double px = -uy;
    const double py =  ux;

    // ── Sub-feature 1: round receptacle ──────────────────────────────────
    TopoDS_Shape roundCyl = makeCyl(in.round_x_mm, in.round_y_mm, rPin);

    // ── Sub-feature 2: diamond receptacle (capsule slot) ────────────────
    // Two end caps at (diamond_x ± slot_extension · u) of radius rPin,
    // plus a bar between them of width pin_dia and length 2·slot_extension.
    const double slotEnd1x = in.diamond_x_mm + ux * in.diamond_slot_extension_mm;
    const double slotEnd1y = in.diamond_y_mm + uy * in.diamond_slot_extension_mm;
    const double slotEnd2x = in.diamond_x_mm - ux * in.diamond_slot_extension_mm;
    const double slotEnd2y = in.diamond_y_mm - uy * in.diamond_slot_extension_mm;

    TopoDS_Shape capA = makeCyl(slotEnd1x, slotEnd1y, rPin);
    TopoDS_Shape capB = makeCyl(slotEnd2x, slotEnd2y, rPin);

    // Rectangular bar between the two end caps: width pin_dia, length =
    // 2*slot_extension, oriented along (ux, uy).  Build via gp_Ax2 box.
    TopoDS_Shape slotCutter = pr::fuse(capA, capB);
    if (in.diamond_slot_extension_mm > 1e-6) {
        const double barLen = 2.0 * in.diamond_slot_extension_mm;
        const double barWid = in.pin_dia_mm;
        // gp_Ax2(P, V, Vx): V = local Z (the depth direction), Vx = local X
        // (the bar length direction).  We want DX = barLen along (ux, uy),
        // DY = barWid along (px, py), DZ = cut depth along adir.
        const gp_Pnt origin(
            in.diamond_x_mm - barLen / 2.0 * ux - barWid / 2.0 * px,
            in.diamond_y_mm - barLen / 2.0 * uy - barWid / 2.0 * py,
            entryZ - adir.Z() * kOverhang);
        const gp_Dir mainZ = adir;       // axial / depth direction
        const gp_Dir xLoc(ux, uy, 0.0);  // bar length direction
        const gp_Ax2 ax(origin, mainZ, xLoc);
        const TopoDS_Shape bar = pr::box(ax, barLen, barWid, cutLen);
        slotCutter = pr::fuse(slotCutter, bar);
    }

    // ── Sub-feature 3: round chamfer cone ───────────────────────────────
    TopoDS_Shape chamferRound;
    if (in.chamfer_mm > 1e-6) {
        const gp_Pnt p(in.round_x_mm, in.round_y_mm,
                       entryZ - adir.Z() * 0.0);
        const gp_Ax2 ax(p, adir);
        const double rTop = rPin + in.chamfer_mm;
        const double rBot = rPin;
        chamferRound = pr::coneFrustum(ax, rTop, rBot, in.chamfer_mm);
    }

    // ── Sub-feature 4: diamond chamfer cones (one on each cap) ──────────
    TopoDS_Shape chamferDiaA, chamferDiaB;
    if (in.chamfer_mm > 1e-6) {
        const double rTop = rPin + in.chamfer_mm;
        const double rBot = rPin;
        {
            const gp_Pnt p(slotEnd1x, slotEnd1y, entryZ);
            const gp_Ax2 ax(p, adir);
            chamferDiaA = pr::coneFrustum(ax, rTop, rBot, in.chamfer_mm);
        }
        {
            const gp_Pnt p(slotEnd2x, slotEnd2y, entryZ);
            const gp_Ax2 ax(p, adir);
            chamferDiaB = pr::coneFrustum(ax, rTop, rBot, in.chamfer_mm);
        }
    }

    TopoDS_Shape fused = pr::fuse(roundCyl, slotCutter);
    if (!chamferRound.IsNull()) fused = pr::fuse(fused, chamferRound);
    if (!chamferDiaA.IsNull())  fused = pr::fuse(fused, chamferDiaA);
    if (!chamferDiaB.IsNull())  fused = pr::fuse(fused, chamferDiaB);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "entry_face_id",            *entryId },
        { "axis_dir",                 { adir.X(), adir.Y(), adir.Z() } },
        { "round_x_mm",               in.round_x_mm },
        { "round_y_mm",               in.round_y_mm },
        { "diamond_x_mm",             in.diamond_x_mm },
        { "diamond_y_mm",             in.diamond_y_mm },
        { "pin_dia_mm",               in.pin_dia_mm },
        { "hole_depth_mm",            in.hole_depth_mm },
        { "diamond_slot_extension_mm", in.diamond_slot_extension_mm },
        { "chamfer_mm",               in.chamfer_mm },
    };
    const int subCount = (in.chamfer_mm > 1e-6) ? 4 : 2;
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      subCount },
        { "pin_dia_mm",            in.pin_dia_mm },
        { "hole_depth_mm",         in.hole_depth_mm },
        { "connection_length_mm",  L },
        { "diamond_slot_extension_mm", in.diamond_slot_extension_mm },
        { "round_center",          { in.round_x_mm,   in.round_y_mm } },
        { "diamond_center",        { in.diamond_x_mm, in.diamond_y_mm } },
        { "axis_dir",              { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "drill;endmill;chamfer";
    tooling.tool_dia_mm      = in.pin_dia_mm;
    tooling.tool_length_mm   = in.hole_depth_mm + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 320.0;
    tooling.feed_per_tooth_mm = 0.04;
    // Removed volume:
    //   round hole + diamond capsule + 4 small chamfer rings
    const double roundVol = M_PI * rPin * rPin * in.hole_depth_mm;
    const double capsuleArea = M_PI * rPin * rPin +
                               2.0 * in.diamond_slot_extension_mm * in.pin_dia_mm;
    const double diamondVol = capsuleArea * in.hole_depth_mm;
    tooling.stock_removed_mm3 = roundVol + diamondVol;
    tooling.est_cycle_time_s  = std::max(2.0, (roundVol + diamondVol) / 1500.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },   { "operation", "round_pin_hole" },
              { "tool_dia_mm", in.pin_dia_mm },
              { "depth_mm",    in.hole_depth_mm } },
            { { "tool_type", "endmill" }, { "operation", "diamond_slot" },
              { "tool_dia_mm", in.pin_dia_mm },
              { "depth_mm",    in.hole_depth_mm },
              { "slot_extension_mm", in.diamond_slot_extension_mm } },
            { { "tool_type", "chamfer" }, { "operation", "entry_lead_in" },
              { "depth_mm",  in.chamfer_mm },
              { "pass_count", 3 } },
        } },
        { "fixture_principle", "3-2-1_locating" },
        { "design_handbook",   "Carr_Lane_Jig_and_Fixture" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::pin_and_diamond_locating_set applied: "
                  "pin Ø{} L={:.2f} slot_ext={}",
                  in.pin_dia_mm, L, in.diamond_slot_extension_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic PRIMARY: identify a round hole + a slot of identical
// radius end caps.  We look for ≥ 3 parallel-axis cylindrical faces with
// the same radius (round + 2 caps of the diamond), grouped by spatial
// arrangement.  Metadata replay FALLBACK.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata first.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" },
                                { "is_compound", true } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // Geometric path: find ≥ 3 cylindrical faces with parallel axes and
    // same radius.  Two of them are the diamond slot caps; one is the
    // round receptacle.
    struct CylRef { gp_Pnt loc; double r; gp_Dir axisDir; };
    std::vector<CylRef> cyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cyl = surf.Cylinder();
        // GROUNDING gate (B1.2, fpscan 2026-06-14): locating pins/receptacles
        // are full-round bores; a pocket's quarter-round CORNER FILLET (~pi/2)
        // is not.  Counting them made this recognizer read mill_rect_pocket's
        // 4 same-radius corner fillets as a pin-and-diamond locating set.
        // Require the cylinder to subtend ≥ ~2.5 rad.
        if (surf.LastUParameter() - surf.FirstUParameter() < 2.5) continue;
        cyls.push_back({ cyl.Axis().Location(),
                         cyl.Radius(),
                         cyl.Axis().Direction() });
    }
    if (cyls.size() < 3) return out;

    // Find a triple with same radius (±1e-3) and parallel axes.
    for (size_t i = 0; i < cyls.size(); ++i) {
        for (size_t j = i + 1; j < cyls.size(); ++j) {
            if (std::abs(cyls[i].r - cyls[j].r) > 1e-3) continue;
            const double dotij = std::abs(cyls[i].axisDir.Dot(cyls[j].axisDir));
            if (dotij < 0.999) continue;
            for (size_t k = j + 1; k < cyls.size(); ++k) {
                if (std::abs(cyls[i].r - cyls[k].r) > 1e-3) continue;
                const double dotik = std::abs(cyls[i].axisDir.Dot(cyls[k].axisDir));
                if (dotik < 0.999) continue;

                // Geometry: two of {i,j,k} are diamond caps (close together
                // — distance 2·slot_extension), the third is the round
                // receptacle (far away from both caps).
                std::vector<int> ids = { static_cast<int>(i),
                                         static_cast<int>(j),
                                         static_cast<int>(k) };
                // Compute pairwise XY distances.
                auto dxyOf = [&](int a, int b) {
                    return std::hypot(cyls[a].loc.X() - cyls[b].loc.X(),
                                      cyls[a].loc.Y() - cyls[b].loc.Y());
                };
                const double d01 = dxyOf(ids[0], ids[1]);
                const double d02 = dxyOf(ids[0], ids[2]);
                const double d12 = dxyOf(ids[1], ids[2]);
                // Pick the pair with the smallest distance — those are the
                // two caps; the remaining one is the round.
                int capA, capB, roundI;
                if      (d01 <= d02 && d01 <= d12) { capA = ids[0]; capB = ids[1]; roundI = ids[2]; }
                else if (d02 <= d01 && d02 <= d12) { capA = ids[0]; capB = ids[2]; roundI = ids[1]; }
                else                                { capA = ids[1]; capB = ids[2]; roundI = ids[0]; }
                const double slotSpan = dxyOf(capA, capB);
                // Sanity: slot span must be < distance from caps to round.
                const double dr1 = dxyOf(capA, roundI);
                const double dr2 = dxyOf(capB, roundI);
                if (slotSpan >= std::min(dr1, dr2)) continue;

                const double slotExt = slotSpan / 2.0;
                const double pinDia  = 2.0 * cyls[i].r;
                const double diamondCx = (cyls[capA].loc.X() + cyls[capB].loc.X()) / 2.0;
                const double diamondCy = (cyls[capA].loc.Y() + cyls[capB].loc.Y()) / 2.0;
                const double connL = std::hypot(
                    cyls[roundI].loc.X() - diamondCx,
                    cyls[roundI].loc.Y() - diamondCy);

                json rp = {
                    { "axis_dir",                 json::array({ 0.0, 0.0, -1.0 }) },
                    { "round_x_mm",               cyls[roundI].loc.X() },
                    { "round_y_mm",               cyls[roundI].loc.Y() },
                    { "diamond_x_mm",             diamondCx },
                    { "diamond_y_mm",             diamondCy },
                    { "pin_dia_mm",               pinDia },
                    { "hole_depth_mm",            10.0 },   // not recoverable
                    { "diamond_slot_extension_mm", slotExt },
                    { "chamfer_mm",               0.3 },
                };
                RecognizedFeature rf;
                rf.skill_id         = kSkillId;
                rf.recovered_params = rp;
                rf.confidence       = 0.75;
                rf.matched_geometry = {
                    { "round_cyl_id", roundI },
                    { "diamond_cap_ids", { capA, capB } },
                    { "connection_length_mm", connL },
                    { "source", "geometric" },
                };
                out.push_back(rf);
                return out;
            }
        }
    }
    return out;
}

}  // namespace koocadcam::skill::pin_and_diamond_locating_set
