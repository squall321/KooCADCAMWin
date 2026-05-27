// @lat: [[engine/skills#counterbore]]

#include "counterbore.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::counterbore {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pilot_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "counterbore pilot diameter " + std::to_string(in.pilot_dia_mm) +
              " mm < min 0.8 mm");
    }
    if (in.pilot_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "counterbore pilot diameter must be > 0");
    }
    if (in.seat_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "counterbore seat diameter must be > 0");
    }
    if (in.pilot_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "counterbore pilot depth must be > 0");
    }
    if (in.seat_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "counterbore seat depth must be > 0");
    }
    if (in.seat_dia_mm <= in.pilot_dia_mm) {
        r.add("DFM-COUNTERBORE-GEOM", "error",
              "counterbore seat dia " + std::to_string(in.seat_dia_mm) +
              " must be > pilot dia " + std::to_string(in.pilot_dia_mm) +
              " (else feature is not a counterbore)");
    }
    if (in.seat_depth_mm >= in.pilot_depth_mm) {
        r.add("DFM-COUNTERBORE-GEOM", "error",
              "counterbore seat depth " + std::to_string(in.seat_depth_mm) +
              " must be < pilot depth " + std::to_string(in.pilot_depth_mm) +
              " (else feature is not a counterbore)");
    }
    const double ratio = (in.pilot_dia_mm > 0.0) ? (in.pilot_depth_mm / in.pilot_dia_mm) : 0.0;
    if (ratio > 8.0) {
        r.add("DFM-PECK", "warning",
              "pilot depth/dia ratio " + std::to_string(ratio) +
              " > 8 — peck drilling recommended (tool length / chip evacuation)");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    // 1) DFM gate
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "counterbore DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // 2) Resolve entry face datum
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("counterbore: entry_face datum unresolved");

    // 3) Compute drill geometry (same pattern as drill_hole)
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // toolStart: a point on (or just outside) the entry plane, on the axis.
    gp_Pnt toolStart(
        in.position_x_mm - adir.X() * (bboxDiag + kEntryOverhang),
        in.position_y_mm - adir.Y() * (bboxDiag + kEntryOverhang),
        (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kEntryOverhang));

    // Common-case shortcut: straight-down drilling on a Z-faced block.
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kEntryOverhang);
        }
    }

    // 4) Build the two cutters.  Both share the same gp_Ax2 axis.
    const gp_Ax2 toolAx(toolStart, adir);

    // Pilot — goes the full pilot depth (plus overhang for clean entry).
    const TopoDS_Shape pilotTool =
        pr::cylinder(toolAx, in.pilot_dia_mm / 2.0, in.pilot_depth_mm + kEntryOverhang);

    // Seat — goes only seat_depth (plus overhang).
    const TopoDS_Shape seatTool =
        pr::cylinder(toolAx, in.seat_dia_mm / 2.0, in.seat_depth_mm + kEntryOverhang);

    // 5) Pilot and seat are CONCENTRIC overlapping cylinders.  OCCT's
    //    BRepAlgoAPI_Cut on a compound containing overlapping solids can
    //    miss interior overlap; fuse them into a single connected cutter
    //    first, then perform a single cut.
    const TopoDS_Shape fusedCutter = pr::fuse(pilotTool, seatTool);
    const TopoDS_Shape newShape    = pr::cut(wp.shape(), fusedCutter);

    // 6) Build signature
    json params = {
        { "entry_face_kind", "resolved_id" },
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "pilot_dia_mm",    in.pilot_dia_mm },
        { "pilot_depth_mm",  in.pilot_depth_mm },
        { "seat_dia_mm",     in.seat_dia_mm },
        { "seat_depth_mm",   in.seat_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     2 },             // pilot + seat
        { "step_annular_face_present",  true },          // the planar annulus at seat depth
        { "bottom_planar_face_present", true },          // pilot's blind bottom
        { "pilot_dia_mm",               in.pilot_dia_mm },
        { "seat_dia_mm",                in.seat_dia_mm },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    // CAM exec: typically a dedicated counterbore tool, or pilot drill +
    // endmill / cbore for the seat.  We report it as a counterbore tool;
    // the "extra" field encodes the alternative two-tool sequence.
    tooling.tool_type        = "counterbore";
    tooling.tool_dia_mm      = in.seat_dia_mm;        // largest engagement
    tooling.tool_length_mm   = in.pilot_depth_mm * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double pilotR = in.pilot_dia_mm / 2.0;
    const double seatR  = in.seat_dia_mm  / 2.0;
    tooling.stock_removed_mm3 =
        M_PI * pilotR * pilotR * in.pilot_depth_mm +
        M_PI * (seatR * seatR - pilotR * pilotR) * in.seat_depth_mm;
    tooling.est_cycle_time_s = std::max(1.0, in.pilot_depth_mm / 50.0);
    tooling.extra = {
        { "two_tool_sequence", {
            { { "tool_type", "drill" },   { "tool_dia_mm", in.pilot_dia_mm } },
            { { "tool_type", "endmill" }, { "tool_dia_mm", in.seat_dia_mm  } },
        } }
    };

    FeatureSignature sig {
        kSkillId,
        params,
        pattern,
        tooling,
    };

    // 7) Build new workpiece, register feature
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::counterbore applied: pilot {}x{}, seat {}x{}, faces {}→{}",
                  in.pilot_dia_mm, in.pilot_depth_mm,
                  in.seat_dia_mm, in.seat_depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern matching strategy:
//   1. Collect every cylindrical face into a list with its (radius, axis,
//      top center along axis, bottom center along axis).
//   2. For every pair (A, B) sharing the same axis (parallel direction +
//      shared infinite-line within tolerance):
//        - The one with LARGER radius and shallower extent from entry side
//          is the SEAT.
//        - The other (smaller, deeper) is the PILOT.
//   3. Recover parameters: pilot_dia = 2·r_small, seat_dia = 2·r_large,
//      seat_depth = |seat top − seat bottom|, pilot_depth = |seat top −
//      pilot bottom|.

namespace {

struct CylInfo
{
    int        faceIdx = -1;
    double     radius  = 0.0;
    gp_Ax1     axis;
    gp_Pnt     topCenter;    // along axis direction, "entry" side
    gp_Pnt     botCenter;    // along axis direction, deeper side
    double     length = 0.0;
};

// Collect cylindrical faces with their bounding-circle endpoints.
std::vector<CylInfo> collectCylinders(const Workpiece& wp)
{
    std::vector<CylInfo> out;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();

        CylInfo info;
        info.faceIdx = fIdx;
        info.radius  = cyl.Radius();
        info.axis    = cyl.Axis();

        // Find circular edges on this face, along the same axis.
        std::vector<gp_Pnt> centers;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(info.axis.Direction())) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - info.radius) > 1e-3) continue;
            centers.push_back(c.Location());
        }
        if (centers.size() < 2) continue;

        const gp_Dir adir = info.axis.Direction();
        auto projOnAxis = [&](const gp_Pnt& p) {
            return (p.X() - info.axis.Location().X()) * adir.X() +
                   (p.Y() - info.axis.Location().Y()) * adir.Y() +
                   (p.Z() - info.axis.Location().Z()) * adir.Z();
        };
        auto cmp = [&](const gp_Pnt& a, const gp_Pnt& b) {
            return projOnAxis(a) < projOnAxis(b);
        };
        const auto minIt = std::min_element(centers.begin(), centers.end(), cmp);
        const auto maxIt = std::max_element(centers.begin(), centers.end(), cmp);
        // The "top" (entry) center is the one against the drilling
        // direction (smaller projection along axis_into-material), so
        // higher in workpiece-Z for a top-face drill.  We treat axis
        // direction as "into material", so botCenter (deeper) has
        // LARGER projection along axis direction.
        info.botCenter = *maxIt;
        info.topCenter = *minIt;
        info.length    = info.botCenter.Distance(info.topCenter);
        out.push_back(info);
    }
    return out;
}

// Two cylinders share an axis (infinite-line) within tolerance?
bool sameAxis(const CylInfo& a, const CylInfo& b, double angTolDeg = 0.5, double posTolMm = 1e-3)
{
    const gp_Dir da = a.axis.Direction();
    const gp_Dir db = b.axis.Direction();
    const double dot = std::abs(da.X() * db.X() + da.Y() * db.Y() + da.Z() * db.Z());
    if (dot < std::cos(angTolDeg * M_PI / 180.0)) return false;
    // Position check: distance from b.axis.Location() to the infinite line
    // through a.axis.Location() along da.
    const gp_Pnt& pa = a.axis.Location();
    const gp_Pnt& pb = b.axis.Location();
    gp_Vec v(pa, pb);
    gp_Vec axV(da.X(), da.Y(), da.Z());
    gp_Vec perp = v - axV * v.Dot(axV);
    return perp.Magnitude() < posTolMm;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto cyls = collectCylinders(wp);
    if (cyls.size() < 2) return out;

    std::vector<bool> consumed(cyls.size(), false);

    for (size_t i = 0; i < cyls.size(); ++i) {
        if (consumed[i]) continue;
        for (size_t j = i + 1; j < cyls.size(); ++j) {
            if (consumed[j]) continue;
            if (!sameAxis(cyls[i], cyls[j])) continue;

            const CylInfo& small = (cyls[i].radius < cyls[j].radius) ? cyls[i] : cyls[j];
            const CylInfo& large = (cyls[i].radius < cyls[j].radius) ? cyls[j] : cyls[i];

            if (std::abs(large.radius - small.radius) < 1e-3) continue;  // same dia

            // Determine the drilling direction (entry → deeper).
            // The seat (large) must be on the entry side.  Sample the
            // axis direction "into material" by picking the orientation
            // that places `large` shallower than `small`.
            const gp_Dir adir = small.axis.Direction();
            auto proj = [&](const gp_Pnt& p) {
                return (p.X() - small.axis.Location().X()) * adir.X() +
                       (p.Y() - small.axis.Location().Y()) * adir.Y() +
                       (p.Z() - small.axis.Location().Z()) * adir.Z();
            };
            // Pick the entry side: between the four endpoints
            // {large.topCenter, large.botCenter, small.topCenter,
            // small.botCenter}, the entry is the SHALLOWEST along axis_dir.
            const double entryProj = std::min({
                proj(large.topCenter), proj(large.botCenter),
                proj(small.topCenter), proj(small.botCenter)
            });

            const double largeEntryProj = std::min(proj(large.topCenter), proj(large.botCenter));
            if (std::abs(largeEntryProj - entryProj) > 1e-3) {
                // Large is NOT on entry — not a counterbore arrangement.
                continue;
            }

            const double smallEntryProj = std::min(proj(small.topCenter), proj(small.botCenter));
            // Seat depth = how far the large extends from entry.
            const double seatDepth   = std::abs(proj(large.topCenter) - proj(large.botCenter));
            // Pilot depth: total pilot extent from entry to the small's
            // deeper end.  Small.topCenter sits at seatDepth (right where
            // seat ends); pilot ends at smallBotCenter.
            const double smallDeepProj = std::max(proj(small.topCenter), proj(small.botCenter));
            const double pilotDepth    = smallDeepProj - entryProj;

            // Verify pilot starts at (or beyond) the seat-bottom — i.e.
            // small.topCenter is near large.botCenter along the axis
            // (small's shallow end ≈ large's deep end).
            const double junctionGap = std::abs(smallEntryProj -
                                                (entryProj + seatDepth));
            if (junctionGap > 1e-2) continue;  // not a clean step

            // Recover position from the seat top circle (entry-face circle).
            const gp_Pnt seatEntryCenter =
                (proj(large.topCenter) < proj(large.botCenter)) ? large.topCenter
                                                                : large.botCenter;

            // Confidence heuristic: penalize loose axis alignment / gap.
            double conf = 0.92;
            if (junctionGap > 1e-3) conf -= 0.2;

            json recovered = {
                { "position_x_mm",   seatEntryCenter.X() },
                { "position_y_mm",   seatEntryCenter.Y() },
                { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
                { "pilot_dia_mm",    2.0 * small.radius },
                { "pilot_depth_mm",  pilotDepth },
                { "seat_dia_mm",     2.0 * large.radius },
                { "seat_depth_mm",   seatDepth },
            };
            json matched = {
                { "seat_cyl_face_id",  large.faceIdx },
                { "pilot_cyl_face_id", small.faceIdx },
                { "entry_center", { seatEntryCenter.X(), seatEntryCenter.Y(),
                                    seatEntryCenter.Z() } },
            };
            out.push_back(RecognizedFeature{
                kSkillId, recovered, conf, matched
            });
            consumed[i] = consumed[j] = true;
            break;
        }
    }
    return out;
}

}  // namespace koocadcam::skill::counterbore
