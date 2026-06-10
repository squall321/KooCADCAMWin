// @lat: [[engine/skills#keyway_external]]

#include "keyway_external.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::keyway_external {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.width_mm < 0.8) {
        r.add("DFM-002", "error",
              "keyway_external width_mm " + std::to_string(in.width_mm) +
              " < min 0.8 mm");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "depth_mm must be > 0");
    }
    if (in.length_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "length_mm must be > 0");
    }
    if (in.shaft_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "shaft_radius_mm must be > 0");
    }
    if (in.depth_mm > 0.0 && in.depth_mm >= in.shaft_radius_mm) {
        r.add("DFM-KEY-DEPTH", "error",
              "depth_mm " + std::to_string(in.depth_mm) +
              " >= shaft_radius_mm " + std::to_string(in.shaft_radius_mm) +
              " — would punch through the shaft");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "keyway_external DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("keyway_external: entry_face datum unresolved");

    // Build a box positioned tangent to the shaft OD at `angle_rad`, opening
    // inward (radially).  Slice-1 assumes shaft_axis = +Z; the box's axis-Z
    // runs along the shaft axis, axis-X is radial outward, axis-Y is tangent
    // (CCW).  Origin = shaft surface point - width/2 along tangent.
    const double cA = std::cos(in.angle_rad);
    const double sA = std::sin(in.angle_rad);
    const gp_Dir radialOut(cA, sA, 0.0);
    const gp_Dir tangentCCW(-sA, cA, 0.0);

    // Surface point on the shaft, axial center.
    const gp_Pnt surfacePt(
        in.shaft_center_x_mm + in.shaft_radius_mm * radialOut.X(),
        in.shaft_center_y_mm + in.shaft_radius_mm * radialOut.Y(),
        in.axial_center_mm);

    // Origin: surface_pt − (width/2)·tangent − (length/2)·shaft_axis
    // (so the box runs centered axially and tangentially on the slot).
    // Then push it INWARD by depth_mm so it cuts INTO the shaft.
    const gp_Pnt origin(
        surfacePt.X() - in.width_mm / 2.0 * tangentCCW.X()
                      - in.length_mm / 2.0 * in.shaft_axis.X()
                      - in.depth_mm        * radialOut.X(),
        surfacePt.Y() - in.width_mm / 2.0 * tangentCCW.Y()
                      - in.length_mm / 2.0 * in.shaft_axis.Y()
                      - in.depth_mm        * radialOut.Y(),
        surfacePt.Z() - in.width_mm / 2.0 * tangentCCW.Z()
                      - in.length_mm / 2.0 * in.shaft_axis.Z()
                      - in.depth_mm        * radialOut.Z());

    // gp_Ax2(origin, mainDir = radialOut, xDir = shaft_axis)
    // → DX along shaft_axis (length), DY along tangentCCW (width)
    //   wait: gp_Ax2(O, V, Vx): MainDir=V, XDir=Vx, YDir=V×Vx.
    // For BRepPrimAPI_MakeBox(P, DX, DY, DZ), DX→XDir, DY→YDir, DZ→MainDir.
    // We want: DX = length (along shaft), DY = width (tangent), DZ = 2·depth
    // (radial inward extending outward to ensure clean Boolean).
    const gp_Ax2 boxAx(origin, radialOut, in.shaft_axis);
    const TopoDS_Shape cutter = pr::box(boxAx,
        in.length_mm,       // DX along shaft_axis
        in.width_mm,        // DY along tangentCCW
        2.0 * in.depth_mm); // DZ along radialOut (cut deeper than needed)

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",      *entryId },
        { "shaft_axis",         { in.shaft_axis.X(), in.shaft_axis.Y(), in.shaft_axis.Z() } },
        { "shaft_center_x_mm",  in.shaft_center_x_mm },
        { "shaft_center_y_mm",  in.shaft_center_y_mm },
        { "shaft_radius_mm",    in.shaft_radius_mm },
        { "angle_rad",          in.angle_rad },
        { "axial_center_mm",    in.axial_center_mm },
        { "length_mm",          in.length_mm },
        { "width_mm",           in.width_mm },
        { "depth_mm",           in.depth_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "planar_wall_count",     4 },
        { "floor_planar_face",     true },
        { "on_cylindrical_OD",     true },
        { "width_mm",              in.width_mm },
        { "depth_mm",              in.depth_mm },
        { "length_mm",             in.length_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = in.width_mm;
    tooling.tool_length_mm    = in.depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = in.length_mm * in.width_mm * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(2.0, in.length_mm * 0.05);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::keyway_external applied: L={} W={} D={} faces {}→{}",
                  in.length_mm, in.width_mm, in.depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// Metadata replay — the rectangular external slot on a cylinder is
// topologically a 4-wall + 1-floor pocket; recovery of the *shaft* axis +
// angular position from BREP alone is heuristic-heavy.  Rely on feature
// history for slice-1.
std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 0.80,
            { { "via", "metadata_replay" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::keyway_external
