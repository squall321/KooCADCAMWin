// @lat: [[engine/skills#rack_tooth_cut]]

#include "rack_tooth_cut.hpp"

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::rack_tooth_cut {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.module_mm < 0.2 || in.module_mm > 50.0) {
        r.add("DFM-GEAR-MOD", "error",
              "rack_tooth_cut module_mm " + std::to_string(in.module_mm) +
              " out of [0.2, 50.0]");
    }
    if (in.pressure_angle_deg < 14.5 || in.pressure_angle_deg > 25.0) {
        r.add("DFM-GEAR-PA", "error",
              "pressure_angle " + std::to_string(in.pressure_angle_deg) +
              " out of [14.5, 25.0] deg");
    }
    if (in.depth_mm <= 0.0) r.add("DFM-INPUT", "error", "depth_mm must be > 0");
    if (in.width_mm <= 0.0) r.add("DFM-INPUT", "error", "width_mm must be > 0");
    if (in.length_mm <= 0.0) r.add("DFM-INPUT", "error", "length_mm must be > 0");
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "rack_tooth_cut DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("rack_tooth_cut: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    // Local frame:
    //   x_loc = length_dir (along rack length)
    //   y_loc = perp horizontal (= z × x)
    //   z_loc = -axis_dir (up; cut goes "down" along axis_dir)
    const gp_Dir x_loc = in.length_dir;
    // Compute y_loc = z × x assuming z_loc = +Z for slice-1 simplification.
    const gp_Dir y_loc(-x_loc.Y(), x_loc.X(), 0.0);

    // Origin = (position_x - width/2·y, position_y - width/2·y, zMax - depth)
    const gp_Pnt origin(
        in.position_x_mm - in.length_mm / 2.0 * x_loc.X() - in.width_mm / 2.0 * y_loc.X(),
        in.position_y_mm - in.length_mm / 2.0 * x_loc.Y() - in.width_mm / 2.0 * y_loc.Y(),
        zMax - in.depth_mm);

    const gp_Ax2 boxAx(origin, gp_Dir(0.0, 0.0, 1.0), x_loc);
    const TopoDS_Shape cutter = pr::box(boxAx,
        in.length_mm, in.width_mm, in.depth_mm + kOverhang);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",       *entryId },
        { "position_x_mm",       in.position_x_mm },
        { "position_y_mm",       in.position_y_mm },
        { "module_mm",           in.module_mm },
        { "pressure_angle_deg",  in.pressure_angle_deg },
        { "depth_mm",            in.depth_mm },
        { "width_mm",            in.width_mm },
        { "length_mm",           in.length_mm },
        { "length_dir",          { x_loc.X(), x_loc.Y(), x_loc.Z() } },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "flank_planar_count",    2 },
        { "root_planar_face",      true },
        { "module_mm",             in.module_mm },
        { "pressure_angle_deg",    in.pressure_angle_deg },
        { "pitch_mm",              M_PI * in.module_mm },
        { "width_mm",              in.width_mm },
        { "depth_mm",              in.depth_mm },
        { "approximation",         "rect_notch" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = std::min(in.width_mm, 6.0);
    tooling.tool_length_mm    = in.depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = in.length_mm * in.width_mm * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.5, in.length_mm * 0.04);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::rack_tooth_cut applied: module={} faces {}→{}",
                  in.module_mm, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// Metadata replay (same reason as gear_tooth_cut — rectangular notch is
// topologically indistinguishable from a plain milled slot in slice-1).
std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 0.85,
            { { "via", "metadata_replay" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::rack_tooth_cut
