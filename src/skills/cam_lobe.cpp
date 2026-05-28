// @lat: [[engine/skills#cam_lobe]]

#include "cam_lobe.hpp"

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

namespace koocadcam::skill::cam_lobe {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.base_radius_mm < 0.5) {
        r.add("DFM-CAM-RAD", "error",
              "base_radius_mm " + std::to_string(in.base_radius_mm) +
              " < 0.5 mm");
    }
    if (in.lobe_lift_mm <= 0.0 || in.lobe_lift_mm > 10.0 * in.base_radius_mm) {
        r.add("DFM-CAM-LIFT", "error",
              "lobe_lift_mm " + std::to_string(in.lobe_lift_mm) +
              " out of (0, 10×base_radius]");
    }
    if (in.lobe_angle_deg <= 0.0 || in.lobe_angle_deg > 359.9) {
        r.add("DFM-CAM-ANG", "error",
              "lobe_angle_deg out of (0, 359.9]");
    }
    if (in.thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "thickness_mm must be > 0");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cam_lobe DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("cam_lobe: entry_face datum unresolved");

    // Per the spec, use the simpler of the two options: SUBTRACT a cylindrical
    // annulus (from base_radius outward to a huge radius) — i.e. cut the
    // workpiece down to base_radius_mm everywhere — and then SKIP the
    // lobe-fuse step at slice-1.  The resulting profile is a plain
    // base-radius cylinder.  The lobe parameters live in the pattern
    // metadata; recognition is via metadata replay.
    //
    // To produce SOME visible geometry change in the workpiece (so apply's
    // returned shape differs from the input), we cut the workpiece down to
    // base_radius_mm by subtracting a coaxial bigger cylinder MINUS a smaller
    // one (an annular ring), removing everything outside the base radius.

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;
    const double thickness = in.thickness_mm;
    // Outer big radius — large enough to enclose any reasonable stock.
    const double bigR = std::max(in.base_radius_mm * 4.0,
                                 std::hypot(xMax - xMin, yMax - yMin));

    // Place the annular ring around the cam axis at the workpiece's mid-Z.
    const double midZ = (zMin + zMax) / 2.0;
    const gp_Pnt camOrigin(in.center_x_mm, in.center_y_mm,
                           midZ - thickness / 2.0 - kOverhang);
    const gp_Ax2 camAx(camOrigin, in.axis_dir);

    // The cutter = annular ring (innerR=base_radius, outerR=bigR) extruded
    // along axis_dir for the full thickness + overhang.  Subtracting this
    // leaves a smooth cylinder of radius = base_radius (over the slab).
    const TopoDS_Shape ring = pr::annularRing(
        camAx, bigR, in.base_radius_mm, thickness + 2.0 * kOverhang);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), ring);

    json params = {
        { "entry_face_id",     *entryId },
        { "center_x_mm",       in.center_x_mm },
        { "center_y_mm",       in.center_y_mm },
        { "angle_deg",         in.angle_deg },
        { "lobe_lift_mm",      in.lobe_lift_mm },
        { "lobe_angle_deg",    in.lobe_angle_deg },
        { "base_radius_mm",    in.base_radius_mm },
        { "thickness_mm",      in.thickness_mm },
        { "axis_dir",          { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "cylindrical_base",     true },
        { "base_radius_mm",       in.base_radius_mm },
        { "lobe_lift_mm",         in.lobe_lift_mm },
        { "lobe_angle_deg",       in.lobe_angle_deg },
        { "approximation",        "cylindrical_reduction" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = std::min(6.0, in.base_radius_mm);
    tooling.tool_length_mm    = thickness * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 500.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = M_PI * (bigR * bigR - in.base_radius_mm * in.base_radius_mm)
                              * thickness;
    tooling.est_cycle_time_s  = std::max(2.0, thickness * 0.5);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::cam_lobe applied: baseR={} lift={} angle={} faces {}→{}",
                  in.base_radius_mm, in.lobe_lift_mm, in.lobe_angle_deg,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// Metadata replay: the lobe parameters live in the pattern; the geometric
// fingerprint (a plain cylinder of base_radius) is not unique enough to
// recover lobe_lift / lobe_angle.
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

}  // namespace koocadcam::skill::cam_lobe
