// @lat: [[engine/skills#keyway_woodruff_arc]]

#include "keyway_woodruff_arc.hpp"

#include "_keyway_table.hpp"
#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::keyway_woodruff_arc {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.shaft_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "shaft_dia_mm must be > 0");
        return r;
    }
    if (in.woodruff_size_id.empty()) {
        r.add("DFM-WOODRUFF-SIZE", "error",
              "woodruff_size_id is empty (e.g. \"404\", \"808\")");
        return r;
    }
    const keyway::WoodruffSpec* w = keyway::findWoodruff(in.woodruff_size_id);
    if (w == nullptr) {
        r.add("DFM-WOODRUFF-SIZE", "error",
              "woodruff_size_id '" + in.woodruff_size_id + "' not in "
              "DIN 6888 / ANSI B17.2 table (9 supported: 404 505 606 "
              "807 808 1008 1009 1011 1212)");
        return r;
    }
    if (in.shaft_dia_mm < 2.0 * w->shaft_keyway_depth_mm + 1.0) {
        r.add("DFM-WOODRUFF-DEPTH", "warning",
              "shaft_dia_mm " + std::to_string(in.shaft_dia_mm) +
              " is very small for Woodruff #" + in.woodruff_size_id +
              " (depth " + std::to_string(w->shaft_keyway_depth_mm) + " mm)");
    }
    // Length analogue for Woodruff = arc diameter; the 1.5×b rule applies
    // implicitly because woodruff key_diameter > 1.5 × b by construction.
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "keyway_woodruff_arc DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }
    const keyway::WoodruffSpec* w = keyway::findWoodruff(in.woodruff_size_id);
    if (w == nullptr)
        throw SkillError("keyway_woodruff_arc: spec lookup failed");

    auto faceId = wp.resolve(in.face_id);
    const int faceIdResolved = faceId.value_or(-1);

    const double cA = std::cos(in.angle_rad);
    const double sA = std::sin(in.angle_rad);
    const gp_Dir radialOut(cA, sA, 0.0);
    const gp_Dir tangentCCW(-sA, cA, 0.0);

    const double shaftR     = in.shaft_dia_mm / 2.0;
    const double cutterR    = w->key_diameter_mm / 2.0;
    const double cutterThk  = w->key_width_mm;
    const double depth      = w->shaft_keyway_depth_mm;

    // Surface point on the shaft OD at the keyway angular position.
    const gp_Pnt surfacePt(
        in.shaft_center_x_mm + shaftR * radialOut.X(),
        in.shaft_center_y_mm + shaftR * radialOut.Y(),
        in.key_position_z_mm);

    // Cutter (Woodruff disc) is a cylinder whose axis is TANGENT to the
    // shaft (perpendicular to both the shaft axis and the radial-out
    // direction).  Its center is positioned along the radial-in direction
    // by (cutterR - depth) so that the lowest point of the cutter sits
    // `depth` below the shaft surface.
    //
    // Cylinder axis direction = tangentCCW (perpendicular to radial-out).
    // Place cylinder origin at:
    //   surfacePt − radialOut · (cutterR − depth) − tangentCCW · (cutterThk/2)
    const gp_Pnt cylOrigin(
        surfacePt.X() - radialOut.X()   * (cutterR - depth)
                      - tangentCCW.X()  * (cutterThk / 2.0),
        surfacePt.Y() - radialOut.Y()   * (cutterR - depth)
                      - tangentCCW.Y()  * (cutterThk / 2.0),
        surfacePt.Z() - radialOut.Z()   * (cutterR - depth)
                      - tangentCCW.Z()  * (cutterThk / 2.0));

    // gp_Ax2(origin, direction): the cylinder is extruded along
    // `direction` for `cutterThk`.  Direction = tangentCCW.
    const gp_Ax2 cylAx(cylOrigin, tangentCCW);
    const TopoDS_Shape cutter = pr::cylinder(cylAx, cutterR, cutterThk);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // Analytical volume of removed material: circular segment of radius R
    // and height `depth` extruded along cutter thickness.
    // Segment area = R^2 · acos((R−h)/R) − (R−h) · sqrt(2·R·h − h^2)
    const double R = cutterR;
    const double h = depth;
    double segArea = 0.0;
    if (h > 0.0 && h <= 2.0 * R) {
        const double ratio = std::clamp((R - h) / R, -1.0, 1.0);
        segArea = R * R * std::acos(ratio)
                  - (R - h) * std::sqrt(std::max(0.0, 2.0 * R * h - h * h));
    }
    const double volume_removed_mm3 = segArea * cutterThk;

    json params = {
        { "face_id_resolved",   faceIdResolved },
        { "shaft_axis",         { in.shaft_axis.X(), in.shaft_axis.Y(), in.shaft_axis.Z() } },
        { "shaft_center_x_mm",  in.shaft_center_x_mm },
        { "shaft_center_y_mm",  in.shaft_center_y_mm },
        { "shaft_dia_mm",       in.shaft_dia_mm },
        { "key_position_z_mm",  in.key_position_z_mm },
        { "angle_rad",          in.angle_rad },
        { "woodruff_size_id",   in.woodruff_size_id },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "keyway_standard",       "DIN_6888_woodruff_arc" },
        { "key_width_mm",          cutterThk },
        { "key_height_mm",         w->key_height_mm },
        { "key_diameter_mm",       w->key_diameter_mm },
        { "key_depth_mm",          depth },
        { "derived_volume_removed", volume_removed_mm3 },
        { "end_geometry",          "arc" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "woodruff_cutter";
    tooling.tool_dia_mm       = w->key_diameter_mm;
    tooling.tool_length_mm    = cutterThk + 10.0;
    tooling.tool_material     = "hss";
    tooling.flute_count       = 8;
    tooling.cutting_speed_sfm = 80.0;
    tooling.feed_per_tooth_mm = 0.02;
    tooling.stock_removed_mm3 = volume_removed_mm3;
    tooling.est_cycle_time_s  = std::max(2.5, depth * 1.5);
    tooling.extra = {
        { "standard",         "DIN 6888 / ANSI B17.2 (Woodruff key seat)" },
        { "woodruff_size_id", in.woodruff_size_id },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::keyway_woodruff_arc applied: dia={} #{} b={} d={} depth={}",
                  in.shaft_dia_mm, in.woodruff_size_id, cutterThk,
                  w->key_diameter_mm, depth);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& feat : wp.features()) {
        if (feat.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, feat.params, 1.0,
            { { "source", "metadata_replay" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback: find a cylindrical face whose axis is
    // perpendicular to the dominant shaft axis (= a Woodruff trough),
    // match its radius to a Woodruff key_diameter.
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface as(wp.face(i));
        const gp_Cylinder c = as.Cylinder();
        const double cutterDia = 2.0 * c.Radius();
        // Match against woodruff table.
        const keyway::WoodruffSpec* match = nullptr;
        double bestErr = 1.0;
        for (const auto& w : keyway::kWoodruff) {
            const double err = std::abs(w.key_diameter_mm - cutterDia);
            if (err < bestErr) { bestErr = err; match = &w; }
        }
        if (match == nullptr || bestErr > 0.5) continue;
        // Axis must be ~perpendicular to Z.
        const gp_Dir d = c.Axis().Direction();
        if (std::abs(d.Z()) > 0.1) continue;
        json recovered = {
            { "shaft_dia_mm",     0.0 },   // unknown from geometry alone
            { "woodruff_size_id", std::string(match->key_id) },
        };
        json matched = {
            { "source",         "geometric_fallback" },
            { "cyl_face_id",    i },
            { "cutter_dia_mm",  cutterDia },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
        break;
    }
    return out;
}

}  // namespace koocadcam::skill::keyway_woodruff_arc
