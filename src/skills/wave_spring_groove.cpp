// @lat: [[engine/skills#wave_spring_groove]]

#include "wave_spring_groove.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::wave_spring_groove {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// Smalley C600 / Rotor Clip wave-spring catalog gates.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.mean_dia <= 0.0 || in.wave_spring_id <= 0.0
        || in.wave_spring_height <= 0.0) {
        r.add("DFM-INPUT", "error",
              "wave_spring_groove: mean_dia, wave_spring_id, wave_spring_height "
              "must all be > 0");
        return r;
    }

    if (in.mean_dia < 5.0 || in.mean_dia > 250.0) {
        r.add("DFM-WAVE-RANGE", "error",
              "wave_spring_groove: mean_dia " + std::to_string(in.mean_dia) +
              " mm outside Smalley C600 catalog [5, 250]");
    }

    if (in.wave_spring_id >= in.mean_dia) {
        r.add("DFM-WAVE-GEOM", "error",
              "wave_spring_groove: wave_spring_id " +
              std::to_string(in.wave_spring_id) +
              " must be < mean_dia " + std::to_string(in.mean_dia) +
              " (inner-cyl retention: spring sits IN the bore)");
        return r;
    }

    const double width = in.wave_spring_height * 1.05;
    if (width < 0.5) {
        r.add("DFM-WAVE-WIDTH", "error",
              "wave_spring_groove: derived width " + std::to_string(width) +
              " mm < min 0.5 mm");
    }

    const double depth = (in.mean_dia - in.wave_spring_id) / 2.0;
    if (depth < 0.2) {
        r.add("DFM-WAVE-DEPTH", "error",
              "wave_spring_groove: derived depth " + std::to_string(depth) +
              " mm < min 0.2 mm (Smalley engagement minimum)");
    }
    if (depth > in.mean_dia * 0.05) {
        r.add("DFM-WAVE-DEPTH", "error",
              "wave_spring_groove: derived depth " + std::to_string(depth) +
              " > mean_dia × 0.05 (" + std::to_string(in.mean_dia * 0.05) +
              ") — over-deep groove weakens bore wall");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "wave_spring_groove DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Derived geometry
    const double meanR = in.mean_dia / 2.0;
    const double idR   = in.wave_spring_id / 2.0;
    const double width = in.wave_spring_height * 1.05;
    const double depth = (in.mean_dia - in.wave_spring_id) / 2.0;
    const double reliefR = depth * 0.15;             // 15% of depth

    // Place the cutter centered axially at `axial_position_mm` along the
    // bore axis; cutter is concentric with bore_axis.
    const gp_Dir baseDir = in.bore_axis.Direction();
    const gp_Pnt basePt  = in.bore_axis.Location();

    const gp_Pnt cutterStart(
        basePt.X() + baseDir.X() * (in.axial_position_mm - width / 2.0),
        basePt.Y() + baseDir.Y() * (in.axial_position_mm - width / 2.0),
        basePt.Z() + baseDir.Z() * (in.axial_position_mm - width / 2.0));

    // gp_Ax2(P, V): V = LOCAL Z (axial) — cutter axis is bore axis.
    const gp_Ax2 ax(cutterStart, baseDir);

    constexpr double kOverhang = 0.02;

    // Sub-feature 1: main slot — annular band cutting OUTWARD from bore
    // wall.  We model as an outer cylinder of radius meanR fused with a
    // thin relief sub-cyl of radius (meanR + reliefR).
    const TopoDS_Shape mainSlot =
        pr::cylinder(ax, meanR, width + 2.0 * kOverhang);

    // Sub-feature 2: bottom relief cylinder — extends radially outward by
    // reliefR for chip evacuation, at the same axial position.  Slightly
    // narrower in axial extent (only at one side, per Smalley §4.2).
    const gp_Pnt reliefStart(
        basePt.X() + baseDir.X() * (in.axial_position_mm - width / 2.0 + width * 0.4),
        basePt.Y() + baseDir.Y() * (in.axial_position_mm - width / 2.0 + width * 0.4),
        basePt.Z() + baseDir.Z() * (in.axial_position_mm - width / 2.0 + width * 0.4));
    const gp_Ax2 reliefAx(reliefStart, baseDir);
    const TopoDS_Shape reliefCyl =
        pr::cylinder(reliefAx, meanR + reliefR, width * 0.25 + kOverhang);

    // Fuse concentric overlapping cutters → one fused cutter → one cut.
    const TopoDS_Shape fusedCutter = pr::fuse(mainSlot, reliefCyl);
    const TopoDS_Shape newShape    = pr::cut(wp.shape(), fusedCutter);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "bore_axis_origin", { basePt.X(), basePt.Y(), basePt.Z() } },
        { "bore_axis_dir",    { baseDir.X(), baseDir.Y(), baseDir.Z() } },
        { "mean_dia",         in.mean_dia },
        { "wave_spring_id",   in.wave_spring_id },
        { "wave_spring_height", in.wave_spring_height },
        { "axial_position_mm", in.axial_position_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           2 },
        { "subfeatures", json::array({
            { { "op", "main_slot_band" },
              { "radius_mm", meanR },     { "width_mm", width } },
            { { "op", "bottom_relief" },
              { "radius_mm", meanR + reliefR },
              { "width_mm", width * 0.25 } },
        }) },
        { "mean_dia",                   in.mean_dia },
        { "wave_spring_id",             in.wave_spring_id },
        { "wave_spring_height",         in.wave_spring_height },
        { "groove_width_mm",            width },
        { "groove_depth_mm",            depth },
        { "relief_radial_mm",           reliefR },
        { "axial_position_mm",          in.axial_position_mm },
        { "cylindrical_face_count",     1 },
        { "planar_sidewall_count",      2 },
        { "axis_dir",                   { baseDir.X(), baseDir.Y(), baseDir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "groove_tool;relief_tool";
    tooling.tool_dia_mm       = width;
    tooling.tool_length_mm    = depth * 1.5 + 3.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 =
        M_PI * (meanR * meanR - idR * idR) * width
      + M_PI * ((meanR + reliefR) * (meanR + reliefR) - meanR * meanR)
              * (width * 0.25);
    tooling.est_cycle_time_s  = std::max(2.0, depth / 20.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "groove_tool" },
              { "width_mm", width }, { "depth_mm", depth } },
            { { "tool_type", "relief_tool" },
              { "radial_mm", reliefR }, { "width_mm", width * 0.25 } },
        } },
        { "feature_standard", "Smalley C600 wave-spring retention groove" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::wave_spring_groove applied: mean_dia={} id={} h={}",
                  in.mean_dia, in.wave_spring_id, in.wave_spring_height);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

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
            { "source", "metadata_replay" },
            { "is_compound", true },
            { "subfeature_count", 2 },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::wave_spring_groove
