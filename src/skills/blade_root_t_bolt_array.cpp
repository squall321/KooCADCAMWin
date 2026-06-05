// @lat: [[engine/skills#blade_root_t_bolt_array]]

#include "blade_root_t_bolt_array.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::blade_root_t_bolt_array {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "blade_root_t_bolt_array: face_id out of range");
    }
    if (in.bolt_count <= 4) {
        r.add("DFM-BOLT-COUNT", "error",
              "blade_root_t_bolt_array: bolt_count " +
              std::to_string(in.bolt_count) +
              " must be > 4 (utility-scale blade root needs ≥ 60)");
    }
    if (in.bolt_pcd_mm <= 0.0 || in.t_bolt_dia_mm <= 0.0 ||
        in.t_bolt_depth_mm <= 0.0 ||
        in.recess_dia_mm <= 0.0 || in.recess_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "blade_root_t_bolt_array: all dimensions must be > 0");
        return r;
    }
    // T-bolt nominal sizes for utility wind: M27/M30/M36.
    if (!(std::abs(in.t_bolt_dia_mm - 27.0) < 0.5 ||
          std::abs(in.t_bolt_dia_mm - 30.0) < 0.5 ||
          std::abs(in.t_bolt_dia_mm - 36.0) < 0.5)) {
        r.add("DFM-TBOLT-DIA", "warning",
              "blade_root_t_bolt_array: t_bolt_dia " +
              std::to_string(in.t_bolt_dia_mm) +
              " mm not in IEC 61400-2 utility set {M27, M30, M36}");
    }
    if (in.recess_dia_mm <= in.t_bolt_dia_mm + 4.0) {
        r.add("DFM-RECESS", "error",
              "blade_root_t_bolt_array: recess_dia must be > "
              "t_bolt_dia + 4 mm (head clearance)");
    }
    // PCD large enough to fit N recesses without overlap.
    const double arcPerBolt = (M_PI * in.bolt_pcd_mm) /
                              static_cast<double>(in.bolt_count);
    if (in.recess_dia_mm >= arcPerBolt) {
        r.add("DFM-PCD-FIT", "error",
              "blade_root_t_bolt_array: recess_dia " +
              std::to_string(in.recess_dia_mm) +
              " mm >= per-bolt arc " + std::to_string(arcPerBolt) +
              " mm (adjacent counterbores overlap)");
    }
    if (in.recess_depth_mm >= in.t_bolt_depth_mm) {
        r.add("DFM-RECESS-DEPTH", "error",
              "blade_root_t_bolt_array: recess_depth must be < t_bolt_depth");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Build sequence (SEQUENTIAL — no compound cutter):
//   for i in 0..bolt_count-1:
//     theta_i = 2π·i / bolt_count
//     (bx, by) = (cx + R·cos θ, cy + R·sin θ)
//     wp = pr::cut(wp, through_cyl_i)     // T-bolt shank bore
//     wp = pr::cut(wp, counterbore_i)     // T-bolt head recess
//
// Sub-feature count = 2 × bolt_count.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "blade_root_t_bolt_array DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());

    constexpr double kOverhang = 0.05;
    const double tBoltR = in.t_bolt_dia_mm / 2.0;
    const double recR   = in.recess_dia_mm / 2.0;
    const double pcdR   = in.bolt_pcd_mm / 2.0;

    TopoDS_Shape current = wp.shape();

    // Sequential rotation cuts — one through-bore + one counterbore per slot,
    // applied in order, never combined into a compound cutter (PCD-arrayed
    // cylinders never overlap each other but concentric through+recess MAY
    // share the axis, so we keep them as separate sequential cuts).
    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        const double bx = in.center_x_mm + pcdR * std::cos(theta);
        const double by = in.center_y_mm + pcdR * std::sin(theta);

        // Entry point at the face plane, offset by kOverhang along +n.
        const gp_Pnt entry(bx + n.X() * kOverhang,
                           by + n.Y() * kOverhang,
                           faceC.Z() + n.Z() * kOverhang);

        const gp_Ax2 ax(entry, drillDir);
        const TopoDS_Shape throughTool = pr::cylinder(
            ax, tBoltR, in.t_bolt_depth_mm + 2.0 * kOverhang);
        current = pr::cut(current, throughTool);

        const TopoDS_Shape recessTool = pr::cylinder(
            ax, recR, in.recess_depth_mm + 2.0 * kOverhang);
        current = pr::cut(current, recessTool);
    }
    const TopoDS_Shape newShape = current;

    // Derived volume removed (analytic).
    const double vThrough = M_PI * tBoltR * tBoltR * in.t_bolt_depth_mm;
    const double vRecess  = M_PI * (recR * recR - tBoltR * tBoltR) *
                            in.recess_depth_mm;
    const double removedVol = static_cast<double>(in.bolt_count) *
                              (vThrough + vRecess);

    // Per-bolt JSON for pattern.
    json bolts_json = json::array();
    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        bolts_json.push_back({
            { "x_mm", in.center_x_mm + pcdR * std::cos(theta) },
            { "y_mm", in.center_y_mm + pcdR * std::sin(theta) },
            { "theta_rad", theta },
        });
    }

    json params = {
        { "face_id",         in.face_id },
        { "center_x_mm",     in.center_x_mm },
        { "center_y_mm",     in.center_y_mm },
        { "bolt_pcd_mm",     in.bolt_pcd_mm },
        { "bolt_count",      in.bolt_count },
        { "t_bolt_dia_mm",   in.t_bolt_dia_mm },
        { "t_bolt_depth_mm", in.t_bolt_depth_mm },
        { "recess_dia_mm",   in.recess_dia_mm },
        { "recess_depth_mm", in.recess_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "wind_feature_type",          "blade_root_t_bolt_array" },
        { "bolt_count",                 in.bolt_count },
        { "bolt_pcd_mm",                in.bolt_pcd_mm },
        { "t_bolt_dia_mm",              in.t_bolt_dia_mm },
        { "recess_dia_mm",              in.recess_dia_mm },
        { "subfeature_count",           2 * in.bolt_count },
        { "bolt_positions",             bolts_json },
        { "derived_volume_removed_mm3", removedVol },
        { "standard_ref",               "IEC 61400-2 / GL Wind" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;counterbore";
    tooling.tool_dia_mm       = in.recess_dia_mm;
    tooling.tool_length_mm    = in.t_bolt_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.08;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = 30.0 * static_cast<double>(in.bolt_count);
    tooling.extra = {
        { "wind_feature_type", "blade_root_t_bolt_array" },
        { "bolt_family",       "T-bolt (utility wind)" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::blade_root_t_bolt_array applied: N={} PCD={} dia={}",
                  in.bolt_count, in.bolt_pcd_mm, in.t_bolt_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay ─────────────────────────────────────────

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
            { "is_compound",       true },
            { "wind_feature_type", "blade_root_t_bolt_array" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::blade_root_t_bolt_array
