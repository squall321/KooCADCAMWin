// @lat: [[engine/skills#thermal_pad_recess_with_to220]]

#include "thermal_pad_recess_with_to220.hpp"

#include "_iso_thread_table.hpp"
#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::thermal_pad_recess_with_to220 {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Package footprint table ──────────────────────────────────────────────

namespace {

struct PkgEntry {
    const char* key;
    double      width_mm;
    double      depth_mm;
    double      screw_offset_mm;
    const char* recommended_screw;
};

constexpr std::array<PkgEntry, 4> kPackages {{
    // key      width depth offset rec_screw    (mfg-standard footprints, mm)
    { "TO-220", 10.0,  4.6,  2.85, "M3" },  // hole 2.85 mm from top edge
    { "TO-247", 16.0,  5.5,  3.6,  "M4" },  // hole 3.6 mm from top edge
    { "TO-263", 10.0,  8.5,  2.85, "M3" },  // SMD D2PAK family
    { "D2PAK",  10.3,  8.4,  2.85, "M3" },
}};

}  // namespace

PackageFootprint footprintFor(const std::string& key)
{
    for (const auto& p : kPackages) {
        if (key == p.key) {
            PackageFootprint f;
            f.width_mm          = p.width_mm;
            f.depth_mm          = p.depth_mm;
            f.screw_offset_mm   = p.screw_offset_mm;
            f.recommended_screw = p.recommended_screw;
            return f;
        }
    }
    return {};
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;

    const PackageFootprint pkg = footprintFor(in.device_package);
    if (pkg.width_mm <= 0.0) {
        r.add("DFM-PKG", "error",
              "thermal_pad_recess_with_to220: unknown device_package '" +
              in.device_package + "' (supported: TO-220/TO-247/TO-263/D2PAK)");
    }
    if (!(in.pad_thickness_mm > 0.05) || in.pad_thickness_mm > 5.0) {
        r.add("DFM-PAD-THK", "error",
              "thermal_pad_recess_with_to220: pad_thickness_mm " +
              std::to_string(in.pad_thickness_mm) +
              " out of range (0.05 mm .. 5.0 mm)");
    }
    const auto* spec = thread_table::findMetric(in.screw_thread_size_key);
    if (!spec) {
        r.add("DFM-THREAD-KEY", "error",
              "thermal_pad_recess_with_to220: screw_thread_size_key '" +
              in.screw_thread_size_key + "' not in central ISO M-thread table");
        return r;
    }
    if (pkg.width_mm > 0.0 &&
        std::string(pkg.recommended_screw) != in.screw_thread_size_key) {
        r.add("DFM-PKG-MATCH", "warning",
              std::string("thermal_pad_recess_with_to220: package ") +
              in.device_package + " typically uses " + pkg.recommended_screw +
              " (got " + in.screw_thread_size_key + ")");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "thermal_pad_recess_with_to220 DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const PackageFootprint pkg = footprintFor(in.device_package);
    const auto* spec = thread_table::findMetric(in.screw_thread_size_key);
    // validated above

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zTop = zMax;

    // Recess depth is the pad thickness + 0.1 mm clearance.
    constexpr double kClearance = 0.1;
    const double recessDepth = in.pad_thickness_mm + kClearance;
    constexpr double kEmbed = 0.05;

    // Add 0.2 mm wall clearance around the package footprint for thermal pad fit.
    constexpr double kFootprintMargin = 0.2;
    const double recessW = pkg.width_mm + 2.0 * kFootprintMargin;
    const double recessD = pkg.depth_mm + 2.0 * kFootprintMargin;

    // ── Sub-feature 1: rounded-rect (R=0.5) recess pocket
    const gp_Pnt recessBottomCenter(in.center_x_mm, in.center_y_mm,
                                    zTop - recessDepth);
    const TopoDS_Shape recess = pr::roundedRectPocketTool(
        recessBottomCenter, recessW, recessD, recessDepth + kEmbed, 0.5);

    // ── Sub-feature 2: screw hole (clearance medium fit from central table)
    // Hole is offset from the recess centre by (width/2 - screw_offset_mm)
    // along +X (the package's mounting-tab axis is conventionally aligned with X).
    const double screwX = in.center_x_mm + (pkg.width_mm / 2.0 - pkg.screw_offset_mm);
    const double screwY = in.center_y_mm;
    const double clearanceR = spec->clearance_medium_mm * 0.5;
    const gp_Pnt drillTop(screwX, screwY, zTop + kEmbed);
    const gp_Ax2 drillAx(drillTop, gp_Dir(0.0, 0.0, -1.0));
    // Drill all the way through.
    const double drillLen = (zMax - zMin) + 2.0 * kEmbed;
    const TopoDS_Shape drill = pr::cylinder(drillAx, clearanceR, drillLen);

    const TopoDS_Shape afterRecess = pr::cut(wp.shape(), recess);
    const TopoDS_Shape newShape    = pr::cut(afterRecess, drill);

    const double recessVol = recessW * recessD * recessDepth;
    const double drillVol  = M_PI * clearanceR * clearanceR * (zMax - zMin);
    const double removedVol = recessVol + drillVol;

    json params = {
        { "face_id",                in.face_id },
        { "center_x_mm",            in.center_x_mm },
        { "center_y_mm",            in.center_y_mm },
        { "device_package",         in.device_package },
        { "pad_thickness_mm",       in.pad_thickness_mm },
        { "screw_thread_size_key",  in.screw_thread_size_key },
    };
    json pattern_js = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "electronics_feature_type", "power_device_thermal_mount" },
        { "device_package",         in.device_package },
        { "subfeature_count",       2 },
        { "footprint_w_mm",         pkg.width_mm },
        { "footprint_d_mm",         pkg.depth_mm },
        { "recess_w_mm",            recessW },
        { "recess_d_mm",            recessD },
        { "recess_depth_mm",        recessDepth },
        { "screw_thread_size_key",  in.screw_thread_size_key },
        { "screw_clearance_dia_mm", spec->clearance_medium_mm },
        { "derived_volume_removed_mm3", removedVol },
        { "subfeatures", json::array({
            { { "op", "cut_rounded_recess" },
              { "w_mm", recessW }, { "d_mm", recessD },
              { "depth_mm", recessDepth },
              { "removed_vol_mm3", recessVol } },
            { { "op", "cut_clearance_screw_hole" },
              { "dia_mm", spec->clearance_medium_mm },
              { "removed_vol_mm3", drillVol } },
        }) },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill";
    tooling.tool_dia_mm       = spec->clearance_medium_mm;
    tooling.tool_length_mm    = (zMax - zMin) + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(8.0, removedVol / 80.0);
    tooling.extra = {
        { "feature_standard", "Power-device package thermal mount" },
        { "device_package",   in.device_package },
        { "screw_thread_size_key", in.screw_thread_size_key },
        { "removed_volume_mm3", removedVol },
    };

    FeatureSignature sig { kSkillId, params, pattern_js, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::thermal_pad_recess_with_to220 applied: {} pad {} mm screw {}",
        in.device_package, in.pad_thickness_mm, in.screw_thread_size_key);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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
            { "device_package",    f.pattern.value("device_package", std::string{}) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::thermal_pad_recess_with_to220
