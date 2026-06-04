// @lat: [[engine/skills#screw_terminal_strip_array]]

#include "screw_terminal_strip_array.hpp"

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
#include <cmath>
#include <memory>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::screw_terminal_strip_array {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.position_count < 2) {
        r.add("DFM-INPUT", "error",
              "screw_terminal_strip_array: position_count must be >= 2");
    }
    if (!(in.position_pitch_mm >= 4.0)) {
        r.add("DFM-STRIP-PITCH", "error",
              "screw_terminal_strip_array: pitch " +
              std::to_string(in.position_pitch_mm) +
              " < 4.0 mm (smallest standard Phoenix = 5.08 mm)");
    }
    if (!(in.strip_length_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "screw_terminal_strip_array: strip_length_mm must be > 0");
    }
    const auto* spec = thread_table::findMetric(in.screw_thread_size);
    if (!spec) {
        r.add("DFM-THREAD-KEY", "error",
              "screw_terminal_strip_array: screw_thread_size '" +
              in.screw_thread_size + "' not in central ISO M-thread table");
        return r;
    }
    if (in.position_count >= 2 && in.position_pitch_mm > 0.0 &&
        in.strip_length_mm > 0.0) {
        const double span = (in.position_count - 1) * in.position_pitch_mm +
                            spec->clearance_medium_mm;
        if (span > in.strip_length_mm + 1e-3) {
            r.add("DFM-STRIP-FIT", "error",
                  "screw_terminal_strip_array: hole span " +
                  std::to_string(span) + " > strip_length " +
                  std::to_string(in.strip_length_mm));
        }
    }
    // wp bbox sanity
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    if (in.position_count >= 2 && in.position_pitch_mm > 0.0) {
        const double endX =
            in.strip_origin_x_mm + (in.position_count - 1) * in.position_pitch_mm;
        if (endX > xMax + 1e-3 || in.strip_origin_x_mm < xMin - 1e-3) {
            r.add("DFM-STRIP-FIT", "error",
                  "screw_terminal_strip_array: hole array overruns workpiece bbox in X");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "screw_terminal_strip_array DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = thread_table::findMetric(in.screw_thread_size);

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zTop = zMax;
    constexpr double kEmbed = 0.05;
    const double drillLen = (zMax - zMin) + 2.0 * kEmbed;
    // Threaded mounting holes use the tap pilot dia (we drill the pilot;
    // tap is a downstream metadata-only op).
    const double pilotR = spec->tap_pilot_dia_mm * 0.5;

    std::vector<TopoDS_Shape> drills;
    drills.reserve(static_cast<std::size_t>(in.position_count));
    for (int i = 0; i < in.position_count; ++i) {
        const gp_Pnt top(
            in.strip_origin_x_mm + i * in.position_pitch_mm,
            in.strip_origin_y_mm,
            zTop + kEmbed);
        const gp_Ax2 ax(top, gp_Dir(0.0, 0.0, -1.0));
        drills.push_back(pr::cylinder(ax, pilotR, drillLen));
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), drills);

    const double perHoleVol = M_PI * pilotR * pilotR * (zMax - zMin);
    const double removedVol = perHoleVol * in.position_count;

    json params = {
        { "face_id",            in.face_id },
        { "strip_origin_x_mm",  in.strip_origin_x_mm },
        { "strip_origin_y_mm",  in.strip_origin_y_mm },
        { "position_count",     in.position_count },
        { "position_pitch_mm",  in.position_pitch_mm },
        { "screw_thread_size",  in.screw_thread_size },
        { "strip_length_mm",    in.strip_length_mm },
    };
    json pattern_js = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "electronics_feature_type", "screw_terminal_strip" },
        { "subfeature_count",       in.position_count },
        { "position_count",         in.position_count },
        { "position_pitch_mm",      in.position_pitch_mm },
        { "screw_thread_size",      in.screw_thread_size },
        { "pilot_dia_mm",           spec->tap_pilot_dia_mm },
        { "strip_length_mm",        in.strip_length_mm },
        { "derived_volume_removed_mm3", removedVol },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;tap";
    tooling.tool_dia_mm       = spec->tap_pilot_dia_mm;
    tooling.tool_length_mm    = drillLen + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(2.0, in.position_count * 1.5);
    tooling.extra = {
        { "feature_standard",   "Phoenix Contact-style screw terminal strip" },
        { "screw_thread_size",  in.screw_thread_size },
        { "position_count",     in.position_count },
        { "removed_volume_mm3", removedVol },
    };

    FeatureSignature sig { kSkillId, params, pattern_js, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::screw_terminal_strip_array applied: {} holes pitch {} mm thread {}",
        in.position_count, in.position_pitch_mm, in.screw_thread_size);

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
        r.confidence       = 0.95;
        r.matched_geometry = {
            { "source",      "metadata_replay" },
            { "is_compound", true },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::screw_terminal_strip_array
