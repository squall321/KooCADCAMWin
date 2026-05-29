// @lat: [[engine/skills#vibration_qualify]]

#include "vibration_qualify.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>

namespace koocadcam::skill::vibration_qualify {

using nlohmann::json;

namespace {

bool isKnownResult(const std::string& r)
{
    return r == "pass" || r == "fail";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.freq_max_hz <= in.freq_min_hz) {
        r.add("DFM-INPUT", "error",
              "vibration_qualify freq_max_hz (" +
              std::to_string(in.freq_max_hz) +
              ") must be > freq_min_hz (" +
              std::to_string(in.freq_min_hz) + ")");
    }

    if (in.freq_min_hz < 5.0) {
        r.add("DFM-VIB-FREQ", "warning",
              "vibration_qualify freq_min_hz " +
              std::to_string(in.freq_min_hz) +
              " < 5 — below typical shaker low end");
    }
    if (in.freq_max_hz > 2000.0) {
        r.add("DFM-VIB-FREQ", "warning",
              "vibration_qualify freq_max_hz " +
              std::to_string(in.freq_max_hz) +
              " > 2000 — above typical electrodynamic shaker ceiling");
    }

    if (in.accel_g_rms <= 0.0) {
        r.add("DFM-VIB-ACCEL", "error",
              "vibration_qualify accel_g_rms must be > 0");
    } else if (in.accel_g_rms > 50.0) {
        r.add("DFM-VIB-ACCEL", "warning",
              "vibration_qualify accel_g_rms " +
              std::to_string(in.accel_g_rms) +
              " > 50 g rms — confirm fixture / specimen survives");
    }

    if (in.duration_min < 1.0) {
        r.add("DFM-INPUT", "error",
              "vibration_qualify duration_min must be ≥ 1.0");
    }

    if (in.axes_count != 1 && in.axes_count != 3) {
        r.add("DFM-VIB-AXES", "warning",
              "vibration_qualify axes_count " +
              std::to_string(in.axes_count) +
              " not in {1 single-axis, 3 X+Y+Z}");
    }

    if (!isKnownResult(in.result)) {
        r.add("DFM-VIB-RESULT", "warning",
              "vibration_qualify result '" + in.result + "' not in {pass, fail}");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "vibration_qualify DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string result =
        isKnownResult(in.result) ? in.result : "pass";
    const int axes = (in.axes_count == 1 || in.axes_count == 3)
                         ? in.axes_count : 3;

    json params = {
        { "freq_min_hz",   in.freq_min_hz },
        { "freq_max_hz",   in.freq_max_hz },
        { "accel_g_rms",   in.accel_g_rms },
        { "duration_min",  in.duration_min },
        { "axes_count",    axes },
        { "result",        result },
    };
    json pattern = {
        { "kind",            kSkillId },
        { "freq_min_hz",     in.freq_min_hz },
        { "freq_max_hz",     in.freq_max_hz },
        { "freq_range_hz",
          { in.freq_min_hz, in.freq_max_hz } },     // explicit [min, max] tuple
        { "accel_g_rms",     in.accel_g_rms },
        { "duration_min",    in.duration_min },
        { "axes_count",      axes },
        { "result",          result },
        { "test_kind",
          (axes == 3) ? "tri_axis_random" : "single_axis_sine" },
        { "geometry_changed", false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "electrodynamic_shaker";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;       // QA record only
    // Cycle = setup 10 min + per-axis duration + 5 min teardown.
    tooling.est_cycle_time_s  =
        10.0 * 60.0 + axes * in.duration_min * 60.0 + 5.0 * 60.0;
    tooling.extra = {
        { "process",       "vibration_qa" },
        { "freq_min_hz",   in.freq_min_hz },
        { "freq_max_hz",   in.freq_max_hz },
        { "accel_g_rms",   in.accel_g_rms },
        { "duration_min",  in.duration_min },
        { "axes_count",    axes },
        { "result",        result },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::vibration_qualify applied: f∈[{},{}] Hz {} g rms {} min axes={} result={}",
                  in.freq_min_hz, in.freq_max_hz, in.accel_g_rms,
                  in.duration_min, axes, result);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::vibration_qualify
