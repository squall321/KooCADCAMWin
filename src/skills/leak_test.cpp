// @lat: [[engine/skills#leak_test]]

#include "leak_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>

namespace koocadcam::skill::leak_test {

using nlohmann::json;

namespace {

bool isKnownType(const std::string& t)
{
    return t == "pressure" || t == "vacuum" || t == "helium";
}

bool isKnownResult(const std::string& r)
{
    return r == "pass" || r == "fail";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!isKnownType(in.test_type)) {
        r.add("DFM-LEAK-TYPE", "warning",
              "leak_test test_type '" + in.test_type +
              "' not in {pressure, vacuum, helium}");
    }

    if (in.test_pressure_pa <= 0.0) {
        r.add("DFM-INPUT", "error",
              "leak_test test_pressure_pa must be > 0");
    }

    if (in.max_leak_rate_pa_m3_s <= 0.0) {
        r.add("DFM-INPUT", "error",
              "leak_test max_leak_rate_pa_m3_s must be > 0");
    }

    if (in.duration_s < 30.0) {
        r.add("DFM-LEAK-DUR", "info",
              "leak_test duration_s " + std::to_string(in.duration_s) +
              " < 30 — short hold may miss slow leaks");
    }

    if (in.test_type == "helium" &&
        in.max_leak_rate_pa_m3_s > 0.0 &&
        in.max_leak_rate_pa_m3_s > 1e-5) {
        r.add("DFM-LEAK-HE", "info",
              "leak_test helium max_leak_rate_pa_m3_s " +
              std::to_string(in.max_leak_rate_pa_m3_s) +
              " > 1e-5 — relax test if specifying He (typ. floor ≈ 1e-9)");
    }

    if (!isKnownResult(in.result)) {
        r.add("DFM-LEAK-RESULT", "warning",
              "leak_test result '" + in.result + "' not in {pass, fail}");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "leak_test DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string testType =
        isKnownType(in.test_type) ? in.test_type : "pressure";
    const std::string result =
        isKnownResult(in.result) ? in.result : "pass";

    json params = {
        { "test_type",             testType },
        { "test_pressure_pa",      in.test_pressure_pa },
        { "max_leak_rate_pa_m3_s", in.max_leak_rate_pa_m3_s },
        { "duration_s",            in.duration_s },
        { "result",                result },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "test_type",             testType },
        { "test_pressure_pa",      in.test_pressure_pa },
        { "max_leak_rate_pa_m3_s", in.max_leak_rate_pa_m3_s },
        { "duration_s",            in.duration_s },
        { "result",                result },
        { "qa_category",
          (testType == "helium") ? "mass_spec_helium" : "pressure_decay" },
        { "geometry_changed",      false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = (testType == "helium") ? "helium_mass_spec"
                                                       : "leak_decay_rig";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = (testType == "helium") ? "stainless_chamber"
                                                       : "test_manifold";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;        // QA record only
    // Cycle = setup 5 min + hold + vent 1 min.
    tooling.est_cycle_time_s  = 5.0 * 60.0 + std::max(0.0, in.duration_s) + 60.0;
    tooling.extra = {
        { "process",                "leak_qa" },
        { "test_type",              testType },
        { "test_pressure_pa",       in.test_pressure_pa },
        { "max_leak_rate_pa_m3_s",  in.max_leak_rate_pa_m3_s },
        { "result",                 result },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::leak_test applied: type={} P={} Pa rate={} Pa·m3/s result={}",
                  testType, in.test_pressure_pa, in.max_leak_rate_pa_m3_s, result);

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

}  // namespace koocadcam::skill::leak_test
