// @lat: [[engine/skills#string_tune]]

#include "string_tune.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::string_tune {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.string_count < 1) {
        r.add("DFM-INPUT", "error",
              "string_tune string_count must be >= 1 (got " +
              std::to_string(in.string_count) + ")");
    }

    if (static_cast<int>(in.target_freq_hz_per_string.size()) != in.string_count) {
        r.add("DFM-TUNE-COUNT", "error",
              "string_tune target_freq_hz_per_string size " +
              std::to_string(in.target_freq_hz_per_string.size()) +
              " != string_count " + std::to_string(in.string_count));
    }

    for (size_t i = 0; i < in.target_freq_hz_per_string.size(); ++i) {
        const double f = in.target_freq_hz_per_string[i];
        if (f < 16.0 || f > 8000.0) {
            r.add("DFM-TUNE-FREQ", "error",
                  "string_tune string[" + std::to_string(i) +
                  "] target_freq " + std::to_string(f) +
                  " Hz outside musical range [16, 8000]");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "string_tune DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Compute mean target frequency for diagnostics.
    double sumHz = 0.0;
    for (double f : in.target_freq_hz_per_string) sumHz += f;
    const double meanHz = in.target_freq_hz_per_string.empty()
                          ? 0.0
                          : sumHz / static_cast<double>(in.target_freq_hz_per_string.size());

    json freqs = in.target_freq_hz_per_string;
    json params = {
        { "string_count",              in.string_count },
        { "target_freq_hz_per_string", freqs },
    };
    json pattern = {
        { "kind",                        kSkillId },
        { "string_count",                in.string_count },
        { "target_freq_hz_per_string",   freqs },
        { "mean_freq_hz",                meanHz },
        { "geometry_changed",            false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "electronic_tuner";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "none";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~10 s per string for tuning + verification.
    tooling.est_cycle_time_s  = 10.0 * static_cast<double>(in.string_count);
    tooling.extra = {
        { "process",      "string_tuning" },
        { "string_count", in.string_count },
        { "mean_freq_hz", meanHz },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::string_tune applied: strings={} mean_hz={}",
                  in.string_count, meanHz);

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

}  // namespace koocadcam::skill::string_tune
