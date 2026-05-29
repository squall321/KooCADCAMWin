// @lat: [[engine/skills#vacuum_brazing]]

#include "vacuum_brazing.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::vacuum_brazing {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.filler_metal.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + " filler_metal must be non-empty");
    }

    if (in.vacuum_torr <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              " vacuum_torr must be > 0 (got " +
              std::to_string(in.vacuum_torr) + ")");
    } else if (in.vacuum_torr > 1.0e-3) {
        r.add("DFM-BRAZE-VAC", "warning",
              std::string(kSkillId) + " vacuum_torr " +
              std::to_string(in.vacuum_torr) +
              " > 1e-3 — oxide risk; use partial-pressure inert or improve pump");
    }

    if (in.peak_temp_c <= 600.0) {
        r.add("DFM-BRAZE-TEMP", "error",
              std::string(kSkillId) +
              " peak_temp_c " + std::to_string(in.peak_temp_c) +
              " <= 600 C — below brazing regime (this is soldering)");
    } else if (in.peak_temp_c > 1300.0) {
        r.add("DFM-BRAZE-TEMP", "error",
              std::string(kSkillId) +
              " peak_temp_c " + std::to_string(in.peak_temp_c) +
              " > 1300 C — risk of melting common parent metals");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "vacuum_brazing DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "filler_metal", in.filler_metal },
        { "vacuum_torr",  in.vacuum_torr },
        { "peak_temp_c",  in.peak_temp_c },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "filler_metal",     in.filler_metal },
        { "vacuum_torr",      in.vacuum_torr },
        { "peak_temp_c",      in.peak_temp_c },
        { "joint_quality",    "leak_tight" },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "vacuum_brazing_furnace";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "graphite_hot_zone";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Typical cycle: pump down (~30 min) + ramp + hold + cool ≈ 4 h.
    tooling.est_cycle_time_s  = 4.0 * 3600.0;
    tooling.extra = {
        { "process",      "vacuum_brazing" },
        { "filler_metal", in.filler_metal },
        { "vacuum_torr",  in.vacuum_torr },
        { "peak_temp_c",  in.peak_temp_c },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::vacuum_brazing applied: filler={} vacuum={}torr T={}C",
                  in.filler_metal, in.vacuum_torr, in.peak_temp_c);

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

}  // namespace koocadcam::skill::vacuum_brazing
