// @lat: [[engine/skills#pickling_passivate]]

#include "pickling_passivate.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::pickling_passivate {

using nlohmann::json;

namespace {

bool isKnownMix(const std::string& m)
{
    return m == "HNO3+HF" || m == "HNO3" || m == "citric";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.surface_area_m2 <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string("pickling_passivate: surface_area_m2 must be > 0 (got ") +
              std::to_string(in.surface_area_m2) + ")");
    }

    if (in.acid_mix.empty()) {
        r.add("DFM-INPUT", "warning",
              "pickling_passivate: acid_mix empty — defaulting to 'HNO3+HF'");
    } else if (!isKnownMix(in.acid_mix)) {
        r.add("DFM-PICK-MIX", "info",
              std::string("pickling_passivate: acid_mix '") + in.acid_mix +
              "' not in {HNO3+HF, HNO3, citric} — assumed equivalent");
    }

    if (in.passivation_time_h <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string("pickling_passivate: passivation_time_h must be > 0 (got ") +
              std::to_string(in.passivation_time_h) + ")");
    } else if (in.passivation_time_h < 0.5 || in.passivation_time_h > 4.0) {
        r.add("DFM-PICK-TIME", "info",
              std::string("pickling_passivate: passivation_time_h ") +
              std::to_string(in.passivation_time_h) +
              " outside typical ASTM A967 range [0.5, 4.0] h");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pickling_passivate DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string acid_mix = in.acid_mix.empty() ? "HNO3+HF" : in.acid_mix;

    json params = {
        { "surface_area_m2",    in.surface_area_m2 },
        { "acid_mix",           acid_mix },
        { "passivation_time_h", in.passivation_time_h },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "surface_area_m2",     in.surface_area_m2 },
        { "acid_mix",            acid_mix },
        { "passivation_time_h",  in.passivation_time_h },
        { "expected_surface",    "chromium_oxide_passive" },
        { "geometry_changed",    false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "pickling_bath";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "acid_resistant_lining";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;  // chemical, negligible at this granularity
    tooling.est_cycle_time_s  = in.passivation_time_h * 3600.0;
    tooling.extra = {
        { "process",            "pickle_then_passivate" },
        { "acid_mix",           acid_mix },
        { "surface_area_m2",    in.surface_area_m2 },
        { "passivation_time_h", in.passivation_time_h },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::pickling_passivate applied: area={}m2 mix={} t={}h",
                  in.surface_area_m2, acid_mix, in.passivation_time_h);

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

}  // namespace koocadcam::skill::pickling_passivate
