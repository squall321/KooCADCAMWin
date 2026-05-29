// @lat: [[engine/skills#blister_pack]]

#include "blister_pack.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::blister_pack {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownLiddings()
{
    static const std::unordered_set<std::string> s = {
        "aluminum_foil", "paper_foil", "polymer_film",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cavity_count <= 0) {
        r.add("DFM-INPUT", "error",
              "blister_pack cavity_count must be > 0 (got " +
              std::to_string(in.cavity_count) + ")");
    }

    if (in.sealing_temp_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              "blister_pack sealing_temp_c must be > 0 (got " +
              std::to_string(in.sealing_temp_c) + ")");
    }

    if (in.lidding_material.empty()) {
        r.add("DFM-BLISTER-LID", "error",
              "blister_pack lidding_material must be specified");
    } else if (knownLiddings().count(in.lidding_material) == 0) {
        r.add("DFM-BLISTER-LID", "info",
              "blister_pack lidding_material '" + in.lidding_material +
              "' not in known catalog (aluminum_foil | paper_foil | polymer_film)");
    }

    if (in.cavity_count > 60) {
        r.add("DFM-BLISTER-COUNT", "info",
              "blister_pack cavity_count " + std::to_string(in.cavity_count) +
              " > 60 — unusually high-count card");
    }

    if (in.sealing_temp_c > 0.0 &&
        (in.sealing_temp_c < 120.0 || in.sealing_temp_c > 220.0))
    {
        r.add("DFM-BLISTER-TEMP", "info",
              "blister_pack sealing_temp_c " + std::to_string(in.sealing_temp_c) +
              " °C outside typical [120, 220] range");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "blister_pack DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "cavity_count",     in.cavity_count },
        { "lidding_material", in.lidding_material },
        { "sealing_temp_c",   in.sealing_temp_c },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "cavity_count",     in.cavity_count },
        { "lidding_material", in.lidding_material },
        { "sealing_temp_c",   in.sealing_temp_c },
        { "process_family",   "blister_packaging" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "blister_thermoformer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "aluminum_mold";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Rough envelope: 4 s per cavity for thermoform + lid sealing.
    tooling.est_cycle_time_s  = 2.0 + 0.25 * in.cavity_count;
    tooling.extra = {
        { "process",          "blister_pack" },
        { "cavity_count",     in.cavity_count },
        { "lidding_material", in.lidding_material },
        { "sealing_temp_c",   in.sealing_temp_c },
        { "process_category", "packaging" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::blister_pack applied: cavities={} lid={} T={}°C",
                  in.cavity_count, in.lidding_material, in.sealing_temp_c);

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
        rf.matched_geometry = { { "source", "feature_history" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::blister_pack
