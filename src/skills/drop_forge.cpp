// @lat: [[engine/skills#drop_forge]]

#include "drop_forge.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::drop_forge {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.hammer_energy_kj <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": hammer_energy_kj must be > 0 (got " +
              std::to_string(in.hammer_energy_kj) + ")");
    }
    if (in.blow_count < 1) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": blow_count must be >= 1 (got " +
              std::to_string(in.blow_count) + ")");
    }

    if (in.hammer_energy_kj > 200.0) {
        r.add("DFM-DF-ENERGY", "warning",
              std::string(kSkillId) + ": hammer_energy_kj " +
              std::to_string(in.hammer_energy_kj) +
              " > 200 kJ — very heavy hammer; verify foundation rating");
    }
    if (in.blow_count > 20) {
        r.add("DFM-DF-BLOWS", "info",
              std::string(kSkillId) + ": blow_count " +
              std::to_string(in.blow_count) +
              " > 20 — accelerated die wear; consider press forging");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double totalKj = in.hammer_energy_kj * static_cast<double>(in.blow_count);

    json params = {
        { "hammer_energy_kj", in.hammer_energy_kj },
        { "blow_count",       in.blow_count },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "hammer_energy_kj", in.hammer_energy_kj },
        { "blow_count",       in.blow_count },
        { "total_energy_kj",  totalKj },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "drop_hammer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "H13_tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Each blow ~ 1 s plus a 5 s reposition.
    tooling.est_cycle_time_s  = 5.0 + 1.0 * static_cast<double>(in.blow_count);
    tooling.extra = json{
        { "process",          "drop_forge" },
        { "hammer_energy_kj", in.hammer_energy_kj },
        { "blow_count",       in.blow_count },
        { "total_energy_kj",  totalKj },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::drop_forge applied: {} kJ × {} blows = {} kJ total",
                  in.hammer_energy_kj, in.blow_count, totalKj);

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

}  // namespace koocadcam::skill::drop_forge
