// @lat: [[engine/skills#panel_attach]]

#include "panel_attach.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::panel_attach {

using nlohmann::json;

namespace {

bool isKnownPanel(const std::string& t)
{
    return t == "metal_siding" ||
           t == "gypsum_board" ||
           t == "sandwich"     ||
           t == "glass";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.panel_type.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": panel_type must not be empty");
    } else if (!isKnownPanel(in.panel_type)) {
        r.add("DFM-PA-TYPE", "info",
              std::string(kSkillId) + ": panel_type '" + in.panel_type +
              "' not in {metal_siding, gypsum_board, sandwich, glass}");
    }

    if (in.fastener_count < 4) {
        r.add("DFM-PA-COUNT", "error",
              std::string(kSkillId) + ": fastener_count " +
              std::to_string(in.fastener_count) +
              " < 4 — minimum one fastener per panel corner");
    } else if (in.fastener_count > 200) {
        r.add("DFM-PA-COUNT", "error",
              std::string(kSkillId) + ": fastener_count " +
              std::to_string(in.fastener_count) +
              " > 200 — exceeds typical single-panel fastener layout");
    }

    if (in.sealant_type.empty()) {
        r.add("DFM-INPUT", "warning",
              std::string(kSkillId) + ": sealant_type empty — "
              "joints will leak without weather seal");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "panel_attach DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string sealant = in.sealant_type.empty() ? "none"
                                                        : in.sealant_type;

    json params = {
        { "panel_type",     in.panel_type },
        { "fastener_count", in.fastener_count },
        { "sealant_type",   sealant },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "panel_type",       in.panel_type },
        { "fastener_count",   in.fastener_count },
        { "sealant_type",     sealant },
        { "weather_sealed",   !in.sealant_type.empty() },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = (in.panel_type == "glass") ? "suction_lifter"
                                                           : "impact_driver";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 4 s per fastener + 30 s sealant bead pass.
    tooling.est_cycle_time_s  = 4.0 * static_cast<double>(in.fastener_count) +
                                (in.sealant_type.empty() ? 0.0 : 30.0);
    tooling.extra = {
        { "process",        "panel_attach" },
        { "panel_type",     in.panel_type },
        { "fastener_count", in.fastener_count },
        { "sealant_type",   sealant },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::panel_attach applied: panel={} fasteners={} sealant={}",
                  in.panel_type, in.fastener_count, sealant);

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

}  // namespace koocadcam::skill::panel_attach
