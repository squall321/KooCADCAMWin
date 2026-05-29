// @lat: [[engine/skills#wood_finishing]]

#include "wood_finishing.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::wood_finishing {

using nlohmann::json;

namespace {

bool isValidFinishType(const std::string& s)
{
    return s == "varnish" || s == "stain" || s == "oil" ||
           s == "lacquer" || s == "wax";
}

bool isValidSheen(const std::string& s)
{
    return s == "matte" || s == "satin" ||
           s == "semi_gloss" || s == "gloss";
}

// Lookup: water resistance per finish type.
std::string waterResistance(const std::string& finish)
{
    if (finish == "varnish" || finish == "lacquer") return "high";
    if (finish == "oil")                            return "medium";
    if (finish == "wax")                            return "low";
    if (finish == "stain")                          return "low";
    return "low";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!isValidFinishType(in.finish_type)) {
        r.add("DFM-FINISH-TYPE", "error",
              std::string(kSkillId) + ": finish_type '" + in.finish_type +
              "' unknown — expected 'varnish' | 'stain' | 'oil' | 'lacquer' | 'wax'");
    }
    if (in.coats < 1) {
        r.add("DFM-FINISH-COATS-MIN", "error",
              std::string(kSkillId) + ": coats " + std::to_string(in.coats) +
              " < 1 — at least one coat required");
    } else if (in.coats > 6) {
        r.add("DFM-FINISH-COATS-MAX", "warning",
              std::string(kSkillId) + ": coats " + std::to_string(in.coats) +
              " > 6 — diminishing return + buildup risk");
    }
    if (in.cure_hours_per_coat <= 0.0) {
        r.add("DFM-FINISH-CURE", "error",
              std::string(kSkillId) + ": cure_hours_per_coat must be > 0 (got " +
              std::to_string(in.cure_hours_per_coat) + ")");
    }
    if (!isValidSheen(in.surface_sheen)) {
        r.add("DFM-FINISH-SHEEN", "info",
              std::string(kSkillId) + ": surface_sheen '" + in.surface_sheen +
              "' unrecognised — expected 'matte' | 'satin' | 'semi_gloss' | 'gloss'");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "wood_finishing DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double totalCureHours = in.cure_hours_per_coat * static_cast<double>(in.coats);
    const std::string water = waterResistance(in.finish_type);

    json params = {
        { "finish_type",         in.finish_type },
        { "coats",               in.coats },
        { "cure_hours_per_coat", in.cure_hours_per_coat },
        { "surface_sheen",       in.surface_sheen },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "finish_type",         in.finish_type },
        { "coats",               in.coats },
        { "cure_hours",          totalCureHours },
        { "surface_sheen",       in.surface_sheen },
        { "water_resistance",    water },
        { "geometry_changed",    false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "finishing_booth";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "brush_or_spray";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Apply (~5 min/coat) + cure × coats.
    tooling.est_cycle_time_s  = (300.0 + in.cure_hours_per_coat * 3600.0) * in.coats;
    tooling.extra["finish_type"]       = in.finish_type;
    tooling.extra["water_resistance"]  = water;
    tooling.extra["process_category"]  = "wood_finishing";
    tooling.extra["geometric_no_op"]   = true;
    tooling.extra["dfm_findings"]      = findings;
    tooling.extra["machining_constraint"] =
        "Wood finishing — surface treatment only; geometry unchanged; "
        "between-coat sanding (220-320 grit) typical for film finishes.";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::wood_finishing applied: type={} coats={} cure={}h sheen={}",
                  in.finish_type, in.coats, totalCureHours, in.surface_sheen);

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

}  // namespace koocadcam::skill::wood_finishing
