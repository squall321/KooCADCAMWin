// @lat: [[engine/skills#digital_print]]

#include "digital_print.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::digital_print {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownInkTypes()
{
    static const std::unordered_set<std::string> s = {
        "toner", "aqueous", "uv", "latex",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.dpi <= 0) {
        r.add("DFM-INPUT", "error",
              "digital_print dpi must be > 0 (got " +
              std::to_string(in.dpi) + ")");
    }
    if (in.sheets <= 0) {
        r.add("DFM-INPUT", "error",
              "digital_print sheets must be > 0 (got " +
              std::to_string(in.sheets) + ")");
    }
    if (in.ink_type.empty() || knownInkTypes().count(in.ink_type) == 0) {
        r.add("DFM-INPUT", "error",
              "digital_print unknown ink_type '" + in.ink_type +
              "' (expected toner | aqueous | uv | latex)");
    }

    if (in.dpi > 0 && in.dpi < 300) {
        r.add("DFM-DPI-LOW", "info",
              "digital_print dpi " + std::to_string(in.dpi) +
              " < 300 — draft quality");
    }
    if (in.dpi > 2400) {
        r.add("DFM-DPI-HIGH", "info",
              "digital_print dpi " + std::to_string(in.dpi) +
              " > 2400 — specialty resolution; expect longer cycle");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "digital_print DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "dpi",      in.dpi },
        { "ink_type", in.ink_type },
        { "sheets",   in.sheets },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "dpi",              in.dpi },
        { "ink_type",         in.ink_type },
        { "sheets",           in.sheets },
        { "process_family",   "digital_printing" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    if      (in.ink_type == "toner")   tooling.tool_type = "xerographic_press";
    else if (in.ink_type == "uv")      tooling.tool_type = "uv_inkjet_press";
    else if (in.ink_type == "latex")   tooling.tool_type = "latex_inkjet_press";
    else                                tooling.tool_type = "aqueous_inkjet_press";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "printhead_assembly";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Digital sheet-fed ~ 1.0 s/sheet at typical SRA3 speed.
    tooling.est_cycle_time_s  = 1.0 * static_cast<double>(in.sheets);
    tooling.extra = {
        { "process",          "digital_printing" },
        { "dpi",              in.dpi },
        { "ink_type",         in.ink_type },
        { "sheets",           in.sheets },
        { "process_category", "printing" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::digital_print applied: dpi={} ink={} sheets={}",
                  in.dpi, in.ink_type, in.sheets);

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

}  // namespace koocadcam::skill::digital_print
