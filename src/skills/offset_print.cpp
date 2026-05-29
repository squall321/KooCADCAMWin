// @lat: [[engine/skills#offset_print]]

#include "offset_print.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::offset_print {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.paper_gsm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "offset_print paper_gsm must be > 0 (got " +
              std::to_string(in.paper_gsm) + ")");
    }
    if (in.sheets <= 0) {
        r.add("DFM-INPUT", "error",
              "offset_print sheets must be > 0 (got " +
              std::to_string(in.sheets) + ")");
    }
    if (in.ink_set < 1) {
        r.add("DFM-INPUT", "error",
              "offset_print ink_set must be ≥ 1 (got " +
              std::to_string(in.ink_set) + ")");
    }

    if (in.paper_gsm > 0.0 && in.paper_gsm < 60.0) {
        r.add("DFM-PAPER-LIGHT", "info",
              "offset_print paper_gsm " + std::to_string(in.paper_gsm) +
              " < 60 — show-through risk on duplex jobs");
    }
    if (in.paper_gsm > 400.0) {
        r.add("DFM-PAPER-HEAVY", "info",
              "offset_print paper_gsm " + std::to_string(in.paper_gsm) +
              " > 400 — board stock; verify press feed envelope");
    }
    if (in.ink_set > 6) {
        r.add("DFM-INK-SET", "info",
              "offset_print ink_set " + std::to_string(in.ink_set) +
              " > 6 — beyond hexachrome; specialty tower required");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "offset_print DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "paper_gsm", in.paper_gsm },
        { "ink_set",   in.ink_set },
        { "sheets",    in.sheets },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "paper_gsm",        in.paper_gsm },
        { "ink_set",          in.ink_set },
        { "sheets",           in.sheets },
        { "process_family",   "lithographic_offset" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "offset_press";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "rubber_blanket";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Mid-volume sheet-fed press: ~12000 sph → 0.3 s/sheet.
    tooling.est_cycle_time_s  = 0.3 * static_cast<double>(in.sheets);
    tooling.extra = {
        { "process",          "offset_lithography" },
        { "paper_gsm",        in.paper_gsm },
        { "ink_set",          in.ink_set },
        { "sheets",           in.sheets },
        { "process_category", "printing" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::offset_print applied: gsm={} inks={} sheets={}",
                  in.paper_gsm, in.ink_set, in.sheets);

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

}  // namespace koocadcam::skill::offset_print
