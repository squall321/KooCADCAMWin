// @lat: [[engine/skills#deck_cast]]

#include "deck_cast.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::deck_cast {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.span_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": span_m must be > 0 (got " +
              std::to_string(in.span_m) + ")");
    }

    if (in.area_m2 <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": area_m2 must be > 0 (got " +
              std::to_string(in.area_m2) + ")");
    }

    if (in.rebar_kg_m3 <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": rebar_kg_m3 must be > 0 (got " +
              std::to_string(in.rebar_kg_m3) + ")");
    }

    if (in.span_m > 100.0) {
        r.add("DFM-DECK-SPAN", "info",
              std::string(kSkillId) + ": span_m " +
              std::to_string(in.span_m) +
              " > 100 — long-span deck; verify prestress / cable-stay design");
    }

    if (in.rebar_kg_m3 > 0.0 && in.rebar_kg_m3 < 60.0) {
        r.add("DFM-DECK-REBAR", "info",
              std::string(kSkillId) + ": rebar_kg_m3 " +
              std::to_string(in.rebar_kg_m3) +
              " < 60 — unusually light reinforcement for a bridge deck");
    }

    if (in.rebar_kg_m3 > 350.0) {
        r.add("DFM-DECK-REBAR", "info",
              std::string(kSkillId) + ": rebar_kg_m3 " +
              std::to_string(in.rebar_kg_m3) +
              " > 350 — congestion risk; check rebar spacing and concrete placement");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "deck_cast DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "span_m",      in.span_m },
        { "area_m2",     in.area_m2 },
        { "rebar_kg_m3", in.rebar_kg_m3 },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "span_m",            in.span_m },
        { "area_m2",           in.area_m2 },
        { "rebar_kg_m3",       in.rebar_kg_m3 },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "deck_casting_screed";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Deck casting ~ 40 m2/h with screed; add 60 min surface finishing.
    tooling.est_cycle_time_s  = (in.area_m2 / 40.0) * 3600.0 + 3600.0;
    tooling.extra = {
        { "process",          "bridge_deck_cast" },
        { "span_m",           in.span_m },
        { "area_m2",          in.area_m2 },
        { "rebar_kg_m3",      in.rebar_kg_m3 },
        { "code_reference",   "AASHTO_LRFD" },
        { "process_category", "civil_bridge_concrete" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::deck_cast applied: span={}m area={}m2 rebar={}kg/m3",
        in.span_m, in.area_m2, in.rebar_kg_m3);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != std::string(kSkillId)) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::deck_cast
