// @lat: [[engine/skills#deck_assembly]]

#include "deck_assembly.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::deck_assembly {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.deck_level.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": deck_level must be non-empty");
    }

    if (in.area_m2 <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": area_m2 must be > 0 (got " +
              std::to_string(in.area_m2) + ")");
    }

    if (in.member_count <= 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": member_count must be > 0 (got " +
              std::to_string(in.member_count) + ")");
    }

    if (in.area_m2 > 5000.0) {
        r.add("DFM-DECK-AREA", "info",
              std::string(kSkillId) + ": area_m2 " +
              std::to_string(in.area_m2) +
              " > 5000 — super-deck, segment & track each band separately");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "deck_assembly DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double membersPerM2 =
        in.area_m2 > 0.0 ? static_cast<double>(in.member_count) / in.area_m2 : 0.0;

    json params = {
        { "deck_level",   in.deck_level },
        { "area_m2",      in.area_m2 },
        { "member_count", in.member_count },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "deck_level",        in.deck_level },
        { "area_m2",           in.area_m2 },
        { "member_count",      in.member_count },
        { "members_per_m2",    membersPerM2 },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "deck_assembly_jig";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Allow ~ 30 min per member for tack-fit of girders / beams / stiffeners.
    tooling.est_cycle_time_s  = static_cast<double>(in.member_count) * 1800.0;
    tooling.extra = {
        { "process",          "deck_member_tackfit" },
        { "deck_level",       in.deck_level },
        { "area_m2",          in.area_m2 },
        { "member_count",     in.member_count },
        { "code_reference",   "IACS_PR2_deck_assembly" },
        { "process_category", "shipbuilding_deck" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::deck_assembly applied: deck={} area={}m2 members={}",
        in.deck_level, in.area_m2, in.member_count);

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

}  // namespace koocadcam::skill::deck_assembly
