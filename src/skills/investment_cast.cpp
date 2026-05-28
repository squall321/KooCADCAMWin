// @lat: [[engine/skills#investment_cast]]

#include "investment_cast.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <unordered_set>

namespace koocadcam::skill::investment_cast {

using nlohmann::json;

namespace {

// Investment casting is commonly used for stainless, tool steels, super-alloys,
// bronze, jewelry alloys, etc.  Catalog is used for INFO-level reporting only.
const std::unordered_set<std::string>& knownInvestmentCastMaterials()
{
    static const std::unordered_set<std::string> s = {
        "stainless_steel", "carbon_steel", "tool_steel",
        "cobalt_chrome",   "nickel_alloy", "inconel",
        "titanium",        "aluminum_a356",
        "bronze",          "brass",
        "gold",            "silver",
    };
    return s;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.wall_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "investment_cast wall_thickness_mm must be > 0 (got " +
              std::to_string(in.wall_thickness_mm) + ")");
    } else if (in.wall_thickness_mm < 1.5) {
        r.add("DFM-CAST-WALL", "error",
              "investment_cast wall_thickness " +
              std::to_string(in.wall_thickness_mm) +
              " mm < 1.5 mm investment-cast minimum");
    }

    if (in.wax_pattern_shrink_pct < 0.5 ||
        in.wax_pattern_shrink_pct > 3.0) {
        r.add("DFM-CAST-SHRINK", "error",
              "investment_cast wax_pattern_shrink_pct " +
              std::to_string(in.wax_pattern_shrink_pct) +
              " outside typical [0.5, 3.0] %% range");
    }

    if (in.material.empty()) {
        r.add("DFM-INPUT", "error",
              "investment_cast material must be specified");
    } else if (knownInvestmentCastMaterials().count(in.material) == 0) {
        r.add("DFM-CAST-MAT", "info",
              "investment_cast material '" + in.material +
              "' not in standard investment-cast catalog — verify shrink");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// NO-OP geometric: investment_cast is a process annotation.  Build a fresh
// Workpiece around the SAME shape; record metadata.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "investment_cast DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "material",                in.material },
        { "wall_thickness_mm",       in.wall_thickness_mm },
        { "wax_pattern_shrink_pct",  in.wax_pattern_shrink_pct },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_process_only",            true },
        { "geometric_change",           false },
        { "process_family",             "casting" },
        { "process_subtype",            "investment_cast" },
        { "process_alias",              "lost_wax" },
        { "material",                   in.material },
        { "wall_thickness_mm",          in.wall_thickness_mm },
        { "wax_pattern_shrink_pct",     in.wax_pattern_shrink_pct },
        { "additive_workflow",          true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "investment_cast_shell";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "ceramic_shell";  // multi-coat ceramic slurry
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Shell build + dewax + pour + cool — multi-hour cycle.
    tooling.est_cycle_time_s  = 7200.0;
    tooling.extra = {
        { "process",                  "investment_cast" },
        { "process_family",           "casting" },
        { "material",                 in.material },
        { "wax_pattern_shrink_pct",   in.wax_pattern_shrink_pct },
        { "wall_thickness_mm",        in.wall_thickness_mm },
        { "dfm_findings",             findings },
        { "machining_constraint",
          "no material removal — investment casting forms net-shape part" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::investment_cast applied: material={} wall={} shrink={}%%",
                  in.material, in.wall_thickness_mm, in.wax_pattern_shrink_pct);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only.  Final-part geometry alone cannot reveal whether it was
// made by investment cast vs sand cast vs machining.

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

}  // namespace koocadcam::skill::investment_cast
