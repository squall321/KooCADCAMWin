// @lat: [[engine/skills#weaving]]

#include "weaving.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::weaving {

using nlohmann::json;

namespace {

bool isKnownLoom(const std::string& s)
{
    return s == "rapier" || s == "air_jet" || s == "water_jet" ||
           s == "shuttle" || s == "projectile";
}

bool isKnownWeave(const std::string& s)
{
    return s == "plain" || s == "twill" || s == "satin" ||
           s == "basket" || s == "leno";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.warp_count <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": warp_count must be > 0 (got " +
              std::to_string(in.warp_count) + ")");
    }
    if (in.weft_count <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": weft_count must be > 0 (got " +
              std::to_string(in.weft_count) + ")");
    }

    if (!isKnownLoom(in.loom_type)) {
        r.add("DFM-TEX-LOOM", "info",
              std::string(kSkillId) + ": loom_type '" + in.loom_type +
              "' unrecognised — expected rapier | air_jet | water_jet | shuttle | projectile");
    }

    if (!isKnownWeave(in.weave_pattern)) {
        r.add("DFM-TEX-WEAVE", "info",
              std::string(kSkillId) + ": weave_pattern '" + in.weave_pattern +
              "' unrecognised — expected plain | twill | satin | basket | leno");
    }

    if (in.warp_count > 0.0 && in.weft_count > 0.0) {
        const double ratio = in.weft_count / in.warp_count;
        if (ratio < 0.3 || ratio > 3.0) {
            r.add("DFM-TEX-BALANCE", "info",
                  std::string(kSkillId) +
                  ": weft_count/warp_count ratio " + std::to_string(ratio) +
                  " outside balanced fabric range [0.3, 3.0] — verify shed geometry");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "weaving DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double fabric_density = in.warp_count * in.weft_count;  // crossings / cm²

    json params = {
        { "loom_type",     in.loom_type },
        { "warp_count",    in.warp_count },
        { "weft_count",    in.weft_count },
        { "weave_pattern", in.weave_pattern },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "loom_type",         in.loom_type },
        { "warp_count",        in.warp_count },
        { "weft_count",        in.weft_count },
        { "weave_pattern",     in.weave_pattern },
        { "fabric_density",    fabric_density },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "loom";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "steel_reed";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Typical industrial weaving rate: ~600 picks per minute → seconds per cm of fabric.
    tooling.est_cycle_time_s  = in.weft_count > 0.0 ? in.weft_count / 10.0 : 0.0;
    tooling.extra = {
        { "process",          "weaving" },
        { "loom_type",        in.loom_type },
        { "weave_pattern",    in.weave_pattern },
        { "fabric_density",   fabric_density },
        { "process_category", "textile" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::weaving applied: loom={} pattern={} warp={}/cm weft={}/cm",
        in.loom_type, in.weave_pattern, in.warp_count, in.weft_count);

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

}  // namespace koocadcam::skill::weaving
