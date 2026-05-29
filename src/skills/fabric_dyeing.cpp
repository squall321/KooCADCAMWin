// @lat: [[engine/skills#fabric_dyeing]]

#include "fabric_dyeing.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::fabric_dyeing {

using nlohmann::json;

namespace {

bool isKnownDye(const std::string& s)
{
    return s == "reactive" || s == "disperse" || s == "acid" ||
           s == "vat" || s == "direct";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.fixation_pct < 0.0 || in.fixation_pct > 100.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": fixation_pct must be in [0, 100] (got " +
              std::to_string(in.fixation_pct) + ")");
    }
    if (in.temp_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": temp_c must be > 0 (got " +
              std::to_string(in.temp_c) + ")");
    }

    if (!isKnownDye(in.dye_type)) {
        r.add("DFM-TEX-DYE", "info",
              std::string(kSkillId) + ": dye_type '" + in.dye_type +
              "' unrecognised — expected reactive | disperse | acid | vat | direct");
    }

    if (in.fixation_pct >= 0.0 && in.fixation_pct < 70.0) {
        r.add("DFM-TEX-FIXATION", "info",
              std::string(kSkillId) + ": fixation_pct " +
              std::to_string(in.fixation_pct) +
              " < 70 — washfastness risk, expect bleed in laundry");
    }

    if (in.temp_c > 0.0 && (in.temp_c < 20.0 || in.temp_c > 140.0)) {
        r.add("DFM-TEX-DYE-TEMP", "info",
              std::string(kSkillId) + ": temp_c " + std::to_string(in.temp_c) +
              " outside typical dye-bath range [20, 140] °C");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "fabric_dyeing DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double effluent_dye_pct = (in.fixation_pct >= 0.0)
                                  ? (100.0 - in.fixation_pct)
                                  : 0.0;

    json params = {
        { "dye_type",     in.dye_type },
        { "fixation_pct", in.fixation_pct },
        { "temp_c",       in.temp_c },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "dye_type",          in.dye_type },
        { "fixation_pct",      in.fixation_pct },
        { "temp_c",            in.temp_c },
        { "effluent_dye_pct",  effluent_dye_pct },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "dye_bath";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_316_vessel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Heat ramp ~30 min + soak ~60 min + cool 30 min = 120 min nominal.
    tooling.est_cycle_time_s  = 120.0 * 60.0;
    tooling.extra = {
        { "process",          "fabric_dyeing" },
        { "dye_type",         in.dye_type },
        { "fixation_pct",     in.fixation_pct },
        { "temp_c",           in.temp_c },
        { "effluent_dye_pct", effluent_dye_pct },
        { "process_category", "textile" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::fabric_dyeing applied: dye={} fixation={}% T={}°C",
        in.dye_type, in.fixation_pct, in.temp_c);

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

}  // namespace koocadcam::skill::fabric_dyeing
