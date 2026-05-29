// @lat: [[engine/skills#wear_inspect]]

#include "wear_inspect.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::wear_inspect {

using nlohmann::json;

namespace {

bool isKnownPattern(const std::string& p)
{
    return p == "uniform" || p == "galling" || p == "abrasive";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.wear_depth_um < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": wear_depth_um must be >= 0 (got " +
              std::to_string(in.wear_depth_um) + ")");
    }
    if (in.accept_threshold_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": accept_threshold_um must be > 0 (got " +
              std::to_string(in.accept_threshold_um) + ")");
    }

    if (!isKnownPattern(in.wear_pattern)) {
        r.add("DFM-WEAR-PAT", "info",
              std::string(kSkillId) + ": wear_pattern '" + in.wear_pattern +
              "' unrecognised — expected 'uniform' / 'galling' / 'abrasive'");
    }

    if (in.accept_threshold_um > 0.0 && in.wear_depth_um > in.accept_threshold_um) {
        r.add("DFM-WEAR-FAIL", "info",
              std::string(kSkillId) + ": wear_depth_um " +
              std::to_string(in.wear_depth_um) +
              " exceeds accept_threshold_um " +
              std::to_string(in.accept_threshold_um) +
              " — schedule replace_part or rework");
    }
    if (in.wear_depth_um > 500.0) {
        r.add("DFM-WEAR-DEEP", "info",
              std::string(kSkillId) + ": wear_depth_um " +
              std::to_string(in.wear_depth_um) +
              " > 500 μm — suspect secondary damage");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "wear_inspect DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const bool acceptPass = (in.wear_depth_um <= in.accept_threshold_um);

    json params = {
        { "wear_depth_um",       in.wear_depth_um },
        { "wear_pattern",        in.wear_pattern },
        { "accept_threshold_um", in.accept_threshold_um },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "wear_depth_um",       in.wear_depth_um },
        { "wear_pattern",        in.wear_pattern },
        { "accept_threshold_um", in.accept_threshold_um },
        { "accept_pass",         acceptPass },
        { "geometry_changed",    false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "micrometer;profilometer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 300.0;       // ~ 5 min typical wear survey
    tooling.extra = {
        { "process",          "wear_inspect" },
        { "process_category", "maintenance" },
        { "geometric_no_op",  true },
        { "wear_pattern",     in.wear_pattern },
        { "accept_pass",      acceptPass },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::wear_inspect applied: depth={}μm pattern={} threshold={}μm pass={}",
        in.wear_depth_um, in.wear_pattern, in.accept_threshold_um, acceptPass);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Wear measurement is sub-OCCT resolution.  Metadata replay only.

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

}  // namespace koocadcam::skill::wear_inspect
