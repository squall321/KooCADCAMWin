// @lat: [[engine/skills#bag_seal]]

#include "bag_seal.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <unordered_set>

namespace koocadcam::skill::bag_seal {

using nlohmann::json;

namespace {

const std::unordered_set<std::string>& knownSealTypes()
{
    static const std::unordered_set<std::string> s = {
        "heat", "ultrasonic", "induction",
    };
    return s;
}

// Per-modality nominal seal-temperature envelopes (°C).  Outside these
// ranges is INFO-only, not blocking.
struct TempEnvelope { double lo_c; double hi_c; };

TempEnvelope envelopeFor(const std::string& seal_type)
{
    if (seal_type == "heat")       return { 120.0, 230.0 };
    if (seal_type == "ultrasonic") return {  20.0, 120.0 }; // mostly friction
    if (seal_type == "induction")  return {  80.0, 200.0 };
    return { 0.0, 1000.0 };  // unknown — skip range check
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.seal_type.empty() || knownSealTypes().count(in.seal_type) == 0) {
        r.add("DFM-INPUT", "error",
              "bag_seal unknown seal_type '" + in.seal_type +
              "' (expected heat | ultrasonic | induction)");
    }

    if (in.seal_time_s <= 0.0) {
        r.add("DFM-SEAL-TIME", "error",
              "bag_seal seal_time_s must be > 0 (got " +
              std::to_string(in.seal_time_s) + ")");
    }

    if (knownSealTypes().count(in.seal_type) > 0) {
        const auto env = envelopeFor(in.seal_type);
        if (in.seal_temp_c < env.lo_c) {
            r.add("DFM-SEAL-TEMP-LOW", "info",
                  "bag_seal seal_temp_c " + std::to_string(in.seal_temp_c) +
                  " °C below typical [" + std::to_string(env.lo_c) +
                  ", " + std::to_string(env.hi_c) + "] for '" + in.seal_type + "'");
        } else if (in.seal_temp_c > env.hi_c) {
            r.add("DFM-SEAL-TEMP-HIGH", "info",
                  "bag_seal seal_temp_c " + std::to_string(in.seal_temp_c) +
                  " °C above typical [" + std::to_string(env.lo_c) +
                  ", " + std::to_string(env.hi_c) + "] for '" + in.seal_type +
                  "' — film degradation risk");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bag_seal DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "seal_type",   in.seal_type },
        { "seal_temp_c", in.seal_temp_c },
        { "seal_time_s", in.seal_time_s },
    };

    json pattern = {
        { "kind",             kSkillId },
        { "seal_type",        in.seal_type },
        { "seal_temp_c",      in.seal_temp_c },
        { "seal_time_s",      in.seal_time_s },
        { "process_family",   "flexible_bag_sealing" },
        { "geometry_changed", false },
        { "is_process_only",  true },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code",     f.code },
                            { "severity", f.severity },
                            { "message",  f.message }});

    ToolingMeta tooling;
    if      (in.seal_type == "heat")       tooling.tool_type = "heat_seal_jaw";
    else if (in.seal_type == "ultrasonic") tooling.tool_type = "ultrasonic_horn";
    else if (in.seal_type == "induction")  tooling.tool_type = "induction_coil";
    else                                    tooling.tool_type = "bag_sealer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "ptfe_coated_bar";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = in.seal_time_s + 1.0;
    tooling.extra = {
        { "process",          "bag_sealing" },
        { "seal_type",        in.seal_type },
        { "seal_temp_c",      in.seal_temp_c },
        { "seal_time_s",      in.seal_time_s },
        { "process_category", "packaging" },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::bag_seal applied: type={} T={}°C t={}s",
                  in.seal_type, in.seal_temp_c, in.seal_time_s);

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

}  // namespace koocadcam::skill::bag_seal
