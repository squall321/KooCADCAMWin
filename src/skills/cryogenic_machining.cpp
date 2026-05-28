// @lat: [[engine/skills#cryogenic_machining]]

#include "cryogenic_machining.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>

namespace koocadcam::skill::cryogenic_machining {

using nlohmann::json;

namespace {

bool isKnownDelivery(const std::string& d)
{
    return d == "through_spindle" || d == "external_jet";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.sub_skill_id.empty()) {
        r.add("DFM-INPUT", "error",
              "cryogenic_machining: sub_skill_id must be non-empty "
              "(name the workhorse skill — e.g. \"drill_hole\")");
    }

    // LN2 flow rate envelope: (0, 20] litres/minute.
    if (in.flow_rate_lpm <= 0.0) {
        r.add("DFM-CRYO-FLOW", "error",
              "cryogenic_machining: flow_rate_lpm must be > 0 (got " +
              std::to_string(in.flow_rate_lpm) + ")");
    }
    if (in.flow_rate_lpm > 20.0) {
        r.add("DFM-CRYO-FLOW", "warning",
              "cryogenic_machining: flow_rate_lpm " +
              std::to_string(in.flow_rate_lpm) +
              " > 20 L/min — verify cryogenic vessel capacity / safety");
    }

    if (!isKnownDelivery(in.delivery)) {
        r.add("DFM-CRYO-DELIVERY", "info",
              "cryogenic_machining: delivery '" + in.delivery +
              "' unrecognised — expected 'through_spindle' | 'external_jet'");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Wrapper: in phase-1 we do NOT actually dispatch the sub-skill (that
// requires a dynamic skill registry).  Instead we clone the workpiece
// geometry unchanged and stamp a wrapper signature with the sub-skill
// reference.  Process planners are expected to expand this into the
// real sub-skill call + cryogenic tooling directives at codegen time.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cryogenic_machining DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "sub_skill_id",     in.sub_skill_id },
        { "sub_skill_params", in.sub_skill_params },
        { "flow_rate_lpm",    in.flow_rate_lpm },
        { "delivery",         in.delivery },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "sub_skill_id",     in.sub_skill_id },
        { "coolant",          "ln2" },
        { "flow_rate_lpm",    in.flow_rate_lpm },
        { "delivery",         in.delivery },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "cryogenic_delivery_nozzle";
    tooling.tool_dia_mm       = 4.0;       // typical LN2 nozzle bore
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_steel_insulated";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;       // wrapper itself doesn't cut
    tooling.est_cycle_time_s  = 0.0;       // sub-skill carries the time
    tooling.extra["coolant"]       = "ln2";
    tooling.extra["coolant_temp_c"]= -196.0;
    tooling.extra["flow_rate_lpm"] = in.flow_rate_lpm;
    tooling.extra["delivery"]      = in.delivery;
    tooling.extra["cryogenic"]     = true;
    tooling.extra["sub_skill_id"]  = in.sub_skill_id;
    tooling.extra["dfm_findings"]  = findings;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // Geometry unchanged — clone shape + material, copy prior features.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::cryogenic_machining applied: sub={} flow={} delivery={}",
                  in.sub_skill_id, in.flow_rate_lpm, in.delivery);

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

}  // namespace koocadcam::skill::cryogenic_machining
