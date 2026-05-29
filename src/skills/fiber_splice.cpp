// @lat: [[engine/skills#fiber_splice]]

#include "fiber_splice.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>

namespace koocadcam::skill::fiber_splice {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.loss_db < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": loss_db must be ≥ 0 (got " +
              std::to_string(in.loss_db) + ")");
    } else if (in.loss_db > 1.0) {
        r.add("DFM-SPLICE-LOSS-HIGH", "error",
              std::string(kSkillId) + ": loss_db " +
              std::to_string(in.loss_db) +
              " > 1.0 dB — splice failure threshold; re-cleave and re-splice");
    } else if (in.loss_db > 0.3) {
        r.add("DFM-SPLICE-LOSS-INFO", "info",
              std::string(kSkillId) + ": loss_db " +
              std::to_string(in.loss_db) +
              " > 0.3 dB — marginal; verify cleave quality + alignment");
    }

    if (in.splice_method != "arc" && in.splice_method != "laser") {
        r.add("DFM-SPLICE-METHOD", "info",
              std::string(kSkillId) + ": splice_method '" + in.splice_method +
              "' unrecognised — expected 'arc' or 'laser'");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "fiber_splice DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Field splices are always protected by a heat-shrink reinforcement
    // sleeve; lab splices may not be.  Arc-fusion = field grade.
    const bool protectedJoint = (in.splice_method == "arc");

    json params = {
        { "loss_db",          in.loss_db },
        { "splice_method",    in.splice_method },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "loss_db",          in.loss_db },
        { "splice_method",    in.splice_method },
        { "protected",        protectedJoint },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         =
        in.splice_method == "laser" ? "co2_laser_splicer" : "arc_fusion_splicer";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     =
        in.splice_method == "laser" ? "co2_laser_optics"
                                    : "tungsten_electrode";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // joining, not removal
    // Arc fusion: ~ 10 s cycle.  Laser fusion: ~ 20 s.  Plus protect sleeve.
    tooling.est_cycle_time_s  =
        (in.splice_method == "laser" ? 20.0 : 10.0)
        + (protectedJoint ? 60.0 : 0.0);
    tooling.extra = {
        { "process",       "fusion_splice" },
        { "splice_method", in.splice_method },
        { "loss_db",       in.loss_db },
        { "protected",     protectedJoint },
        { "dfm_findings",  findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::fiber_splice applied: method={} loss={}dB",
                  in.splice_method, in.loss_db);

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

}  // namespace koocadcam::skill::fiber_splice
