// @lat: [[engine/skills#ball_inflate]]

#include "ball_inflate.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::ball_inflate {

using nlohmann::json;

namespace {

struct Spec { double lo; double hi; };

bool specForType(const std::string& type, Spec& out)
{
    if (type == "soccer")     { out = { 8.5, 15.6 }; return true; }
    if (type == "basketball") { out = { 7.5,  8.5 }; return true; }
    if (type == "football")   { out = { 12.5, 13.5 }; return true; }
    if (type == "volleyball") { out = { 4.3,  4.6 }; return true; }
    if (type == "rugby")      { out = { 9.5, 10.0 }; return true; }
    return false;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.target_pressure_psi <= 0.0) {
        r.add("DFM-INPUT", "error",
              "ball_inflate target_pressure_psi must be > 0 (got " +
              std::to_string(in.target_pressure_psi) + ")");
    } else if (in.target_pressure_psi > 30.0) {
        r.add("DFM-BALL-PRESSURE", "error",
              "ball_inflate target_pressure_psi " +
              std::to_string(in.target_pressure_psi) +
              " > 30 psi — bladder burst risk");
    }

    if (in.ball_type.empty()) {
        r.add("DFM-INPUT", "warning",
              "ball_inflate ball_type empty");
    } else {
        Spec spec{};
        if (specForType(in.ball_type, spec)) {
            if (in.target_pressure_psi < spec.lo ||
                in.target_pressure_psi > spec.hi) {
                r.add("DFM-BALL-PRESSURE", "info",
                      "ball_inflate target " +
                      std::to_string(in.target_pressure_psi) +
                      " psi outside " + in.ball_type + " spec [" +
                      std::to_string(spec.lo) + ", " +
                      std::to_string(spec.hi) + "]");
            }
        } else {
            r.add("DFM-BALL-PRESSURE", "info",
                  "ball_inflate ball_type '" + in.ball_type +
                  "' unrecognised — no spec range applied");
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
        std::string msg = "ball_inflate DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    Spec spec{};
    const bool haveSpec = specForType(in.ball_type, spec);
    const bool inSpec   = haveSpec &&
                          in.target_pressure_psi >= spec.lo &&
                          in.target_pressure_psi <= spec.hi;

    json params = {
        { "target_pressure_psi", in.target_pressure_psi },
        { "ball_type",           in.ball_type },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "target_pressure_psi", in.target_pressure_psi },
        { "ball_type",           in.ball_type },
        { "in_spec_range",       inSpec },
        { "geometry_changed",    false },
    };
    if (haveSpec) {
        pattern["spec_lo_psi"] = spec.lo;
        pattern["spec_hi_psi"] = spec.hi;
    }

    ToolingMeta tooling;
    tooling.tool_type         = "pressure_pump";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "none";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~15 s pump + gauge verify.
    tooling.est_cycle_time_s  = 15.0;
    tooling.extra = {
        { "process",             "ball_inflation" },
        { "target_pressure_psi", in.target_pressure_psi },
        { "ball_type",           in.ball_type },
        { "in_spec_range",       inSpec },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::ball_inflate applied: type={} pressure={} psi in_spec={}",
                  in.ball_type, in.target_pressure_psi, inSpec);

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

}  // namespace koocadcam::skill::ball_inflate
