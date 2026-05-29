// @lat: [[engine/skills#flare_install]]

#include "flare_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::flare_install {

using nlohmann::json;

namespace {

bool isKnownIgnition(const std::string& t)
{
    return t == "pilot" || t == "flame_front_generator" || t == "ballistic";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.stack_height_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": stack_height_m must be > 0 (got " +
              std::to_string(in.stack_height_m) + ")");
    } else if (in.stack_height_m < 10.0) {
        r.add("DFM-FI-HEIGHT", "info",
              std::string(kSkillId) + ": stack_height_m " +
              std::to_string(in.stack_height_m) +
              " < 10 m — short-stack class, verify radiation footprint");
    } else if (in.stack_height_m > 120.0) {
        r.add("DFM-FI-HEIGHT", "info",
              std::string(kSkillId) + ": stack_height_m " +
              std::to_string(in.stack_height_m) +
              " > 120 m — tall-stack class, verify aviation lighting + guy-wire plan");
    }

    if (in.max_flow_nm3_h <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": max_flow_nm3_h must be > 0 (got " +
              std::to_string(in.max_flow_nm3_h) + ")");
    }

    if (in.ignition_type.empty()) {
        r.add("DFM-INPUT", "warning",
              std::string(kSkillId) + ": ignition_type empty — defaulting to 'pilot'");
    } else if (!isKnownIgnition(in.ignition_type)) {
        r.add("DFM-FI-IGN", "info",
              std::string(kSkillId) + ": ignition_type '" + in.ignition_type +
              "' not in {pilot, flame_front_generator, ballistic} — verify supplier");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "flare_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string ign = in.ignition_type.empty()
                                ? std::string("pilot") : in.ignition_type;

    json params = {
        { "stack_height_m",  in.stack_height_m },
        { "max_flow_nm3_h",  in.max_flow_nm3_h },
        { "ignition_type",   ign },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "stack_height_m",   in.stack_height_m },
        { "max_flow_nm3_h",   in.max_flow_nm3_h },
        { "ignition_type",    ign },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "crawler_crane_erection";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.stack_height_m * 1000.0;   // stack height in mm
    tooling.tool_material     = "stainless_steel_310";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Erection budget: 8 hr baseline + 1 hr per meter of stack.
    tooling.est_cycle_time_s  = 28800.0 + 3600.0 * in.stack_height_m;
    tooling.extra = {
        { "process",           "oilgas_flare_install" },
        { "stack_height_m",    in.stack_height_m },
        { "max_flow_nm3_h",    in.max_flow_nm3_h },
        { "ignition_type",     ign },
        { "process_category",  "facility_installation" },
        { "geometric_no_op",   true },
        { "dfm_findings",      findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::flare_install applied: height={}m flow={}Nm3/h ign={}",
        in.stack_height_m, in.max_flow_nm3_h, ign);

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

}  // namespace koocadcam::skill::flare_install
