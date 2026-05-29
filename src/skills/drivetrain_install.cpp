// @lat: [[engine/skills#drivetrain_install]]

#include "drivetrain_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::drivetrain_install {

using nlohmann::json;

namespace {
bool isKnownDrivetrain(const std::string& t)
{
    return t == "FWD" || t == "RWD" || t == "AWD";
}
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (!isKnownDrivetrain(in.drivetrain_type)) {
        r.add("DFM-DRIVETRAIN-TYPE", "error",
              std::string(kSkillId) + ": drivetrain_type '" +
              in.drivetrain_type + "' not in {FWD, RWD, AWD}");
    }
    if (in.engine_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": engine_id must be non-empty");
    }
    if (in.transmission_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": transmission_id must be non-empty");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "drivetrain_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "drivetrain_type", in.drivetrain_type },
        { "engine_id",       in.engine_id },
        { "transmission_id", in.transmission_id },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "drivetrain_type",  in.drivetrain_type },
        { "engine_id",        in.engine_id },
        { "transmission_id",  in.transmission_id },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "drivetrain_marriage_station";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // AWD requires an extra driveline alignment — longer cycle.
    tooling.est_cycle_time_s  = (in.drivetrain_type == "AWD") ? 180.0 : 120.0;
    tooling.extra = {
        { "process",          "drivetrain_install" },
        { "drivetrain_type",  in.drivetrain_type },
        { "engine_id",        in.engine_id },
        { "transmission_id",  in.transmission_id },
        { "process_category", "automotive_assembly" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::drivetrain_install applied: type={} engine={} trans={}",
        in.drivetrain_type, in.engine_id, in.transmission_id);

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

}  // namespace koocadcam::skill::drivetrain_install
