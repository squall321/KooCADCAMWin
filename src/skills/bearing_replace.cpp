// @lat: [[engine/skills#bearing_replace]]

#include "bearing_replace.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::bearing_replace {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bearing_id.empty()) {
        r.add("DFM-INPUT", "error",
              "bearing_replace bearing_id must be non-empty");
    }
    if (in.location.empty()) {
        r.add("DFM-INPUT", "error",
              "bearing_replace location must be non-empty");
    }

    if (in.install_torque_nm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "bearing_replace install_torque_nm must be > 0 (got " +
              std::to_string(in.install_torque_nm) + ")");
    } else if (in.install_torque_nm < 5.0 || in.install_torque_nm > 500.0) {
        r.add("DFM-BEARING-TORQUE", "info",
              "bearing_replace install_torque_nm " +
              std::to_string(in.install_torque_nm) +
              " outside typical 5–500 Nm range");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bearing_replace DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "bearing_id",        in.bearing_id },
        { "location",          in.location },
        { "install_torque_nm", in.install_torque_nm },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "bearing_id",        in.bearing_id },
        { "location",          in.location },
        { "install_torque_nm", in.install_torque_nm },
        { "geometry_changed",  false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "bearing_puller_and_torque_wrench";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;       // maintenance — no removal
    tooling.est_cycle_time_s  = 1800.0;    // ~30 min PM job
    tooling.extra = {
        { "process",           "bearing_swap" },
        { "bearing_id",        in.bearing_id },
        { "location",          in.location },
        { "install_torque_nm", in.install_torque_nm },
        { "dfm_findings",      findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::bearing_replace applied: id={} loc={} torque={} Nm",
                  in.bearing_id, in.location, in.install_torque_nm);

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

}  // namespace koocadcam::skill::bearing_replace
