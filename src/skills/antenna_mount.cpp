// @lat: [[engine/skills#antenna_mount]]

#include "antenna_mount.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::antenna_mount {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.antenna_type.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": antenna_type must be non-empty");
    }
    if (in.mount_height_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": mount_height_m must be > 0 (got " +
              std::to_string(in.mount_height_m) + ")");
    } else if (in.mount_height_m > 100.0) {
        r.add("DFM-ANT-HEIGHT", "warning",
              std::string(kSkillId) + ": mount_height_m " +
              std::to_string(in.mount_height_m) +
              " exceeds 100 m — tall tower needs structural / aviation review");
    }
    if (in.azimuth_deg < 0.0 || in.azimuth_deg >= 360.0) {
        r.add("DFM-ANT-AZIMUTH", "error",
              std::string(kSkillId) + ": azimuth_deg " +
              std::to_string(in.azimuth_deg) +
              " must be in [0, 360)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "antenna_mount DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "antenna_type",    in.antenna_type },
        { "mount_height_m",  in.mount_height_m },
        { "azimuth_deg",     in.azimuth_deg },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "antenna_type",     in.antenna_type },
        { "mount_height_m",   in.mount_height_m },
        { "azimuth_deg",      in.azimuth_deg },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "tower_rigging";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 20 min base + 30 s per metre of climb.
    tooling.est_cycle_time_s  = 1200.0 + 30.0 * in.mount_height_m;
    tooling.extra = {
        { "process",          "telecom_antenna_mount" },
        { "antenna_type",     in.antenna_type },
        { "mount_height_m",   in.mount_height_m },
        { "azimuth_deg",      in.azimuth_deg },
        { "process_category", "telecom_install" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::antenna_mount applied: type={} height={}m az={}deg",
        in.antenna_type, in.mount_height_m, in.azimuth_deg);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only: leaves no geometric trace on the workpiece.

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

}  // namespace koocadcam::skill::antenna_mount
