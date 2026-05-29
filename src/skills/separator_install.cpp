// @lat: [[engine/skills#separator_install]]

#include "separator_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::separator_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.separator_type != "2-phase" && in.separator_type != "3-phase") {
        r.add("DFM-SI-TYPE", "info",
              std::string(kSkillId) + ": separator_type '" +
              in.separator_type +
              "' not in {2-phase, 3-phase} — treated as 3-phase for planning");
    }

    if (in.capacity_bpd <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": capacity_bpd must be > 0 (got " +
              std::to_string(in.capacity_bpd) + ")");
    } else if (in.capacity_bpd > 100000.0) {
        r.add("DFM-SI-CAP", "info",
              std::string(kSkillId) + ": capacity_bpd " +
              std::to_string(in.capacity_bpd) +
              " exceeds 100k BPD — verify vessel ID against major-facility class");
    }

    if (in.working_pressure_kpa <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": working_pressure_kpa must be > 0 (got " +
              std::to_string(in.working_pressure_kpa) + ")");
    } else if (in.working_pressure_kpa > 20000.0) {
        r.add("DFM-SI-PRESS", "info",
              std::string(kSkillId) + ": working_pressure_kpa " +
              std::to_string(in.working_pressure_kpa) +
              " > 20 MPa — HP-class vessel, verify ASME flange rating");
    } else if (in.working_pressure_kpa < 100.0) {
        r.add("DFM-SI-PRESS", "info",
              std::string(kSkillId) + ": working_pressure_kpa " +
              std::to_string(in.working_pressure_kpa) +
              " < 100 kPa — atmospheric duty, verify vessel code applicability");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "separator_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string sepType =
        (in.separator_type == "2-phase" || in.separator_type == "3-phase")
            ? in.separator_type : std::string("3-phase");

    json params = {
        { "separator_type",        sepType },
        { "capacity_bpd",          in.capacity_bpd },
        { "working_pressure_kpa",  in.working_pressure_kpa },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "separator_type",       sepType },
        { "capacity_bpd",         in.capacity_bpd },
        { "working_pressure_kpa", in.working_pressure_kpa },
        { "geometry_changed",     false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "crane_rigging_spread";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Set, level, anchor + nozzle hookup ~ 8 hr baseline + 2 hr per 10k BPD.
    tooling.est_cycle_time_s  =
        28800.0 + 720.0 * (in.capacity_bpd / 1000.0);
    tooling.extra = {
        { "process",              "oilgas_separator_install" },
        { "separator_type",       sepType },
        { "capacity_bpd",         in.capacity_bpd },
        { "working_pressure_kpa", in.working_pressure_kpa },
        { "process_category",     "facility_installation" },
        { "geometric_no_op",      true },
        { "dfm_findings",         findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::separator_install applied: type={} cap={}BPD WP={}kPa",
        sepType, in.capacity_bpd, in.working_pressure_kpa);

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

}  // namespace koocadcam::skill::separator_install
