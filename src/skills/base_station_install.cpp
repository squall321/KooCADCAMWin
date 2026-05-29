// @lat: [[engine/skills#base_station_install]]

#include "base_station_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::base_station_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.site_id.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": site_id must be non-empty");
    }
    if (in.equipment_kw <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": equipment_kw must be > 0 (got " +
              std::to_string(in.equipment_kw) + ")");
    } else if (in.equipment_kw > 50.0) {
        r.add("DFM-BS-POWER", "warning",
              std::string(kSkillId) + ": equipment_kw " +
              std::to_string(in.equipment_kw) +
              " exceeds 50 kW — large macro site needs HVAC / mains review");
    }
    if (in.sectors_count < 1) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": sectors_count must be >= 1 (got " +
              std::to_string(in.sectors_count) + ")");
    } else if (in.sectors_count != 1 &&
               in.sectors_count != 3 &&
               in.sectors_count != 6) {
        r.add("DFM-BS-SECTORS", "info",
              std::string(kSkillId) + ": sectors_count " +
              std::to_string(in.sectors_count) +
              " not in {1, 3, 6} — atypical for macro cells");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "base_station_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "site_id",       in.site_id },
        { "equipment_kw",  in.equipment_kw },
        { "sectors_count", in.sectors_count },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "site_id",          in.site_id },
        { "equipment_kw",     in.equipment_kw },
        { "sectors_count",    in.sectors_count },
        { "geometry_changed", false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "site_install_crew";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Base 4 h + 2 h per sector for commissioning.
    tooling.est_cycle_time_s  =
        14400.0 + 7200.0 * static_cast<double>(in.sectors_count);
    tooling.extra = {
        { "process",          "telecom_base_station_install" },
        { "site_id",          in.site_id },
        { "equipment_kw",     in.equipment_kw },
        { "sectors_count",    in.sectors_count },
        { "process_category", "telecom_install" },
        { "geometric_no_op",  true },
        { "dfm_findings",     findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::base_station_install applied: site={} power={}kW sectors={}",
        in.site_id, in.equipment_kw, in.sectors_count);

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

}  // namespace koocadcam::skill::base_station_install
