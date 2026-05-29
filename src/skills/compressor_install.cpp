// @lat: [[engine/skills#compressor_install]]

#include "compressor_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::compressor_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.compressor_type != "centrifugal" &&
        in.compressor_type != "reciprocating") {
        r.add("DFM-CI-TYPE", "info",
              std::string(kSkillId) + ": compressor_type '" +
              in.compressor_type +
              "' not in {centrifugal, reciprocating} — treated as centrifugal");
    }

    if (in.power_kw <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": power_kw must be > 0 (got " +
              std::to_string(in.power_kw) + ")");
    } else if (in.power_kw > 50000.0) {
        r.add("DFM-CI-POWER", "info",
              std::string(kSkillId) + ": power_kw " +
              std::to_string(in.power_kw) +
              " > 50 MW — very large machine, verify driver redundancy");
    } else if (in.power_kw < 10.0) {
        r.add("DFM-CI-POWER", "info",
              std::string(kSkillId) + ": power_kw " +
              std::to_string(in.power_kw) +
              " < 10 kW — micro-compressor class, verify duty");
    }

    if (in.suction_pressure_kpa <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": suction_pressure_kpa must be > 0 (got " +
              std::to_string(in.suction_pressure_kpa) + ")");
    } else if (in.suction_pressure_kpa > 10000.0) {
        r.add("DFM-CI-PRESS", "info",
              std::string(kSkillId) + ": suction_pressure_kpa " +
              std::to_string(in.suction_pressure_kpa) +
              " > 10 MPa — HP suction class, verify ASME flange rating");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "compressor_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string cType =
        (in.compressor_type == "centrifugal" ||
         in.compressor_type == "reciprocating")
            ? in.compressor_type : std::string("centrifugal");

    json params = {
        { "compressor_type",      cType },
        { "power_kw",             in.power_kw },
        { "suction_pressure_kpa", in.suction_pressure_kpa },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "compressor_type",      cType },
        { "power_kw",             in.power_kw },
        { "suction_pressure_kpa", in.suction_pressure_kpa },
        { "geometry_changed",     false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "crane_alignment_spread";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Recip needs ~2× the alignment time of centrifugal.
    const double base = (cType == "reciprocating") ? 86400.0 : 43200.0;
    // Add 2 hr per MW for cool-down loop hook-up.
    tooling.est_cycle_time_s  = base + 7200.0 * (in.power_kw / 1000.0);
    tooling.extra = {
        { "process",              "oilgas_compressor_install" },
        { "compressor_type",      cType },
        { "power_kw",             in.power_kw },
        { "suction_pressure_kpa", in.suction_pressure_kpa },
        { "process_category",     "facility_installation" },
        { "geometric_no_op",      true },
        { "dfm_findings",         findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::compressor_install applied: type={} power={}kW Psuc={}kPa",
        cType, in.power_kw, in.suction_pressure_kpa);

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

}  // namespace koocadcam::skill::compressor_install
