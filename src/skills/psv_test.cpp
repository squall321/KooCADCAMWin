// @lat: [[engine/skills#psv_test]]

#include "psv_test.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::psv_test {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.set_pressure_kpa <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": set_pressure_kpa must be > 0 (got " +
              std::to_string(in.set_pressure_kpa) + ")");
    }
    if (in.blowdown_pct <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": blowdown_pct must be > 0 (got " +
              std::to_string(in.blowdown_pct) + ")");
    } else {
        if (in.blowdown_pct < 2.0) {
            r.add("DFM-PSV-BLOWDOWN", "error",
                  std::string(kSkillId) + ": blowdown_pct " +
                  std::to_string(in.blowdown_pct) +
                  " < 2 % — reseat too close to set, chatter risk");
        }
        if (in.blowdown_pct > 15.0) {
            r.add("DFM-PSV-BLOWDOWN", "error",
                  std::string(kSkillId) + ": blowdown_pct " +
                  std::to_string(in.blowdown_pct) +
                  " > 15 % — poor reseating, excessive system depressurisation");
        }
    }

    if (in.set_pressure_kpa > 50000.0) {
        r.add("DFM-PSV-PRESSURE", "info",
              std::string(kSkillId) + ": set_pressure_kpa " +
              std::to_string(in.set_pressure_kpa) +
              " > 50000 (≈500 bar) — ultra-high pressure regime, requires "
              "certified test stand and witnessing inspector");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double reseatPressureKpa =
        in.set_pressure_kpa * (1.0 - in.blowdown_pct / 100.0);

    json params = {
        { "set_pressure_kpa", in.set_pressure_kpa },
        { "blowdown_pct",     in.blowdown_pct },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "set_pressure_kpa",     in.set_pressure_kpa },
        { "blowdown_pct",         in.blowdown_pct },
        { "reseat_pressure_kpa",  reseatPressureKpa },
        { "geometry_changed",     false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({ { "code", f.code },
                             { "severity", f.severity },
                             { "message", f.message } });

    ToolingMeta tooling;
    tooling.tool_type         = "psv_test_stand";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n2_pressure_gas";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Single-cycle PSV test ≈ 5 min (300 s).
    tooling.est_cycle_time_s  = 300.0;
    tooling.extra = {
        { "process",              "psv_qualification_test" },
        { "set_pressure_kpa",     in.set_pressure_kpa },
        { "blowdown_pct",         in.blowdown_pct },
        { "reseat_pressure_kpa",  reseatPressureKpa },
        { "dfm_findings",         findings },
        { "process_category",     "pressure_vessel_qa" },
        { "geometric_no_op",      true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::psv_test applied: P_set={} kPa blowdown={}% reseat={} kPa",
                  in.set_pressure_kpa, in.blowdown_pct, reseatPressureKpa);

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

}  // namespace koocadcam::skill::psv_test
