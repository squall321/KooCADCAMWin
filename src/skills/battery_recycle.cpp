// @lat: [[engine/skills#battery_recycle]]

#include "battery_recycle.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::battery_recycle {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.battery_chemistry.empty()) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": battery_chemistry must be non-empty");
    } else if (in.battery_chemistry != "Li-ion" &&
               in.battery_chemistry != "NiMH"  &&
               in.battery_chemistry != "Pb-acid") {
        r.add("DFM-BATT-CHEM", "info",
              std::string(kSkillId) + ": battery_chemistry '" +
              in.battery_chemistry +
              "' unrecognised — expected 'Li-ion', 'NiMH', or 'Pb-acid'");
    }

    if (in.mass_kg <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": mass_kg must be > 0 (got " +
              std::to_string(in.mass_kg) + ")");
    }

    if (in.recovery_pct <= 0.0 || in.recovery_pct > 100.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": recovery_pct must be in (0, 100] (got " +
              std::to_string(in.recovery_pct) + ")");
    } else if (in.recovery_pct > 95.0) {
        r.add("DFM-BATT-RECOVERY", "info",
              std::string(kSkillId) + ": recovery_pct " +
              std::to_string(in.recovery_pct) +
              " exceeds 95% — verify pilot-scale yield");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "battery_recycle DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double recoveredMassKg = in.mass_kg * in.recovery_pct / 100.0;

    json params = {
        { "battery_chemistry", in.battery_chemistry },
        { "mass_kg",           in.mass_kg },
        { "recovery_pct",      in.recovery_pct },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "battery_chemistry",  in.battery_chemistry },
        { "mass_kg",            in.mass_kg },
        { "recovery_pct",       in.recovery_pct },
        { "recovered_mass_kg",  recoveredMassKg },
        { "geometry_changed",   false },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "battery_recycle_line";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Pyromet / hydromet line: ~120 s/kg + 30 s setup.
    tooling.est_cycle_time_s  = 30.0 + 120.0 * in.mass_kg;
    tooling.extra = {
        { "process",           "battery_recycle" },
        { "battery_chemistry", in.battery_chemistry },
        { "mass_kg",           in.mass_kg },
        { "recovered_mass_kg", recoveredMassKg },
        { "process_category",  "circular_economy" },
        { "geometric_no_op",   true },
        { "dfm_findings",      findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::battery_recycle applied: chem={} mass={}kg recovery={}% recovered={}kg",
        in.battery_chemistry, in.mass_kg, in.recovery_pct, recoveredMassKg);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata-only: leaves no geometric trace.

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

}  // namespace koocadcam::skill::battery_recycle
