// @lat: [[engine/skills#propellant_load]]

#include "propellant_load.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::propellant_load {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.propellant_type.empty()) {
        r.add("DFM-INPUT", "error",
              "propellant_load propellant_type must not be empty");
    }

    if (in.charge_grain <= 0.0) {
        r.add("DFM-AMMO-CHARGE", "error",
              "propellant_load charge_grain " + std::to_string(in.charge_grain) +
              " must be > 0");
    } else if (in.charge_grain > 200.0) {
        r.add("DFM-AMMO-CHARGE", "error",
              "propellant_load charge_grain " + std::to_string(in.charge_grain) +
              " > 200 gr — exceeds typical small-arms charge envelope");
    }

    if (in.lot_id.empty()) {
        r.add("DFM-AMMO-LOT", "error",
              "propellant_load lot_id must not be empty (traceability required)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "propellant_load DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "propellant_type", in.propellant_type },
        { "charge_grain",    in.charge_grain },
        { "lot_id",          in.lot_id },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "propellant_type",  in.propellant_type },
        { "charge_grain",     in.charge_grain },
        { "lot_id",           in.lot_id },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "powder_dispenser";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 2.0;
    tooling.extra = {
        { "process",         "propellant_charge" },
        { "propellant_type", in.propellant_type },
        { "charge_grain",    in.charge_grain },
        { "lot_id",          in.lot_id },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::propellant_load applied: type={} charge={} lot={}",
                  in.propellant_type, in.charge_grain, in.lot_id);

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

}  // namespace koocadcam::skill::propellant_load
