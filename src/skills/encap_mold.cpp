// @lat: [[engine/skills#encap_mold]]

#include "encap_mold.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::encap_mold {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.cavity_count <= 0) {
        r.add("DFM-INPUT", "error",
              "encap_mold cavity_count must be > 0");
    } else {
        if (in.cavity_count > 256) {
            r.add("DFM-CAVITY", "error",
                  "encap_mold cavity_count " + std::to_string(in.cavity_count) +
                  " > 256 — beyond standard transfer-mold press capacity");
        }
    }

    if (in.transfer_pressure_kpa <= 0.0) {
        r.add("DFM-INPUT", "error",
              "encap_mold transfer_pressure_kpa must be > 0");
    } else {
        if (in.transfer_pressure_kpa < 3000.0) {
            r.add("DFM-PRESSURE", "error",
                  "encap_mold transfer_pressure_kpa " +
                  std::to_string(in.transfer_pressure_kpa) +
                  " < 3000 kPa — insufficient cavity fill, risk of voids");
        }
        if (in.transfer_pressure_kpa > 12000.0) {
            r.add("DFM-PRESSURE", "error",
                  "encap_mold transfer_pressure_kpa " +
                  std::to_string(in.transfer_pressure_kpa) +
                  " > 12000 kPa — risk of wire sweep and flash");
        }
    }

    if (in.molding_compound.empty()) {
        r.add("DFM-COMPOUND", "info",
              "encap_mold molding_compound empty — defaulting to 'EMC_G700'");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "encap_mold DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string compound =
        in.molding_compound.empty() ? "EMC_G700" : in.molding_compound;

    json params = {
        { "molding_compound",      compound },
        { "cavity_count",          in.cavity_count },
        { "transfer_pressure_kpa", in.transfer_pressure_kpa },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "molding_compound",       compound },
        { "cavity_count",           in.cavity_count },
        { "transfer_pressure_kpa",  in.transfer_pressure_kpa },
        { "mold_temperature_c",     175.0 },
        { "cure_time_s",            90.0 },
        { "geometry_changed",       false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "transfer_mold_press";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tool_steel_S136";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 25 s + cure cycle per shot, divided across cavities for per-unit time.
    tooling.est_cycle_time_s  = 115.0 / static_cast<double>(in.cavity_count);
    tooling.extra = {
        { "process",               "encap_mold" },
        { "molding_compound",      compound },
        { "cavity_count",          in.cavity_count },
        { "transfer_pressure_kpa", in.transfer_pressure_kpa },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::encap_mold applied: emc={} cavities={} P={}kPa",
                  compound, in.cavity_count, in.transfer_pressure_kpa);

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

}  // namespace koocadcam::skill::encap_mold
