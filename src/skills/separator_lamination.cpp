// @lat: [[engine/skills#separator_lamination]]

#include "separator_lamination.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::separator_lamination {

using nlohmann::json;

namespace {

bool isKnownFilmMaterial(const std::string& m)
{
    return m == "PE"  || m == "PP" ||
           m == "PE_PP_PE"      ||      // tri-layer Celgard-style
           m == "ceramic_coated_PE" ||
           m == "ceramic_coated_PP";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.film_thickness_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "separator_lamination: film_thickness_um must be > 0 (got " +
              std::to_string(in.film_thickness_um) + ")");
    } else {
        if (in.film_thickness_um < 5.0) {
            r.add("DFM-SEP-THK", "error",
                  "separator_lamination: film_thickness_um " +
                  std::to_string(in.film_thickness_um) +
                  " < 5 μm — below practical separator integrity limit");
        }
        if (in.film_thickness_um > 40.0) {
            r.add("DFM-SEP-THK", "error",
                  "separator_lamination: film_thickness_um " +
                  std::to_string(in.film_thickness_um) +
                  " > 40 μm — penalises ionic conductivity / volumetric energy");
        }
    }

    if (in.lamination_temp_c <= 0.0) {
        r.add("DFM-INPUT", "error",
              "separator_lamination: lamination_temp_c must be > 0 (got " +
              std::to_string(in.lamination_temp_c) + ")");
    } else {
        if (in.lamination_temp_c < 60.0) {
            r.add("DFM-SEP-TEMP-LOW", "error",
                  "separator_lamination: lamination_temp_c " +
                  std::to_string(in.lamination_temp_c) +
                  " < 60 °C — adhesion below tack-bond threshold");
        }
        if (in.lamination_temp_c > 130.0) {
            r.add("DFM-SEP-TEMP-HIGH", "error",
                  "separator_lamination: lamination_temp_c " +
                  std::to_string(in.lamination_temp_c) +
                  " > 130 °C — risk of PE pore collapse / thermal-shutdown trigger");
        }
    }

    if (!isKnownFilmMaterial(in.film_material)) {
        r.add("DFM-SEP-MATERIAL", "info",
              "separator_lamination: film_material '" + in.film_material +
              "' not in standard separator table "
              "(expected PE|PP|PE_PP_PE|ceramic_coated_*)");
    }

    if (in.roll_pressure_bar < 1.0 || in.roll_pressure_bar > 20.0) {
        r.add("DFM-SEP-PRESSURE", "info",
              "separator_lamination: roll_pressure_bar " +
              std::to_string(in.roll_pressure_bar) +
              " outside typical [1, 20] bar window");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "separator_lamination DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "film_thickness_um", in.film_thickness_um },
        { "lamination_temp_c", in.lamination_temp_c },
        { "film_material",     in.film_material },
        { "roll_pressure_bar", in.roll_pressure_bar },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "film_thickness_um", in.film_thickness_um },
        { "lamination_temp_c", in.lamination_temp_c },
        { "film_material",     in.film_material },
        { "roll_pressure_bar", in.roll_pressure_bar },
        { "geometry_changed",  false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "heated_lamination_roll";
    tooling.tool_dia_mm       = 200.0;      // typical lamination drum diameter
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "chrome_plated_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Web speed ≈ 20 m/min → 0.33 m/s; treat cycle as a fixed-time pass.
    tooling.est_cycle_time_s  = 30.0;
    tooling.extra = {
        { "process",           "separator_thermal_lamination" },
        { "film_thickness_um", in.film_thickness_um },
        { "lamination_temp_c", in.lamination_temp_c },
        { "film_material",     in.film_material },
        { "process_category",  "battery_manufacturing" },
        { "geometric_no_op",   true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::separator_lamination applied: film={} thk={}μm temp={}°C",
        in.film_material, in.film_thickness_um, in.lamination_temp_c);

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

}  // namespace koocadcam::skill::separator_lamination
