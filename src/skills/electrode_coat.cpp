// @lat: [[engine/skills#electrode_coat]]

#include "electrode_coat.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>  // std::max

namespace koocadcam::skill::electrode_coat {

using nlohmann::json;

namespace {

bool isKnownActiveMaterial(const std::string& m)
{
    // Common Li-ion cathode + anode chemistries.
    return m == "NMC811" || m == "NMC622" || m == "NMC532" ||
           m == "NCA"    || m == "LFP"    || m == "LCO"    ||
           m == "LMO"    ||
           m == "graphite" || m == "silicon_graphite" ||
           m == "LTO"      || m == "hard_carbon";
}

bool isKnownCoatingMethod(const std::string& method)
{
    return method == "slot_die" || method == "comma_bar" ||
           method == "doctor_blade";
}

bool isKnownFoilMaterial(const std::string& f)
{
    return f == "aluminum" || f == "copper";
}

// Anode chemistries (active material lives on copper foil).
bool isAnodeChemistry(const std::string& m)
{
    return m == "graphite" || m == "silicon_graphite" ||
           m == "hard_carbon";
    // LTO is an anode chemistry but is intentionally coated on Al foil.
}

// Cathode chemistries (active material lives on aluminum foil).
bool isCathodeChemistry(const std::string& m)
{
    return m == "NMC811" || m == "NMC622" || m == "NMC532" ||
           m == "NCA"    || m == "LFP"    || m == "LCO"    ||
           m == "LMO";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thickness_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "electrode_coat: thickness_um must be > 0 (got " +
              std::to_string(in.thickness_um) + ")");
    } else {
        if (in.thickness_um < 30.0) {
            r.add("DFM-COAT-THK", "error",
                  "electrode_coat: thickness_um " +
                  std::to_string(in.thickness_um) +
                  " < 30 μm — below practical slot-die wet-film coverage");
        }
        if (in.thickness_um > 250.0) {
            r.add("DFM-COAT-THK", "error",
                  "electrode_coat: thickness_um " +
                  std::to_string(in.thickness_um) +
                  " > 250 μm — exceeds single-pass dryer capacity");
        }
    }

    if (in.areal_capacity_mAh_cm2 <= 0.0) {
        r.add("DFM-INPUT", "error",
              "electrode_coat: areal_capacity_mAh_cm2 must be > 0 (got " +
              std::to_string(in.areal_capacity_mAh_cm2) + ")");
    } else if (in.areal_capacity_mAh_cm2 < 0.5 ||
               in.areal_capacity_mAh_cm2 > 6.0) {
        r.add("DFM-COAT-LOAD", "info",
              "electrode_coat: areal_capacity " +
              std::to_string(in.areal_capacity_mAh_cm2) +
              " mAh/cm² outside typical [0.5, 6.0] window");
    }

    if (in.electrode_side != "anode" && in.electrode_side != "cathode") {
        r.add("DFM-COAT-SIDE", "error",
              "electrode_coat: electrode_side '" + in.electrode_side +
              "' must be 'anode' or 'cathode'");
    }

    if (!isKnownActiveMaterial(in.active_material)) {
        r.add("DFM-COAT-MATERIAL", "info",
              "electrode_coat: active_material '" + in.active_material +
              "' not in standard chemistry table");
    }

    if (!isKnownCoatingMethod(in.coating_method)) {
        r.add("DFM-COAT-METHOD", "info",
              "electrode_coat: coating_method '" + in.coating_method +
              "' unrecognised (expected slot_die|comma_bar|doctor_blade)");
    }

    if (!isKnownFoilMaterial(in.collector_foil_material)) {
        r.add("DFM-COAT-FOIL", "info",
              "electrode_coat: collector_foil_material '" +
              in.collector_foil_material +
              "' unrecognised (expected aluminum|copper)");
    } else {
        // Foil/chemistry mismatch (info only — LTO/Al is a legitimate exception).
        if (in.electrode_side == "cathode" &&
            in.collector_foil_material == "copper" &&
            isCathodeChemistry(in.active_material)) {
            r.add("DFM-COAT-FOIL-MISMATCH", "info",
                  "electrode_coat: cathode chemistry '" + in.active_material +
                  "' on copper foil is unusual — Al foil is standard");
        }
        if (in.electrode_side == "anode" &&
            in.collector_foil_material == "aluminum" &&
            isAnodeChemistry(in.active_material)) {
            r.add("DFM-COAT-FOIL-MISMATCH", "info",
                  "electrode_coat: anode chemistry '" + in.active_material +
                  "' on aluminum foil is unusual — Cu foil is standard "
                  "(LTO is the typical Al-anode exception)");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "electrode_coat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "active_material",         in.active_material },
        { "electrode_side",          in.electrode_side },
        { "thickness_um",            in.thickness_um },
        { "areal_capacity_mAh_cm2",  in.areal_capacity_mAh_cm2 },
        { "collector_foil_material", in.collector_foil_material },
        { "coating_method",          in.coating_method },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "active_material",         in.active_material },
        { "electrode_side",          in.electrode_side },
        { "thickness_um",            in.thickness_um },
        { "areal_capacity_mAh_cm2",  in.areal_capacity_mAh_cm2 },
        { "collector_foil_material", in.collector_foil_material },
        { "coating_method",          in.coating_method },
        { "geometry_changed",        false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "slot_die_coater";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_die_lip";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Web speed ~ 30 m/min typical; treat cycle time as the dryer dwell.
    tooling.est_cycle_time_s  = std::max(60.0, in.thickness_um * 0.8);
    tooling.extra = {
        { "process",                "electrode_slurry_coat" },
        { "active_material",        in.active_material },
        { "electrode_side",         in.electrode_side },
        { "thickness_um",           in.thickness_um },
        { "areal_capacity_mAh_cm2", in.areal_capacity_mAh_cm2 },
        { "process_category",       "battery_manufacturing" },
        { "geometric_no_op",        true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::electrode_coat applied: side={} mat={} thk={}μm cap={}mAh/cm²",
        in.electrode_side, in.active_material,
        in.thickness_um, in.areal_capacity_mAh_cm2);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// A slurry coating leaves no macroscopic B-rep trace.  We can ONLY recognize
// a prior electrode_coat by replaying signatures previously stamped onto
// the workpiece.

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

}  // namespace koocadcam::skill::electrode_coat
