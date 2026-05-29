// @lat: [[engine/skills#rocket_propellant_load]]

#include "rocket_propellant_load.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::rocket_propellant_load {

using nlohmann::json;

namespace {

struct PropellantSpec
{
    const char* name;
    double      load_temp_min_c;
    double      load_temp_max_c;
    const char* family;       // "cryogenic" | "storable"
};

const PropellantSpec* lookupSpec(const std::string& type)
{
    static const PropellantSpec table[] = {
        // Boiling points / canonical load windows (°C)
        { "LOX",  -200.0, -180.0, "cryogenic" },
        { "LH2",  -260.0, -250.0, "cryogenic" },
        { "RP-1",   -20.0,   40.0, "storable"  },
        { "N2O4",    -5.0,   30.0, "storable"  },
        { "UDMH",   -10.0,   40.0, "storable"  },
    };
    for (const auto& p : table)
        if (type == p.name) return &p;
    return nullptr;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const PropellantSpec* spec = lookupSpec(in.propellant_type);
    if (spec == nullptr) {
        r.add("DFM-PROP-TYPE", "error",
              "rocket_propellant_load propellant_type '" + in.propellant_type +
              "' not in {LOX, RP-1, LH2, N2O4, UDMH}");
    }

    if (in.mass_kg <= 0.0) {
        r.add("DFM-PROP-MASS", "error",
              "rocket_propellant_load mass_kg " + std::to_string(in.mass_kg) +
              " must be > 0");
    } else if (in.mass_kg > 5.0e6) {
        r.add("DFM-PROP-MASS", "error",
              "rocket_propellant_load mass_kg " + std::to_string(in.mass_kg) +
              " > 5,000,000 kg — exceeds heaviest known stage propellant load");
    }

    if (spec != nullptr) {
        if (in.load_temp_c < spec->load_temp_min_c ||
            in.load_temp_c > spec->load_temp_max_c) {
            r.add("DFM-PROP-TEMP", "error",
                  "rocket_propellant_load load_temp_c " +
                  std::to_string(in.load_temp_c) +
                  " outside [" + std::to_string(spec->load_temp_min_c) +
                  ", " + std::to_string(spec->load_temp_max_c) +
                  "] for " + in.propellant_type);
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
        std::string msg = "rocket_propellant_load DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const PropellantSpec* spec = lookupSpec(in.propellant_type);
    const std::string family   = spec ? spec->family : "unknown";

    json params = {
        { "propellant_type", in.propellant_type },
        { "mass_kg",         in.mass_kg },
        { "load_temp_c",     in.load_temp_c },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "propellant_type",  in.propellant_type },
        { "mass_kg",          in.mass_kg },
        { "load_temp_c",      in.load_temp_c },
        { "propellant_family", family },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         =
        (family == "cryogenic") ? "cryo_transfer_line" : "storable_transfer_pump";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     =
        (family == "cryogenic") ? "stainless_steel_316L" : "stainless_steel_304";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Rough estimate: 1000 kg/min storable, 500 kg/min cryo (chill-down).
    {
        const double rate_kg_per_min = (family == "cryogenic") ? 500.0 : 1000.0;
        tooling.est_cycle_time_s =
            (in.mass_kg / rate_kg_per_min) * 60.0;
    }
    tooling.extra = {
        { "process",          "rocket_propellant_transfer" },
        { "propellant_type",  in.propellant_type },
        { "propellant_family", family },
        { "mass_kg",          in.mass_kg },
        { "load_temp_c",      in.load_temp_c },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::rocket_propellant_load applied: type={} mass={}kg T={}C",
                  in.propellant_type, in.mass_kg, in.load_temp_c);

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

}  // namespace koocadcam::skill::rocket_propellant_load
