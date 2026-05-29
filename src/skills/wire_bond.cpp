// @lat: [[engine/skills#wire_bond]]

#include "wire_bond.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::wire_bond {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.wire_dia_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "wire_bond wire_dia_um must be > 0");
    } else {
        if (in.wire_dia_um < 15.0) {
            r.add("DFM-WIRE-DIA", "error",
                  "wire_bond wire_dia_um " + std::to_string(in.wire_dia_um) +
                  " < 15 um — wire too fragile for production");
        }
        if (in.wire_dia_um > 75.0) {
            r.add("DFM-WIRE-DIA", "error",
                  "wire_bond wire_dia_um " + std::to_string(in.wire_dia_um) +
                  " > 75 um — use heavy wire / ribbon bonder instead");
        }
    }

    if (in.bond_count <= 0) {
        r.add("DFM-INPUT", "error",
              "wire_bond bond_count must be > 0");
    }

    if (in.wire_material != "Au" && in.wire_material != "Cu" &&
        in.wire_material != "Al") {
        r.add("DFM-WIRE-MAT", "info",
              "wire_bond wire_material '" + in.wire_material +
              "' unrecognised — expected 'Au' / 'Cu' / 'Al'");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "wire_bond DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string bonder_type =
        (in.wire_material == "Cu") ? "thermosonic_Cu_bonder" :
        (in.wire_material == "Al") ? "ultrasonic_wedge_bonder" :
                                     "thermosonic_Au_bonder";

    json params = {
        { "wire_dia_um",   in.wire_dia_um },
        { "wire_material", in.wire_material },
        { "bond_count",    in.bond_count },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "wire_dia_um",      in.wire_dia_um },
        { "wire_material",    in.wire_material },
        { "bond_count",       in.bond_count },
        { "bonder_type",      bonder_type },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = bonder_type;
    tooling.tool_dia_mm       = in.wire_dia_um / 1000.0;     // um → mm
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "tungsten_carbide_capillary";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 80 ms / bond on modern bonders; baseline for the loop set.
    tooling.est_cycle_time_s  = 0.08 * static_cast<double>(in.bond_count);
    tooling.extra = {
        { "process",       "wire_bond" },
        { "wire_dia_um",   in.wire_dia_um },
        { "wire_material", in.wire_material },
        { "bond_count",    in.bond_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::wire_bond applied: dia={}um mat={} bonds={}",
                  in.wire_dia_um, in.wire_material, in.bond_count);

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

}  // namespace koocadcam::skill::wire_bond
