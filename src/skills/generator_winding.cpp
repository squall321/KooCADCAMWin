// @lat: [[engine/skills#generator_winding]]

#include "generator_winding.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <string>

namespace koocadcam::skill::generator_winding {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.conductor_dia_mm <= 0.05 || in.conductor_dia_mm > 5.0) {
        r.add("DFM-WIND-DIA", "error",
              "generator_winding conductor_dia_mm " +
              std::to_string(in.conductor_dia_mm) +
              " outside magnet-wire range (0.05, 5.0] mm");
    }
    if (in.slot_count <= 0) {
        r.add("DFM-INPUT", "error",
              "generator_winding slot_count must be > 0 (got " +
              std::to_string(in.slot_count) + ")");
    }
    if (in.turns_per_coil <= 0) {
        r.add("DFM-INPUT", "error",
              "generator_winding turns_per_coil must be > 0 (got " +
              std::to_string(in.turns_per_coil) + ")");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "generator_winding DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Per-coil bare-copper area (mm²) — diagnostic for fill-factor.
    const double conductor_area_mm2 =
        0.25 * M_PI * in.conductor_dia_mm * in.conductor_dia_mm;

    json params = {
        { "conductor_dia_mm", in.conductor_dia_mm },
        { "slot_count",       in.slot_count },
        { "turns_per_coil",   in.turns_per_coil },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "conductor_dia_mm",   in.conductor_dia_mm },
        { "slot_count",         in.slot_count },
        { "turns_per_coil",     in.turns_per_coil },
        { "conductor_area_mm2", conductor_area_mm2 },
        { "geometry_changed",   false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "coil_winder";
    tooling.tool_dia_mm       = in.conductor_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "enameled_copper";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 0.5 s per turn per slot.
    tooling.est_cycle_time_s  = 0.5 * static_cast<double>(in.slot_count) *
                                static_cast<double>(in.turns_per_coil);
    tooling.extra = {
        { "process",            "stator_winding" },
        { "conductor_dia_mm",   in.conductor_dia_mm },
        { "slot_count",         in.slot_count },
        { "turns_per_coil",     in.turns_per_coil },
        { "conductor_area_mm2", conductor_area_mm2 },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::generator_winding applied: dia={} slots={} turns={}",
                  in.conductor_dia_mm, in.slot_count, in.turns_per_coil);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata-only) ──────────────────────────────────────────

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

}  // namespace koocadcam::skill::generator_winding
