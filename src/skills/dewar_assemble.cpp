// @lat: [[engine/skills#dewar_assemble]]

#include "dewar_assemble.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::dewar_assemble {

using nlohmann::json;

namespace {

bool isKnownInsulation(const std::string& s)
{
    return s == "MLI" || s == "aerogel" || s == "perlite" || s == "none";
}

double estimatedHeatLeakWPerM2(const std::string& insulation)
{
    // Rough order-of-magnitude radiative + conductive loss at 77 K boundary.
    if (insulation == "MLI")     return 1.0;
    if (insulation == "aerogel") return 5.0;
    if (insulation == "perlite") return 8.0;
    return 30.0;   // bare vacuum gap, no MLI
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.inner_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) +
              " inner_dia_mm must be > 0 (got " +
              std::to_string(in.inner_dia_mm) + ")");
    }

    if (in.vacuum_jacket_thickness_mm < 5.0) {
        r.add("DFM-DEWAR-GAP", "error",
              std::string(kSkillId) +
              " vacuum_jacket_thickness_mm " +
              std::to_string(in.vacuum_jacket_thickness_mm) +
              " < 5 mm — gap too small to maintain MFP-limited vacuum regime");
    } else if (in.vacuum_jacket_thickness_mm > 200.0) {
        r.add("DFM-DEWAR-GAP", "error",
              std::string(kSkillId) +
              " vacuum_jacket_thickness_mm " +
              std::to_string(in.vacuum_jacket_thickness_mm) +
              " > 200 mm — convection re-onset, structural inefficiency");
    }

    if (!isKnownInsulation(in.insulation_type)) {
        r.add("DFM-DEWAR-INS", "info",
              std::string(kSkillId) + " insulation_type '" +
              in.insulation_type +
              "' not in {MLI, aerogel, perlite, none}");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "dewar_assemble DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double heatLeak = estimatedHeatLeakWPerM2(in.insulation_type);

    json params = {
        { "inner_dia_mm",               in.inner_dia_mm },
        { "vacuum_jacket_thickness_mm", in.vacuum_jacket_thickness_mm },
        { "insulation_type",            in.insulation_type },
    };
    json pattern = {
        { "kind",                          kSkillId },
        { "inner_dia_mm",                  in.inner_dia_mm },
        { "vacuum_jacket_thickness_mm",    in.vacuum_jacket_thickness_mm },
        { "insulation_type",               in.insulation_type },
        { "estimated_heat_leak_w_per_m2",  heatLeak },
        { "geometry_changed",              false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "vacuum_assembly_station";
    tooling.tool_dia_mm       = in.inner_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_steel_304L";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 3600.0 * 2.0;   // ~2 h for fit + pump-down
    tooling.extra = {
        { "process",                       "dewar_assemble" },
        { "insulation_type",               in.insulation_type },
        { "estimated_heat_leak_w_per_m2",  heatLeak },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::dewar_assemble applied: id={} jacket={}mm insul={}",
                  in.inner_dia_mm, in.vacuum_jacket_thickness_mm,
                  in.insulation_type);

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

}  // namespace koocadcam::skill::dewar_assemble
