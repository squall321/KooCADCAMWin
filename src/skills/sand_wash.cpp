// @lat: [[engine/skills#sand_wash]]

#include "sand_wash.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::sand_wash {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.throughput_kg_h <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": throughput_kg_h must be > 0 (got " +
              std::to_string(in.throughput_kg_h) + ")");
    }
    if (in.water_l_kg < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": water_l_kg must be ≥ 0 (got " +
              std::to_string(in.water_l_kg) + ")");
    }

    if (in.final_fineness_modulus < 2.0 ||
        in.final_fineness_modulus > 3.5) {
        r.add("DFM-SAND-FM-RANGE", "info",
              std::string(kSkillId) + ": final_fineness_modulus " +
              std::to_string(in.final_fineness_modulus) +
              " outside concrete-sand band [2.0, 3.5]");
    }

    if (in.water_l_kg > 0.0 && in.water_l_kg < 0.3) {
        r.add("DFM-SAND-WATER-LOW", "info",
              std::string(kSkillId) + ": water_l_kg " +
              std::to_string(in.water_l_kg) + " < 0.3 — likely under-washed");
    }
    if (in.water_l_kg > 5.0) {
        r.add("DFM-SAND-WATER-HIGH", "info",
              std::string(kSkillId) + ": water_l_kg " +
              std::to_string(in.water_l_kg) +
              " > 5.0 — wasteful, consider water recycling");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sand_wash DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double waterTotal = in.throughput_kg_h * in.water_l_kg;

    json params = {
        { "throughput_kg_h",        in.throughput_kg_h },
        { "final_fineness_modulus", in.final_fineness_modulus },
        { "water_l_kg",             in.water_l_kg },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "throughput_kg_h",         in.throughput_kg_h },
        { "final_fineness_modulus",  in.final_fineness_modulus },
        { "water_l_kg",              in.water_l_kg },
        { "water_total_l_h",         waterTotal },
        { "geometry_changed",        false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "sand_wash_screw";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "abrasion_resistant_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // 1 kg processed per (1 / throughput_kg_h) hours = 3600/throughput seconds
    tooling.est_cycle_time_s  =
        (in.throughput_kg_h > 0.0) ? 3600.0 / in.throughput_kg_h : 0.0;
    tooling.extra = {
        { "process",                 "aggregate_washing" },
        { "throughput_kg_h",         in.throughput_kg_h },
        { "final_fineness_modulus",  in.final_fineness_modulus },
        { "water_l_kg",              in.water_l_kg },
        { "water_total_l_h",         waterTotal },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::sand_wash applied: throughput={}kg/h FM={} water={}L/kg",
                  in.throughput_kg_h, in.final_fineness_modulus, in.water_l_kg);

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

}  // namespace koocadcam::skill::sand_wash
