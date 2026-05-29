// @lat: [[engine/skills#hvac_install]]

#include "hvac_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::hvac_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.duct_size_mm <= 0.0 || in.duct_size_mm > 2000.0) {
        r.add("DFM-HVAC-DUCT", "error",
              std::string(kSkillId) + ": duct_size_mm " +
              std::to_string(in.duct_size_mm) +
              " outside (0, 2000] mm — non-standard duct size");
    }

    if (in.total_length_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": total_length_m must be > 0");
    }

    if (in.register_count < 1) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": register_count must be >= 1");
    }

    if (in.fan_kw <= 0.0 || in.fan_kw > 500.0) {
        r.add("DFM-HVAC-FAN", "error",
              std::string(kSkillId) + ": fan_kw " +
              std::to_string(in.fan_kw) +
              " outside (0, 500] kW — atypical fan size");
    }

    if (in.register_count > 0 && in.total_length_m > 0.0) {
        const double per = in.total_length_m / in.register_count;
        if (per > 30.0) {
            r.add("DFM-HVAC-COVERAGE", "info",
                  std::string(kSkillId) + ": " + std::to_string(per) +
                  " m duct per register > 30 m — long throw, balancing risk");
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
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "duct_size_mm",   in.duct_size_mm },
        { "total_length_m", in.total_length_m },
        { "register_count", in.register_count },
        { "fan_kw",         in.fan_kw },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "duct_size_mm",     in.duct_size_mm },
        { "total_length_m",   in.total_length_m },
        { "register_count",   in.register_count },
        { "fan_kw",           in.fan_kw },
        { "length_per_register_m",
          (in.register_count > 0) ? (in.total_length_m / in.register_count) : 0.0 },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "sheet_metal_install_crew";
    tooling.tool_dia_mm       = in.duct_size_mm;
    tooling.tool_length_mm    = in.total_length_m * 1000.0;
    tooling.tool_material     = "galvanized_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 30 min / 3 m duct + 1 h per register + fan commissioning.
    tooling.est_cycle_time_s  = in.total_length_m * 600.0
                              + in.register_count * 3600.0
                              + 7200.0;
    tooling.extra = {
        { "process",        "hvac_install" },
        { "duct_size_mm",   in.duct_size_mm },
        { "total_length_m", in.total_length_m },
        { "register_count", in.register_count },
        { "fan_kw",         in.fan_kw },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::hvac_install applied: duct={} len={} reg={} fan={}kW",
                  in.duct_size_mm, in.total_length_m, in.register_count, in.fan_kw);

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

}  // namespace koocadcam::skill::hvac_install
