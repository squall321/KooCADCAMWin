// @lat: [[engine/skills#wheel_truing]]

#include "wheel_truing.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::wheel_truing {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.wheel_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": wheel_dia_mm must be > 0 (got " +
              std::to_string(in.wheel_dia_mm) + ")");
    } else {
        if (in.wheel_dia_mm < 780.0) {
            r.add("DFM-WT-DIA", "error",
                  std::string(kSkillId) + ": wheel_dia_mm " +
                  std::to_string(in.wheel_dia_mm) +
                  " < 780 mm — below scrap-limit minimum");
        }
        if (in.wheel_dia_mm > 1250.0) {
            r.add("DFM-WT-DIA", "error",
                  std::string(kSkillId) + ": wheel_dia_mm " +
                  std::to_string(in.wheel_dia_mm) +
                  " > 1250 mm — exceeds typical rolling-stock wheel range");
        }
        if (in.wheel_dia_mm >= 780.0 && in.wheel_dia_mm < 840.0) {
            r.add("DFM-WT-SCRAP", "info",
                  std::string(kSkillId) + ": wheel_dia_mm " +
                  std::to_string(in.wheel_dia_mm) +
                  " approaching scrap diameter — schedule replacement");
        }
    }

    if (in.flange_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": flange_height_mm must be > 0 (got " +
              std::to_string(in.flange_height_mm) + ")");
    } else {
        if (in.flange_height_mm < 22.0) {
            r.add("DFM-WT-FLANGE", "error",
                  std::string(kSkillId) + ": flange_height_mm " +
                  std::to_string(in.flange_height_mm) +
                  " < 22 mm — below derailment safety limit");
        }
        if (in.flange_height_mm > 36.0) {
            r.add("DFM-WT-FLANGE", "error",
                  std::string(kSkillId) + ": flange_height_mm " +
                  std::to_string(in.flange_height_mm) +
                  " > 36 mm — risks fouling check-rails / point work");
        }
    }

    if (in.run_out_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": run_out_mm must be >= 0 (got " +
              std::to_string(in.run_out_mm) + ")");
    } else if (in.run_out_mm > 0.3) {
        r.add("DFM-WT-RUNOUT", "error",
              std::string(kSkillId) + ": run_out_mm " +
              std::to_string(in.run_out_mm) +
              " > 0.3 mm — exceeds allowable radial run-out");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "wheel_truing DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "wheel_dia_mm",     in.wheel_dia_mm },
        { "flange_height_mm", in.flange_height_mm },
        { "run_out_mm",       in.run_out_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "wheel_dia_mm",     in.wheel_dia_mm },
        { "flange_height_mm", in.flange_height_mm },
        { "run_out_mm",       in.run_out_mm },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "under_floor_wheel_lathe";
    tooling.tool_dia_mm       = in.wheel_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "carbide_insert";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 220.0;     // typical wheel-tread lathe SFM
    tooling.feed_per_tooth_mm = 0.4;
    tooling.stock_removed_mm3 = 0.0;       // modelled as metadata-only
    // ~ 25 min per wheelset.
    tooling.est_cycle_time_s  = 1500.0;
    tooling.extra = {
        { "process",          "wheel_truing" },
        { "wheel_dia_mm",     in.wheel_dia_mm },
        { "flange_height_mm", in.flange_height_mm },
        { "run_out_mm",       in.run_out_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::wheel_truing applied: dia={} flange={} runout={}",
                  in.wheel_dia_mm, in.flange_height_mm, in.run_out_mm);

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

}  // namespace koocadcam::skill::wheel_truing
