// @lat: [[engine/skills#bga_reball]]

#include "bga_reball.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::bga_reball {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.ball_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "bga_reball ball_dia_mm must be > 0");
    } else {
        if (in.ball_dia_mm < 0.20) {
            r.add("DFM-BALL-DIA", "error",
                  "bga_reball ball_dia_mm " + std::to_string(in.ball_dia_mm) +
                  " < 0.20 mm — below micro-BGA reflow tolerance");
        }
        if (in.ball_dia_mm > 1.50) {
            r.add("DFM-BALL-DIA", "error",
                  "bga_reball ball_dia_mm " + std::to_string(in.ball_dia_mm) +
                  " > 1.50 mm — above standard BGA pitch capability");
        }
    }

    if (in.ball_count <= 0) {
        r.add("DFM-INPUT", "error",
              "bga_reball ball_count must be > 0");
    } else if (in.ball_count > 2500) {
        r.add("DFM-BALL-COUNT", "error",
              "bga_reball ball_count " + std::to_string(in.ball_count) +
              " > 2500 — exceeds reasonable single-package density");
    }

    if (in.ball_alloy.empty()) {
        r.add("DFM-BALL-ALLOY", "info",
              "bga_reball ball_alloy empty — defaulting to 'SAC305'");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bga_reball DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string alloy = in.ball_alloy.empty() ? "SAC305" : in.ball_alloy;
    // SAC305 ~ 217 °C; Sn63Pb37 ~ 183 °C; SAC405 ~ 217 °C.
    const double reflow_peak_c =
        (alloy == "Sn63Pb37") ? 215.0 :
        (alloy == "SAC405")   ? 245.0 :
                                240.0;

    json params = {
        { "ball_dia_mm", in.ball_dia_mm },
        { "ball_count",  in.ball_count },
        { "ball_alloy",  alloy },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "ball_dia_mm",      in.ball_dia_mm },
        { "ball_count",       in.ball_count },
        { "ball_alloy",       alloy },
        { "reflow_peak_c",    reflow_peak_c },
        { "stencil_apertures",in.ball_count },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "reball_reflow_oven";
    tooling.tool_dia_mm       = in.ball_dia_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_stencil";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Whole-package reflow profile dominates over per-ball time.
    tooling.est_cycle_time_s  = 300.0;
    tooling.extra = {
        { "process",       "bga_reball" },
        { "ball_dia_mm",   in.ball_dia_mm },
        { "ball_count",    in.ball_count },
        { "ball_alloy",    alloy },
        { "reflow_peak_c", reflow_peak_c },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::bga_reball applied: dia={} count={} alloy={}",
                  in.ball_dia_mm, in.ball_count, alloy);

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

}  // namespace koocadcam::skill::bga_reball
