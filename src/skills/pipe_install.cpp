// @lat: [[engine/skills#pipe_install]]

#include "pipe_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace koocadcam::skill::pipe_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pipe_dia_mm <= 0.0 || in.pipe_dia_mm > 600.0) {
        r.add("DFM-PIPE-DIA", "error",
              std::string(kSkillId) + ": pipe_dia_mm " +
              std::to_string(in.pipe_dia_mm) +
              " outside (0, 600] mm — non-standard process pipe size");
    }

    if (in.length_m <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": length_m must be > 0");
    }

    if (in.joint_count < 0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": joint_count must be >= 0");
    }

    if (in.length_m > 0.0 && in.joint_count > 0) {
        const double density = static_cast<double>(in.joint_count) / in.length_m;
        if (density > 2.0) {  // > 1 joint per 0.5 m
            r.add("DFM-PIPE-JOINTS", "info",
                  std::string(kSkillId) + ": joint density " +
                  std::to_string(density) +
                  " joints/m > 2 — fitting count looks high");
        }
    }

    static const std::vector<std::string> known =
        { "copper", "pvc", "steel", "stainless", "cpvc", "pex", "cast_iron" };
    bool found = false;
    for (const auto& m : known) if (m == in.pipe_material) { found = true; break; }
    if (!found && !in.pipe_material.empty()) {
        r.add("DFM-PIPE-MATERIAL", "info",
              std::string(kSkillId) + ": pipe_material '" +
              in.pipe_material + "' not in standard set");
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
        { "pipe_dia_mm",   in.pipe_dia_mm },
        { "pipe_material", in.pipe_material },
        { "length_m",      in.length_m },
        { "joint_count",   in.joint_count },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "pipe_dia_mm",       in.pipe_dia_mm },
        { "pipe_material",     in.pipe_material },
        { "length_m",          in.length_m },
        { "joint_count",       in.joint_count },
        { "joint_density",
          (in.length_m > 0.0) ? (in.joint_count / in.length_m) : 0.0 },
        { "geometry_changed",  false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "pipe_fitter_kit";
    tooling.tool_dia_mm       = in.pipe_dia_mm;
    tooling.tool_length_mm    = in.length_m * 1000.0;
    tooling.tool_material     = in.pipe_material;
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // installation, no removal
    // Industry rule of thumb: ~ 15 min per joint.
    tooling.est_cycle_time_s  = in.joint_count * 900.0;
    tooling.extra = {
        { "process",       "pipe_install" },
        { "pipe_dia_mm",   in.pipe_dia_mm },
        { "pipe_material", in.pipe_material },
        { "length_m",      in.length_m },
        { "joint_count",   in.joint_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::pipe_install applied: dia={} material={} length={} joints={}",
                  in.pipe_dia_mm, in.pipe_material, in.length_m, in.joint_count);

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

}  // namespace koocadcam::skill::pipe_install
