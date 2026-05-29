// @lat: [[engine/skills#sleeper_install]]

#include "sleeper_install.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::sleeper_install {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.sleeper_type != "concrete" &&
        in.sleeper_type != "wood" &&
        in.sleeper_type != "steel") {
        r.add("DFM-SL-TYPE", "info",
              std::string(kSkillId) + ": sleeper_type '" + in.sleeper_type +
              "' unrecognised — expected concrete/wood/steel");
    }

    if (in.spacing_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": spacing_mm must be > 0 (got " +
              std::to_string(in.spacing_mm) + ")");
    } else {
        if (in.spacing_mm < 450.0) {
            r.add("DFM-SL-SPACING", "error",
                  std::string(kSkillId) + ": spacing_mm " +
                  std::to_string(in.spacing_mm) +
                  " < 450 mm — sleepers too dense, tamping access blocked");
        }
        if (in.spacing_mm > 900.0) {
            r.add("DFM-SL-SPACING", "error",
                  std::string(kSkillId) + ": spacing_mm " +
                  std::to_string(in.spacing_mm) +
                  " > 900 mm — rail bending stress exceeds allowable");
        }
    }

    if (in.count < 1) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": count must be >= 1 (got " +
              std::to_string(in.count) + ")");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sleeper_install DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double trackLengthMm =
        in.spacing_mm * static_cast<double>(in.count - 1);

    json params = {
        { "sleeper_type", in.sleeper_type },
        { "spacing_mm",   in.spacing_mm },
        { "count",        in.count },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "sleeper_type",     in.sleeper_type },
        { "spacing_mm",       in.spacing_mm },
        { "count",            in.count },
        { "track_length_mm",  trackLengthMm },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "sleeper_layer_crane";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.spacing_mm;
    tooling.tool_material     = in.sleeper_type;
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 30 s placement per sleeper with a mechanised gantry crew.
    tooling.est_cycle_time_s  = 30.0 * static_cast<double>(in.count);
    tooling.extra = {
        { "process",         "sleeper_install" },
        { "sleeper_type",    in.sleeper_type },
        { "spacing_mm",      in.spacing_mm },
        { "count",           in.count },
        { "track_length_mm", trackLengthMm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::sleeper_install applied: type={} spacing={} count={}",
                  in.sleeper_type, in.spacing_mm, in.count);

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

}  // namespace koocadcam::skill::sleeper_install
