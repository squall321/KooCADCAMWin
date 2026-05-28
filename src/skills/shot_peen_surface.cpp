// @lat: [[engine/skills#shot_peen_surface]]

#include "shot_peen_surface.hpp"

#include "Workpiece.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::shot_peen_surface {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.intensity_almen <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": intensity_almen must be > 0");
    } else if (in.intensity_almen < 0.002 || in.intensity_almen > 0.030) {
        r.add("DFM-PEEN-INTENSITY", "error",
              std::string(kSkillId) + ": intensity_almen " +
              std::to_string(in.intensity_almen) +
              " outside Almen-A typical window [0.002, 0.030] in");
    }

    if (in.coverage_pct <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": coverage_pct must be > 0");
    } else if (in.coverage_pct < 50.0) {
        r.add("DFM-PEEN-COVERAGE", "error",
              std::string(kSkillId) + ": coverage_pct " +
              std::to_string(in.coverage_pct) +
              " < 50 % — surface compressive layer not assured");
    } else if (in.coverage_pct > 400.0) {
        r.add("DFM-PEEN-COVERAGE", "error",
              std::string(kSkillId) + ": coverage_pct " +
              std::to_string(in.coverage_pct) +
              " > 400 % — excessive impact, surface delamination risk");
    }

    if (in.shot_size_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string(kSkillId) + ": shot_size_mm must be > 0");
    } else if (in.shot_size_mm < 0.1 || in.shot_size_mm > 3.0) {
        r.add("DFM-PEEN-SHOT-SIZE", "info",
              std::string(kSkillId) + ": shot_size_mm " +
              std::to_string(in.shot_size_mm) +
              " outside typical [0.1, 3.0] mm — verify nozzle / media spec");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "shot_peen_surface DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError(
        "shot_peen_surface: entry_face datum unresolved");

    GProp_GProps props;
    BRepGProp::SurfaceProperties(wp.face(*entryId), props);
    const double faceArea = props.Mass();

    json params = {
        { "entry_face_id",   *entryId },
        { "intensity_almen", in.intensity_almen },
        { "coverage_pct",    in.coverage_pct },
        { "shot_size_mm",    in.shot_size_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "target_face_id",       *entryId },
        { "intensity_almen",      in.intensity_almen },
        { "coverage_pct",         in.coverage_pct },
        { "shot_size_mm",         in.shot_size_mm },
        { "target_face_area_mm2", faceArea },
        { "geometry_changed",     false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "shot_peening_machine";
    tooling.tool_dia_mm       = in.shot_size_mm;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "steel_shot";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;  // peening cold-works, doesn't remove
    // ~ 1 minute per 100 mm² at 100 % coverage; scale by coverage_pct.
    tooling.est_cycle_time_s  = std::max(
        5.0, faceArea / 100.0 * 60.0 * (in.coverage_pct / 100.0));
    tooling.extra = {
        { "intensity_almen", in.intensity_almen },
        { "coverage_pct",    in.coverage_pct },
        { "shot_size_mm",    in.shot_size_mm },
        { "process",         "shot_peen" },
        { "effect",          "compressive residual stress, fatigue life ↑" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::shot_peen_surface applied: face={} almen={} cov={}% shot={}mm",
                  *entryId, in.intensity_almen, in.coverage_pct, in.shot_size_mm);

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

}  // namespace koocadcam::skill::shot_peen_surface
