// @lat: [[engine/skills#magnetic_particle_weld]]

#include "magnetic_particle_weld.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::magnetic_particle_weld {

using nlohmann::json;

namespace {

bool isFerromagnetic(const std::string& m)
{
    // crude heuristic — covers carbon / low-alloy steels, ductile iron
    return m.find("steel") != std::string::npos ||
           m.find("iron")  != std::string::npos;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.weld_id.empty()) {
        r.add("DFM-INPUT", "error",
              "magnetic_particle_weld weld_id must be non-empty");
    }
    if (in.technique != "wet" && in.technique != "dry") {
        r.add("DFM-NDT-TECHNIQUE", "error",
              "magnetic_particle_weld technique '" + in.technique +
              "' not in {wet, dry}");
    }
    if (!isFerromagnetic(wp.material())) {
        r.add("DFM-NDT-MATERIAL", "info",
              "magnetic_particle_weld: workpiece material '" + wp.material() +
              "' likely non-ferromagnetic — use penetrant inspection instead");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "magnetic_particle_weld DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "weld_id",   in.weld_id },
        { "technique", in.technique },
        { "accept",    in.accept },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "weld_id",          in.weld_id },
        { "technique",        in.technique },
        { "accept",           in.accept },
        { "code_reference",   "AWS_D1" },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = (in.technique == "wet")
                                    ? "mt_wet_yoke" : "mt_dry_yoke";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "iron_particles";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 60.0;
    tooling.extra = {
        { "process",   "ndt_magnetic_particle" },
        { "weld_id",   in.weld_id },
        { "technique", in.technique },
        { "accept",    in.accept },
        { "code",      "AWS_D1" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::magnetic_particle_weld applied: weld={} tech={} accept={}",
                  in.weld_id, in.technique, in.accept);

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

}  // namespace koocadcam::skill::magnetic_particle_weld
