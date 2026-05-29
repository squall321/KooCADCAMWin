// @lat: [[engine/skills#wind_blade_layup]]

#include "wind_blade_layup.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <set>
#include <string>

namespace koocadcam::skill::wind_blade_layup {

using nlohmann::json;

namespace {

const std::set<std::string>& knownFibers()
{
    static const std::set<std::string> f{ "E-glass", "S-glass", "carbon" };
    return f;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.ply_count <= 0) {
        r.add("DFM-INPUT", "error",
              "wind_blade_layup ply_count must be > 0 (got " +
              std::to_string(in.ply_count) + ")");
    }
    if (in.length_m < 1.0 || in.length_m > 120.0) {
        r.add("DFM-BLADE-LEN", "error",
              "wind_blade_layup length_m " +
              std::to_string(in.length_m) +
              " outside production range [1.0, 120.0] m");
    }
    if (knownFibers().find(in.fiber_type) == knownFibers().end()) {
        r.add("DFM-BLADE-FIBER", "info",
              "wind_blade_layup fiber_type '" + in.fiber_type +
              "' not in {E-glass, S-glass, carbon}");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "wind_blade_layup DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Carbon dominated layup is much stiffer per ply — record a
    // diagnostic stiffness index for downstream planning.
    const double stiffness_index =
        (in.fiber_type == "carbon")  ? 3.0 :
        (in.fiber_type == "S-glass") ? 1.3 : 1.0;

    json params = {
        { "ply_count",  in.ply_count },
        { "fiber_type", in.fiber_type },
        { "length_m",   in.length_m },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "ply_count",        in.ply_count },
        { "fiber_type",       in.fiber_type },
        { "length_m",         in.length_m },
        { "stiffness_index",  stiffness_index },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "vartm_mold";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.length_m * 1000.0;     // mold span in mm
    tooling.tool_material     = "epoxy_resin";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~ 90 s per ply per metre of blade.
    tooling.est_cycle_time_s  = 90.0 *
        static_cast<double>(in.ply_count) * in.length_m;
    tooling.extra = {
        { "process",         "composite_layup" },
        { "ply_count",       in.ply_count },
        { "fiber_type",      in.fiber_type },
        { "length_m",        in.length_m },
        { "stiffness_index", stiffness_index },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::wind_blade_layup applied: plies={} fiber={} L={}m",
                  in.ply_count, in.fiber_type, in.length_m);

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

}  // namespace koocadcam::skill::wind_blade_layup
