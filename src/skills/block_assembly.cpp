// @lat: [[engine/skills#block_assembly]]

#include "block_assembly.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace koocadcam::skill::block_assembly {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.block_id.empty()) {
        r.add("DFM-INPUT", "warning",
              "block_assembly: block_id empty — defaulting to 'BLK-UNK'");
    }

    if (in.mass_t <= 0.0) {
        r.add("DFM-INPUT", "error",
              std::string("block_assembly: mass_t must be > 0 (got ") +
              std::to_string(in.mass_t) + ")");
    } else if (in.mass_t > 4000.0) {
        r.add("DFM-BLOCK-MASS", "info",
              std::string("block_assembly: mass_t ") +
              std::to_string(in.mass_t) +
              " > 4000 t — grand block, needs goliath crane for erection");
    }

    if (in.weld_meters < 0.0) {
        r.add("DFM-INPUT", "error",
              std::string("block_assembly: weld_meters must be >= 0 (got ") +
              std::to_string(in.weld_meters) + ")");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "block_assembly DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string block_id = in.block_id.empty() ? "BLK-UNK" : in.block_id;

    json params = {
        { "block_id",    block_id },
        { "mass_t",      in.mass_t },
        { "weld_meters", in.weld_meters },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "block_id",         block_id },
        { "mass_t",           in.mass_t },
        { "weld_meters",      in.weld_meters },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "block_assembly_platen";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "n/a";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Crude estimate: 1 m fillet weld ≈ 6 min total (set + weld + clean).
    tooling.est_cycle_time_s  = in.weld_meters * 360.0;
    tooling.extra = {
        { "process",     "block_assembly_fitup_weld" },
        { "block_id",    block_id },
        { "mass_t",      in.mass_t },
        { "weld_meters", in.weld_meters },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::block_assembly applied: id={} mass={}t weld={}m",
                  block_id, in.mass_t, in.weld_meters);

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

}  // namespace koocadcam::skill::block_assembly
