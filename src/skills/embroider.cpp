// @lat: [[engine/skills#embroider]]

#include "embroider.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::embroider {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.design_id.empty()) {
        r.add("DFM-INPUT", "error",
              "embroider design_id must be non-empty");
    }
    if (in.stitches_count < 1) {
        r.add("DFM-INPUT", "error",
              "embroider stitches_count must be >= 1 (got " +
              std::to_string(in.stitches_count) + ")");
    }
    if (in.stitches_count > 1000000) {
        r.add("DFM-EMB-SIZE", "warning",
              "embroider stitches_count " + std::to_string(in.stitches_count) +
              " > 1,000,000 — cycle time excessive, consider applique");
    }
    if (in.thread_colors_count < 1) {
        r.add("DFM-INPUT", "error",
              "embroider thread_colors_count must be >= 1 (got " +
              std::to_string(in.thread_colors_count) + ")");
    }
    if (in.thread_colors_count > 15) {
        r.add("DFM-EMB-COLORS", "warning",
              "embroider thread_colors_count " +
              std::to_string(in.thread_colors_count) +
              " > 15 — exceeds typical multi-head machine capacity");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "embroider DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    json params = {
        { "design_id",           in.design_id },
        { "stitches_count",      in.stitches_count },
        { "thread_colors_count", in.thread_colors_count },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "design_id",           in.design_id },
        { "stitches_count",      in.stitches_count },
        { "thread_colors_count", in.thread_colors_count },
        { "geometry_changed",    false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "multi_head_embroidery_machine";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "polyester_thread";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // ~800 stitches/min on industrial machines + 10 s per color change.
    tooling.est_cycle_time_s  =
        (in.stitches_count / 800.0) * 60.0 +
        10.0 * in.thread_colors_count;
    tooling.extra = {
        { "process",             "apparel_embroider" },
        { "design_id",           in.design_id },
        { "stitches_count",      in.stitches_count },
        { "thread_colors_count", in.thread_colors_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::embroider applied: design={} stitches={} colors={}",
                  in.design_id, in.stitches_count, in.thread_colors_count);

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

}  // namespace koocadcam::skill::embroider
