// @lat: [[engine/skills#sew]]

#include "sew.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace koocadcam::skill::sew {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.stitch_type.empty()) {
        r.add("DFM-INPUT", "error",
              "sew stitch_type must be non-empty");
    }
    if (in.stitches_per_in <= 0.0) {
        r.add("DFM-INPUT", "error",
              "sew stitches_per_in must be > 0 (got " +
              std::to_string(in.stitches_per_in) + ")");
    } else {
        if (in.stitches_per_in < 4.0) {
            r.add("DFM-SEW-DENSITY", "warning",
                  "sew stitches_per_in " + std::to_string(in.stitches_per_in) +
                  " < 4 — seam strength likely insufficient");
        }
        if (in.stitches_per_in > 30.0) {
            r.add("DFM-SEW-DENSITY", "warning",
                  "sew stitches_per_in " + std::to_string(in.stitches_per_in) +
                  " > 30 — fabric perforation risk");
        }
    }
    if (in.thread_color.empty()) {
        r.add("DFM-INPUT", "warning",
              "sew thread_color empty — defaulting to natural");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sew DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const std::string color =
        in.thread_color.empty() ? "natural" : in.thread_color;

    json params = {
        { "stitch_type",     in.stitch_type },
        { "stitches_per_in", in.stitches_per_in },
        { "thread_color",    color },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "stitch_type",      in.stitch_type },
        { "stitches_per_in",  in.stitches_per_in },
        { "thread_color",     color },
        { "geometry_changed", false },
    };

    ToolingMeta tooling;
    tooling.tool_type =
        (in.stitch_type == "overlock")     ? "overlock_serger" :
        (in.stitch_type == "chainstitch")  ? "chainstitch_machine" :
        (in.stitch_type == "zigzag")       ? "zigzag_machine" :
                                             "industrial_lockstitch";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "polyester_thread";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // Approx 1 in/s seam at machine speed; placeholder cycle 5 s for one seam.
    tooling.est_cycle_time_s  = 5.0;
    tooling.extra = {
        { "process",         "apparel_sew" },
        { "stitch_type",     in.stitch_type },
        { "stitches_per_in", in.stitches_per_in },
        { "thread_color",    color },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::sew applied: type={} spi={} color={}",
                  in.stitch_type, in.stitches_per_in, color);

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

}  // namespace koocadcam::skill::sew
