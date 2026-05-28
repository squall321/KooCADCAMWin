// @lat: [[engine/skills#hard_turning]]

#include "hard_turning.hpp"

#include "Workpiece.hpp"
#include "turn_external.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::hard_turning {

using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    // Mirror turn_external input gates first.
    if (in.final_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "hard_turning: final_dia_mm must be > 0");
    }
    if (in.end_z_mm <= in.start_z_mm) {
        r.add("DFM-INPUT", "error",
              "hard_turning: end_z_mm (" + std::to_string(in.end_z_mm) +
              ") must be > start_z_mm (" + std::to_string(in.start_z_mm) + ")");
    }
    if (in.roughing_passes < 1) {
        r.add("DFM-INPUT", "error",
              "hard_turning: roughing_passes must be ≥ 1");
    }

    // Hard-turning regime gate: HRC > 50, < 65 (CBN practical envelope).
    if (in.workpiece_hrc < 50.0) {
        r.add("DFM-HARDTURN-HRC", "error",
              "hard_turning: workpiece_hrc " + std::to_string(in.workpiece_hrc) +
              " < 50 — use turn_external (conventional carbide) instead");
    }
    if (in.workpiece_hrc > 65.0) {
        r.add("DFM-HARDTURN-HRC", "error",
              "hard_turning: workpiece_hrc " + std::to_string(in.workpiece_hrc) +
              " > 65 — above CBN practical limit; use grinding");
    }

    // Feed sanity: CBN inserts chip above ~0.1 mm/rev for HRC > 55.
    if (in.feed_override_mm > 0.1) {
        r.add("DFM-HARDTURN-FEED", "warning",
              "hard_turning: feed_override_mm " + std::to_string(in.feed_override_mm) +
              " > 0.1 mm/rev — CBN insert chip risk at HRC " +
              std::to_string(in.workpiece_hrc));
    }
    if (in.feed_override_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "hard_turning: feed_override_mm must be ≥ 0 (0 = use default)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Geometry is identical to turn_external: we delegate the B-rep work and
// then re-stamp the signature so the hard-turning strategy + CBN-tooling
// metadata replaces the conventional metadata.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "hard_turning DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Delegate to turn_external for the geometric Boolean.
    turn_external::Input te;
    te.start_z_mm      = in.start_z_mm;
    te.end_z_mm        = in.end_z_mm;
    te.final_dia_mm    = in.final_dia_mm;
    te.roughing_passes = in.roughing_passes;

    SkillOutput baseOut = turn_external::apply(wp, te);

    const double effectiveFeed = (in.feed_override_mm > 0.0)
                               ? in.feed_override_mm
                               : 0.05;     // CBN default 0.05 mm/rev

    // ── Re-stamp signature for the hard-turning strategy ─────────────────
    FeatureSignature sig = baseOut.signature;
    sig.skill_id = kSkillId;

    sig.params = {
        { "start_z_mm",       in.start_z_mm },
        { "end_z_mm",         in.end_z_mm },
        { "final_dia_mm",     in.final_dia_mm },
        { "roughing_passes",  in.roughing_passes },
        { "workpiece_hrc",    in.workpiece_hrc },
        { "feed_override_mm", in.feed_override_mm },
    };
    sig.pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "annular_planar_face_count",  2 },
        { "axis",                       { 0.0, 0.0, 1.0 } },
        { "final_dia_mm",               in.final_dia_mm },
        { "start_z_mm",                 in.start_z_mm },
        { "end_z_mm",                   in.end_z_mm },
        { "strategy",                   "hard_turning" },
        { "workpiece_hrc",              in.workpiece_hrc },
    };

    ToolingMeta tooling           = sig.tooling;
    tooling.tool_type             = "lathe_cbn_insert";
    tooling.tool_dia_mm           = 0.0;       // single-point insert
    tooling.tool_length_mm        = (in.end_z_mm - in.start_z_mm) + 5.0;
    tooling.tool_material         = "cbn";     // cubic boron nitride
    tooling.flute_count           = 1;
    tooling.cutting_speed_sfm     = 180.0;     // much lower than carbide turn
    tooling.feed_per_tooth_mm     = effectiveFeed;
    // Cycle time grows ~ 4× vs conventional carbide turn (low SFM + low feed).
    tooling.est_cycle_time_s      = std::max(1.0,
        in.roughing_passes * (in.end_z_mm - in.start_z_mm) / 15.0);
    tooling.extra["strategy"]     = "hard_turning";
    tooling.extra["workpiece_hrc"]= in.workpiece_hrc;
    tooling.extra["feed_mm_rev"]  = effectiveFeed;
    sig.tooling = tooling;

    auto wpNew = baseOut.workpiece;
    wpNew->addFeature(sig);

    spdlog::debug("skill::hard_turning applied: HRC={} R→{} feed={}",
                  in.workpiece_hrc, in.final_dia_mm / 2.0, effectiveFeed);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Topology is identical to turn_external — we can't differentiate by
// geometry.  When the workpiece carries a previously-stamped hard_turning
// signature we surface it with high confidence (0.92).  No geometric
// fallback (turn_external owns the cylindrical-turn topology).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.92;
        rf.matched_geometry = { { "source", "feature_history" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::hard_turning
