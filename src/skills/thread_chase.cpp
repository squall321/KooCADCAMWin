// @lat: [[engine/skills#thread_chase]]
//
// thread_chase — geometric NO-OP repair pass over an existing thread.
// The skill exists solely to record CAPP/CAM intent; the workpiece shape
// passes through unchanged.

#include "thread_chase.hpp"

#include "Workpiece.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace koocadcam::skill::thread_chase {

using nlohmann::json;

namespace {

// Find the most recent thread feature in the workpiece history.  Returns
// the index of that signature in `wp.features()`, or -1 if none exists.
int findParentThreadIndex(const Workpiece& wp)
{
    const auto& feats = wp.features();
    for (int i = static_cast<int>(feats.size()) - 1; i >= 0; --i) {
        const std::string& sid = feats[i].skill_id;
        if (sid == "tap_thread"           ||
            sid == "rigid_tap"            ||
            sid == "form_tap"             ||
            sid == "thread_mill"          ||
            sid == "thread_mill_external" ||
            sid == "thread_grind")
        {
            return i;
        }
    }
    return -1;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    // DFM-CHASE-PASSES — must be in [1, 5].
    if (in.chase_depth_passes < 1 || in.chase_depth_passes > 5) {
        r.add("DFM-CHASE-PASSES", "error",
              "chase_depth_passes " + std::to_string(in.chase_depth_passes) +
              " outside valid range [1, 5] — more than 5 passes means the "
              "thread is too damaged to chase, re-tap or re-mill instead");
    }

    // DFM-CHASE-PARENT — must follow a prior thread feature.
    if (findParentThreadIndex(wp) < 0) {
        r.add("DFM-CHASE-PARENT", "error",
              "thread_chase requires a pre-existing thread feature "
              "(tap_thread / rigid_tap / form_tap / thread_mill / "
              "thread_mill_external / thread_grind) in workpiece.features()");
    }

    // Datum should resolve to a cylindrical face when provided.
    auto cylId = wp.resolve(in.existing_thread_datum);
    if (cylId) {
        BRepAdaptor_Surface surf(wp.face(*cylId));
        if (surf.GetType() != GeomAbs_Cylinder) {
            r.add("DFM-CHASE-DATUM", "warning",
                  "existing_thread_datum did not resolve to a cylindrical face");
        }
    } else {
        r.add("DFM-CHASE-DATUM", "warning",
              "existing_thread_datum unresolved — chasing path will be "
              "inferred from the parent thread signature at CAM-time");
    }

    return r;
}

// ── Synthesis (NO-OP geometric change) ───────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "thread_chase DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto cylId = wp.resolve(in.existing_thread_datum);
    const int cylIdOut = cylId.value_or(-1);

    // Snapshot the parent thread's geometric parameters (if resolvable) so
    // the chase signature carries enough metadata to drive CAM independently
    // of the parent feature.
    double cylDia = 0.0;
    if (cylId) {
        BRepAdaptor_Surface surf(wp.face(*cylId));
        if (surf.GetType() == GeomAbs_Cylinder) {
            cylDia = 2.0 * surf.Cylinder().Radius();
        }
    }

    const int parentIdx = findParentThreadIndex(wp);
    // Copy a few key fields from the parent so the chase signature is
    // self-describing (pitch, thread_size, length) — useful for downstream
    // process planning that may only see this signature without back-
    // reference resolution.
    std::string parentSkillId  = "";
    std::string parentThreadSize = "";
    double      parentPitch    = 0.0;
    double      parentLength   = 0.0;
    if (parentIdx >= 0) {
        const auto& parent = wp.features()[parentIdx];
        parentSkillId = parent.skill_id;
        const auto& p = parent.params;
        if (p.contains("thread_size") && p["thread_size"].is_string())
            parentThreadSize = p["thread_size"].get<std::string>();
        if (p.contains("pitch_mm") && p["pitch_mm"].is_number())
            parentPitch = p["pitch_mm"].get<double>();
        if (p.contains("thread_depth_mm") && p["thread_depth_mm"].is_number())
            parentLength = p["thread_depth_mm"].get<double>();
        else if (p.contains("thread_length_mm") && p["thread_length_mm"].is_number())
            parentLength = p["thread_length_mm"].get<double>();
    }

    json params = {
        { "thread_face_id",     cylIdOut },
        { "chase_depth_passes", in.chase_depth_passes },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "target_thread_id",  parentIdx },
        { "parent_skill",      parentSkillId },
        { "parent_thread_size", parentThreadSize },
        { "parent_pitch_mm",   parentPitch },
        { "parent_length_mm",  parentLength },
        { "cylinder_dia_mm",   cylDia },
        { "chase_passes",      in.chase_depth_passes },
        { "geometry",          "no_op_metadata_only" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "thread_chaser";
    tooling.tool_dia_mm       = (cylDia > 0.0) ? cylDia : 0.0;
    tooling.tool_length_mm    = std::max(20.0, parentLength * 2.0);
    tooling.tool_material     = "hss";
    tooling.flute_count       = 4;        // chaser dies typically have 4 chasing flutes
    tooling.cutting_speed_sfm = 20.0;     // very slow — clearing, not cutting
    tooling.feed_per_tooth_mm = (parentPitch > 0.0) ? (parentPitch / 4.0) : 0.0;
    tooling.stock_removed_mm3 = 0.0;      // intentionally zero (NO-OP)
    tooling.est_cycle_time_s  = std::max(2.0,
                                         in.chase_depth_passes * std::max(2.0, parentLength / 4.0));
    tooling.extra["operation"]    = "chase";
    tooling.extra["passes"]       = in.chase_depth_passes;
    tooling.extra["target_thread_index"] = parentIdx;
    tooling.extra["parent_skill"] = parentSkillId;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    // NO-OP: shape passes through unchanged.
    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::thread_chase applied: passes={} parent={} (NO-OP geometry)",
                  in.chase_depth_passes, parentSkillId);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Since thread_chase makes no geometric change, recognition is purely a
// replay of the metadata trail.  After STEP I/O the signature is lost and
// recognize() will return empty.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;   // perfect replay — we created this metadata
        rf.matched_geometry = { { "source", "metadata" }, { "geometric_change", false } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::thread_chase
