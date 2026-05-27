#pragma once
// @lat: [[engine/skills#Layer 3 ProcessPlan]]
//
// StepInvocation — a single entry in a ProcessPlan.
//
// A StepInvocation is a deferred, serializable description of one skill call:
//
//   skill_id    : the registered skill name (e.g. "drill_hole",
//                 "mill_rect_pocket").  Used by the Executor's dispatch table.
//   params      : skill-specific Input encoded as JSON.  The shape mirrors
//                 what each skill emits in its FeatureSignature.params so a
//                 plan can round-trip through the skill catalog.
//   depends_on  : indices of earlier steps whose resulting datums this step
//                 uses (e.g., a counterbore that targets the entry face
//                 created by a face_milling step).  Empty for stock-relative
//                 steps.  Slice-1 executor runs steps strictly in order and
//                 does not require depends_on to be populated; it is recorded
//                 so future schedulers / parallel executors can re-order.
//   note        : optional free-form text (planner / human / LLM annotation).
//
// All fields serialize to JSON; see ProcessPlan::toJson() for the format.
//
// JSON datum spec — entry-face encoding inside `params`
// ─────────────────────────────────────────────────────
// To make plans human-editable without learning the full Datum.hpp variant
// vocabulary, each skill's parser accepts one of three shorthand keys for
// the entry face:
//
//   "entry_face":     "top"                  → FaceByNormal{ gp_Dir(0,0,1) }
//   "entry_face":     "bottom"               → FaceByNormal{ gp_Dir(0,0,-1) }
//   "entry_face_id":  <int>                  → FaceIdRef{<int>}
//   (neither present)                        → FaceLargestPlanar{}
//
// Other inferred / explicit datum kinds are TODO; for slice-1 the executor
// only needs to cover the "drill into the top face of a cuboid" case.

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace koocadcam::process {

struct StepInvocation
{
    std::string       skill_id;          // "drill_hole", "mill_rect_pocket", ...
    nlohmann::json    params;            // skill-specific input as JSON
    std::vector<int>  depends_on;        // indices of earlier producing steps
    std::string       note;              // free-form comment (optional)
};

}  // namespace koocadcam::process
