#pragma once
// @lat: [[engine/skills#Layer 5 LLM adapter]]
//
// PlanEditor — pure functional editor over ProcessPlan.
//
// All operations are non-destructive: `apply()` and `applyAll()` return a
// new plan; the source plan is never modified.  This makes Layer 5 safe to
// use as a "preview" mechanism (LLM proposes edits → user reviews resulting
// plan + DFM report → commits or rejects).
//
// Operations
// ──────────
//   apply        — apply a single EditOp.  Throws SkillError on invalid
//                  index or malformed payload.
//   applyAll     — apply a vector of EditOps in order.  Stops + rethrows
//                  on the first failure (the caller still has the source
//                  plan unmodified since edits are functional).
//   validatePlan — re-run each step's DFM `validate()` against a
//                  hypothetical initial workpiece.  Aggregates findings
//                  across steps, prefixing each finding's message with
//                  "[step N]" so callers can locate the offending step.
//                  In slice-1 the workpiece does NOT advance between steps
//                  (each step is validated against the initial stock); a
//                  full chained validate is future work.
//   diff         — human-readable list of differences between two plans.
//                  One line per difference; e.g.,
//                    "step 1: skill changed drill_hole -> mill_pocket"
//
// Error contract
// ──────────────
// Invalid edits throw `skill::SkillError`.  This is the same exception type
// used throughout the skills layer so callers can use a single catch site.

#include "EditOp.hpp"

#include "skills/DFM.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {
class Workpiece;
}  // namespace koocadcam::skill

namespace koocadcam::adapt {

class PlanEditor
{
public:
    // Apply one edit op.  Returns a new plan; `source` is unchanged.
    // Throws `skill::SkillError` on invalid index or malformed payload.
    static process::ProcessPlan apply(const process::ProcessPlan& source,
                                       const EditOp&              op);

    // Apply a batch of ops in order, threading the intermediate plan
    // through.  Equivalent to a left-fold of `apply()` over `ops`.
    static process::ProcessPlan applyAll(const process::ProcessPlan&     source,
                                          const std::vector<EditOp>&      ops);

    // Validate the plan: re-run each step's DFM validate() against
    // `initialStock`.  Each finding's message is tagged with its step
    // index ("[step N]") so callers can locate problems.
    //
    // Skills not yet wired into the slice-1 dispatcher are reported as
    // DFM-DISPATCH findings with severity "info" (i.e. they don't fail
    // the plan); a future slice promotes these to "error".
    static skill::DFMReport validatePlan(
        const process::ProcessPlan&             plan,
        std::shared_ptr<skill::Workpiece>       initialStock);

    // Compute a list of human-readable differences between plans `a` and
    // `b`.  Each entry is one line.  Empty vector means the plans are
    // identical (under structural equality of skill_id + params + note).
    static std::vector<std::string> diff(const process::ProcessPlan& a,
                                          const process::ProcessPlan& b);

private:
    // Read the JSON params at a given step index.  Throws SkillError if
    // out of range.
    static nlohmann::json paramsAt(const process::ProcessPlan& plan, int index);

    // Convert a payload JSON (the shape used by EditOp) into a
    // StepInvocation.  Throws SkillError on malformed payload.
    static process::StepInvocation stepFromPayload(const nlohmann::json& payload);

    // Rebuild a plan from a vector of StepInvocations (preserves order).
    static process::ProcessPlan rebuild(const std::vector<process::StepInvocation>& steps);
};

}  // namespace koocadcam::adapt
