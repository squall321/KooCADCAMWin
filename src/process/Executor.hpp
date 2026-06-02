#pragma once
// @lat: [[engine/skills#Layer 3 Executor]]
//
// Executor — runs a ProcessPlan against an initial Workpiece.
//
// Slice-1 semantics:
//   - Strictly sequential.  steps[i] sees the Workpiece produced by
//     steps[i-1]; the depends_on field is recorded but not enforced.
//   - Errors short-circuit.  On any thrown SkillError (or unknown skill_id,
//     or JSON-parse failure), execution stops and ExecutionResult.workpiece
//     points at the partial result *just before* the failing step (i.e.
//     the output of the last successful step, or the initial workpiece if
//     step 0 failed).
//   - ExecutionResult.signatures collects the FeatureSignature produced by
//     each successful step (in order).
//
// Skill dispatch:
//   Each slice-1 skill_id is registered in a static unordered_map keyed by
//   the skill_id string.  Each entry's value is a lambda that parses the
//   JSON params into the skill's Input struct and calls the skill's
//   apply().  See registerDefaultSkills() in Executor.cpp for the table.
//
// Adding a new skill:
//   1. Implement the JSON→Input parser (see parsers in Executor.cpp).
//   2. Add one line to registerDefaultSkills() linking the skill_id to a
//      lambda that does { Input in = parseFoo(j); return foo::apply(wp, in); }.

#include "ProcessPlan.hpp"
#include "skills/Skill.hpp"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace koocadcam::skill { class Workpiece; }

namespace koocadcam::process {

// ExecuteOptions — runtime knobs for Executor::execute().
//
// Default-constructed values preserve the original slice-1 semantics:
//   abort on the first SkillError, no skipping.
//
// `continue_on_error = true` enables robust replay mode (slice-10+):
//   - On SkillError / std::exception / null-output / unknown-skill_id,
//     the current workpiece is KEPT (the failed step is treated as a
//     no-op), the error is recorded in step_errors, skipped_step_count
//     is incremented, and execution proceeds to the next step.
//   - On full success, ExecutionResult.failedAtStep is still -1.
//   - ok() remains false if any step errored, even with skipping on,
//     so callers can opt into the looser contract explicitly.
struct ExecuteOptions
{
    bool continue_on_error = false;
};

struct ExecutionResult
{
    // The workpiece after the last successfully applied step.  If the very
    // first step failed, this is the initial workpiece passed in.
    std::shared_ptr<skill::Workpiece>     workpiece;

    // FeatureSignatures from successful steps (size == failedAtStep if any
    // step failed, else == plan.size()).
    std::vector<skill::FeatureSignature>  signatures;

    // Empty on full success.  On failure, contains one message per error
    // encountered (typically just one — execution short-circuits).
    std::vector<std::string>              errors;

    // -1 if all steps succeeded; otherwise the zero-based index of the
    // step at which execution stopped (abort mode) OR the index of the
    // LAST step that errored (continue_on_error mode).
    int                                   failedAtStep = -1;

    // ── continue_on_error diagnostics ────────────────────────────────
    // Number of steps that errored when continue_on_error=true (always 0
    // when continue_on_error=false; one error short-circuits in that mode).
    int                                              skipped_step_count = 0;

    // (step_index, error_message) for every step that errored under
    // continue_on_error.  Empty in abort mode.
    std::vector<std::pair<int, std::string>>         step_errors;

    bool ok() const { return errors.empty() && failedAtStep == -1; }
};

class Executor
{
public:
    // Run `plan` starting from `initial_workpiece` and return the result.
    //
    // initial_workpiece must be non-null.  If a skill_id is not in the
    // dispatch table, the step fails immediately and the partial result
    // (last good workpiece) is returned.
    //
    // 2-arg form: abort-on-error (legacy default).
    static ExecutionResult execute(const ProcessPlan& plan,
                                   std::shared_ptr<skill::Workpiece> initial_workpiece);

    // 3-arg form: caller controls error-handling policy via ExecuteOptions.
    static ExecutionResult execute(const ProcessPlan& plan,
                                   std::shared_ptr<skill::Workpiece> initial_workpiece,
                                   const ExecuteOptions& opts);

    // Dispatcher type: parses params JSON and applies the skill.
    using SkillFn = std::function<skill::SkillOutput(const skill::Workpiece&,
                                                     const nlohmann::json&)>;

    // Returns the (singleton) dispatch table, populated lazily.
    // Exposed for tests / introspection.
    static const std::unordered_map<std::string, SkillFn>& dispatchTable();
};

}  // namespace koocadcam::process
