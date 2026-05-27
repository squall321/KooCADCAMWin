#pragma once
// @lat: [[engine/skills#Layer 5 LLM adapter]]
//
// EditOp — a typed, serializable description of a single change to a
// ProcessPlan, expressed in a form that an external system (LLM agent, GUI
// editor, batch-script tool) can produce.
//
// Design intent
// ─────────────
// Layer 5 (this `adapt/` module) sits ABOVE the ProcessPlan layer.  Rather
// than letting callers mutate plans directly, all edits go through a
// well-typed `EditOp` so that:
//
//   - the change can be logged / audited / undone,
//   - the change can be re-applied to a different plan version,
//   - an LLM can propose edits without learning C++ APIs (just emit one of
//     the EditKind variants as JSON), and
//   - the resulting plan is automatically re-validated via DFM (see
//     `PlanEditor::validatePlan`).
//
// All edit operations are FUNCTIONAL: `PlanEditor::apply()` returns a new
// plan and leaves the source untouched.

// Layer-3 dependency.  We try the real Layer-3 header first; if the
// process/ subdir is not yet in the build graph (parallel work-stream),
// fall back to the byte-compatible stub shipped alongside this file.
#if __has_include("process/ProcessPlan.hpp")
#  include "process/ProcessPlan.hpp"
#else
#  include "_ProcessPlanStub.hpp"
#endif

#include <nlohmann/json.hpp>

#include <string>

namespace koocadcam::adapt {

// ── EditKind ─────────────────────────────────────────────────────────────
//
// The discriminator for the tagged union below.  Each variant uses a
// different subset of fields on `EditOp`; see the table next to each enum
// value for which fields are read.
enum class EditKind
{
    AppendStep,       // payload = StepInvocation JSON;                     reads: payload
    InsertStepAt,     // payload = StepInvocation JSON; index = where;      reads: index, payload
    RemoveStep,       // index = which step to drop;                        reads: index
    ReplaceParams,    // index = which step; payload = new params JSON;     reads: index, payload
    ReorderSteps,     // index = src position; destIndex = dst position;    reads: index, destIndex
    AnnotateStep,     // index = which step; payload = { "note": "..." };   reads: index, payload
};

// Helper: stringify an EditKind for logs and diff output.
inline const char* editKindName(EditKind k)
{
    switch (k) {
        case EditKind::AppendStep:    return "AppendStep";
        case EditKind::InsertStepAt:  return "InsertStepAt";
        case EditKind::RemoveStep:    return "RemoveStep";
        case EditKind::ReplaceParams: return "ReplaceParams";
        case EditKind::ReorderSteps:  return "ReorderSteps";
        case EditKind::AnnotateStep:  return "AnnotateStep";
    }
    return "Unknown";
}

// ── EditOp ───────────────────────────────────────────────────────────────
//
// Tagged-union representation of a single plan edit.  The `kind` field
// selects which of `index`, `destIndex`, `payload` are meaningful (see the
// EditKind table above).  Fields not used by a given kind should remain at
// their defaults (-1 / empty JSON).
//
// `payload` encodes step content in the same JSON shape as
// `ProcessPlan::toJson()["steps"][i]`, i.e.:
//   { "skill_id": "...", "params": {...}, "depends_on": [...], "note": "..." }
//
// For `AnnotateStep`, payload is reduced to `{ "note": "..." }`.
struct EditOp
{
    EditKind        kind        = EditKind::AppendStep;
    int             index       = -1;                       // target step index
    int             destIndex   = -1;                       // ReorderSteps move-to-here
    nlohmann::json  payload     = nlohmann::json::object(); // step JSON / new params / note

    // ── Convenience builders ─────────────────────────────────────────────
    //
    // These keep call-sites terse for tests and stub authors.  Each maps
    // 1:1 to one of the EditKind variants; runtime validation happens in
    // PlanEditor::apply().

    static EditOp appendStep(const process::StepInvocation& step)
    {
        EditOp op;
        op.kind    = EditKind::AppendStep;
        op.payload = stepToJson(step);
        return op;
    }

    static EditOp insertStepAt(int idx, const process::StepInvocation& step)
    {
        EditOp op;
        op.kind    = EditKind::InsertStepAt;
        op.index   = idx;
        op.payload = stepToJson(step);
        return op;
    }

    static EditOp removeStep(int idx)
    {
        EditOp op;
        op.kind  = EditKind::RemoveStep;
        op.index = idx;
        return op;
    }

    static EditOp replaceParams(int idx, const nlohmann::json& newParams)
    {
        EditOp op;
        op.kind    = EditKind::ReplaceParams;
        op.index   = idx;
        op.payload = newParams;
        return op;
    }

    static EditOp reorderSteps(int src, int dst)
    {
        EditOp op;
        op.kind      = EditKind::ReorderSteps;
        op.index     = src;
        op.destIndex = dst;
        return op;
    }

    static EditOp annotateStep(int idx, const std::string& note)
    {
        EditOp op;
        op.kind    = EditKind::AnnotateStep;
        op.index   = idx;
        op.payload = { { "note", note } };
        return op;
    }

private:
    // Encode a StepInvocation as the JSON shape expected in `payload`.
    static nlohmann::json stepToJson(const process::StepInvocation& s)
    {
        return {
            { "skill_id",   s.skill_id },
            { "params",     s.params },
            { "depends_on", s.depends_on },
            { "note",       s.note },
        };
    }
};

}  // namespace koocadcam::adapt
