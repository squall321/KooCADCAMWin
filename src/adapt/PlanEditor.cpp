// @lat: [[engine/skills#Layer 5 LLM adapter]]

#include "PlanEditor.hpp"

#include "skills/Skill.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"

#include <algorithm>
#include <sstream>
#include <string>

namespace koocadcam::adapt {

using nlohmann::json;
namespace sk = koocadcam::skill;

// ── Internal helpers ─────────────────────────────────────────────────────

namespace {

// Read all steps out of a plan in order — easier than mutating the plan in
// place since ProcessPlan exposes only `append` / `replaceParams`.
std::vector<process::StepInvocation> snapshot(const process::ProcessPlan& plan)
{
    return plan.steps();   // copy
}

// Bounds-check a step index used by edit ops.  Throws on out-of-range.
void requireValidIndex(int index, int size, const char* opName)
{
    if (index < 0 || index >= size) {
        std::ostringstream m;
        m << opName << ": index " << index
          << " out of range [0, " << size << ")";
        throw sk::SkillError(m.str());
    }
}

// Bounds-check an INSERT index (one past the end is OK).
void requireValidInsertIndex(int index, int size, const char* opName)
{
    if (index < 0 || index > size) {
        std::ostringstream m;
        m << opName << ": insert index " << index
          << " out of range [0, " << size << "]";
        throw sk::SkillError(m.str());
    }
}

// ── Skill dispatch for validatePlan ──────────────────────────────────────
//
// Decode a step's params JSON into the right `Input` struct, then call the
// skill's `validate()`.  Returns a DFMReport.  Unknown skill_id → a single
// DFM-DISPATCH error finding.
//
// Slice-1 scope: only `drill_hole` is wired in (sufficient for the Layer-5
// prototype tests).  Adding more is mechanical — each new entry is a JSON
// → Input decode + a `::validate()` call.  Skills not yet wired are
// reported with DFM-DISPATCH "info" severity so the plan still validates
// (the rest of the plan is checkable; just this step is skipped).

sk::DFMReport validateOneStep(const process::StepInvocation& step,
                              const sk::Workpiece&            wp)
{
    if (step.skill_id == sk::drill_hole::kSkillId) {
        sk::drill_hole::Input in;
        const auto& p = step.params;
        if (p.contains("position_x_mm") && p["position_x_mm"].is_number())
            in.position_x_mm = p["position_x_mm"].get<double>();
        if (p.contains("position_y_mm") && p["position_y_mm"].is_number())
            in.position_y_mm = p["position_y_mm"].get<double>();
        if (p.contains("diameter_mm") && p["diameter_mm"].is_number())
            in.diameter_mm = p["diameter_mm"].get<double>();
        if (p.contains("depth_mm") && p["depth_mm"].is_number())
            in.depth_mm = p["depth_mm"].get<double>();
        if (p.contains("through_hole") && p["through_hole"].is_boolean())
            in.through_hole = p["through_hole"].get<bool>();
        // Note: axis_dir / entry_face decode is omitted — DFM checks for
        // drill_hole don't read those fields, and re-applying the cut is
        // outside slice-1 scope.  Default { 0, 0, -1 } / largest planar.
        return sk::drill_hole::validate(wp, in);
    }

    // Unknown skill_id → report as info-only (skipped); a downstream layer
    // can promote this to "error" once all 28 skills are wired in.
    sk::DFMReport r;
    r.add("DFM-DISPATCH", "info",
          "skill_id '" + step.skill_id + "' not validatable in slice-1 PlanEditor");
    return r;
}

// Tag every finding in `src` with the step index so the caller can locate
// which step produced which message.  Appends results into `dst`.
void copyWithStepTag(const sk::DFMReport& src, int stepIdx, sk::DFMReport& dst)
{
    for (const auto& f : src.findings) {
        const std::string tagged =
            "[step " + std::to_string(stepIdx) + "] " + f.message;
        dst.add(f.code, f.severity, tagged);
    }
}

}  // namespace

// ── PlanEditor public API ────────────────────────────────────────────────

process::ProcessPlan PlanEditor::apply(const process::ProcessPlan& source,
                                       const EditOp&              op)
{
    auto steps = snapshot(source);
    const int N = static_cast<int>(steps.size());

    switch (op.kind) {

    case EditKind::AppendStep: {
        steps.push_back(stepFromPayload(op.payload));
        return rebuild(steps);
    }

    case EditKind::InsertStepAt: {
        requireValidInsertIndex(op.index, N, editKindName(op.kind));
        steps.insert(steps.begin() + op.index, stepFromPayload(op.payload));
        return rebuild(steps);
    }

    case EditKind::RemoveStep: {
        requireValidIndex(op.index, N, editKindName(op.kind));
        steps.erase(steps.begin() + op.index);
        return rebuild(steps);
    }

    case EditKind::ReplaceParams: {
        requireValidIndex(op.index, N, editKindName(op.kind));
        if (!op.payload.is_object()) {
            throw sk::SkillError("ReplaceParams: payload must be a JSON object");
        }
        steps[op.index].params = op.payload;
        return rebuild(steps);
    }

    case EditKind::ReorderSteps: {
        requireValidIndex(op.index,     N, editKindName(op.kind));
        requireValidIndex(op.destIndex, N, editKindName(op.kind));
        if (op.index == op.destIndex) return rebuild(steps);
        auto moved = std::move(steps[op.index]);
        steps.erase(steps.begin() + op.index);
        // After erase, indices > op.index shifted down by 1.  Adjust dst.
        int dst = op.destIndex;
        if (dst > op.index) --dst;
        steps.insert(steps.begin() + dst, std::move(moved));
        return rebuild(steps);
    }

    case EditKind::AnnotateStep: {
        requireValidIndex(op.index, N, editKindName(op.kind));
        if (!op.payload.is_object() || !op.payload.contains("note") ||
            !op.payload["note"].is_string())
        {
            throw sk::SkillError("AnnotateStep: payload must be { \"note\": string }");
        }
        steps[op.index].note = op.payload["note"].get<std::string>();
        return rebuild(steps);
    }
    }
    // Defensive — should be unreachable; /WX requires a terminal return.
    throw sk::SkillError("PlanEditor::apply: unknown EditKind");
}

process::ProcessPlan PlanEditor::applyAll(const process::ProcessPlan& source,
                                           const std::vector<EditOp>&  ops)
{
    process::ProcessPlan current = source;
    for (const auto& op : ops) {
        current = apply(current, op);
    }
    return current;
}

sk::DFMReport PlanEditor::validatePlan(
    const process::ProcessPlan&        plan,
    std::shared_ptr<sk::Workpiece>     initialStock)
{
    sk::DFMReport aggregate;
    if (!initialStock) {
        aggregate.add("DFM-DISPATCH", "error",
                      "validatePlan: initialStock is null");
        return aggregate;
    }

    const auto& steps = plan.steps();
    for (size_t i = 0; i < steps.size(); ++i) {
        sk::DFMReport stepReport;
        try {
            stepReport = validateOneStep(steps[i], *initialStock);
        } catch (const std::exception& e) {
            stepReport.add("DFM-DISPATCH", "error",
                           std::string("validateOneStep threw: ") + e.what());
        }
        copyWithStepTag(stepReport, static_cast<int>(i), aggregate);
    }
    return aggregate;
}

std::vector<std::string> PlanEditor::diff(const process::ProcessPlan& a,
                                          const process::ProcessPlan& b)
{
    std::vector<std::string> out;
    const auto& sa = a.steps();
    const auto& sb = b.steps();
    const size_t common = std::min(sa.size(), sb.size());

    for (size_t i = 0; i < common; ++i) {
        if (sa[i].skill_id != sb[i].skill_id) {
            out.push_back("step " + std::to_string(i) +
                          ": skill changed " + sa[i].skill_id +
                          " -> " + sb[i].skill_id);
            // Skip further field-level diff for this step — the change is
            // already material enough that param/note diff is noise.
            continue;
        }
        if (sa[i].params != sb[i].params) {
            out.push_back("step " + std::to_string(i) +
                          ": params changed (" + sa[i].skill_id + ")");
        }
        if (sa[i].depends_on != sb[i].depends_on) {
            out.push_back("step " + std::to_string(i) +
                          ": depends_on changed");
        }
        if (sa[i].note != sb[i].note) {
            out.push_back("step " + std::to_string(i) +
                          ": note changed");
        }
    }
    for (size_t i = common; i < sb.size(); ++i) {
        out.push_back("step " + std::to_string(i) + ": added (" +
                      sb[i].skill_id + ")");
    }
    for (size_t i = common; i < sa.size(); ++i) {
        out.push_back("step " + std::to_string(i) + ": removed (" +
                      sa[i].skill_id + ")");
    }
    return out;
}

// ── Private helpers ──────────────────────────────────────────────────────

json PlanEditor::paramsAt(const process::ProcessPlan& plan, int index)
{
    const auto& steps = plan.steps();
    if (index < 0 || index >= static_cast<int>(steps.size())) {
        throw sk::SkillError("paramsAt: index " + std::to_string(index) +
                             " out of range");
    }
    return steps[index].params;
}

process::StepInvocation PlanEditor::stepFromPayload(const json& payload)
{
    if (!payload.is_object()) {
        throw sk::SkillError("stepFromPayload: payload must be a JSON object");
    }
    if (!payload.contains("skill_id") || !payload["skill_id"].is_string()) {
        throw sk::SkillError("stepFromPayload: missing or invalid 'skill_id'");
    }

    process::StepInvocation step;
    step.skill_id = payload["skill_id"].get<std::string>();

    if (payload.contains("params")) {
        if (!payload["params"].is_object()) {
            throw sk::SkillError("stepFromPayload: 'params' must be a JSON object");
        }
        step.params = payload["params"];
    } else {
        step.params = json::object();
    }

    if (payload.contains("depends_on") && payload["depends_on"].is_array()) {
        for (const auto& d : payload["depends_on"]) {
            if (d.is_number_integer()) {
                step.depends_on.push_back(d.get<int>());
            }
        }
    }
    if (payload.contains("note") && payload["note"].is_string()) {
        step.note = payload["note"].get<std::string>();
    }
    return step;
}

process::ProcessPlan PlanEditor::rebuild(
    const std::vector<process::StepInvocation>& steps)
{
    process::ProcessPlan out;
    for (const auto& s : steps) {
        out.append(s);
    }
    return out;
}

}  // namespace koocadcam::adapt
