#pragma once
// @lat: [[engine/skills#Layer 3 ProcessPlan]]
//
// ProcessPlan — ordered sequence of StepInvocations.
//
// A ProcessPlan is the Layer-3 abstraction that sits ABOVE the skill
// library: it captures *what to do, in what order*, in a form that can be
// serialized, edited externally, re-executed deterministically, and
// transformed by an LLM (Layer 5).
//
//   - Read access:    steps()
//   - Edit access:    append(), replaceParams()
//   - Persistence:    toJson() / fromJson(); writeFile() / readFile()
//
// Slice-1 scope:
//   - Plan is a simple linear list.  No DAG scheduling — execution is in
//     append order (depends_on is recorded but not enforced).
//   - JSON schema is intentionally permissive: extra fields are passed
//     through verbatim so future tooling can attach annotations.

#include "StepInvocation.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace koocadcam::process {

class ProcessPlan
{
public:
    ProcessPlan() = default;

    // ── Mutation ──────────────────────────────────────────────────────────

    // Append a step.  Returns its zero-based index.
    int append(const StepInvocation& step);

    // Replace the JSON params of an existing step.  Returns false if the
    // index is out of range (the plan is left unchanged in that case).
    bool replaceParams(int index, const nlohmann::json& newParams);

    // ── Read access ──────────────────────────────────────────────────────

    const std::vector<StepInvocation>& steps() const { return m_steps; }
    int  size() const { return static_cast<int>(m_steps.size()); }
    bool empty() const { return m_steps.empty(); }

    // ── Serialization ────────────────────────────────────────────────────
    //
    // JSON shape:
    //   {
    //     "version": 1,
    //     "steps": [
    //       { "skill_id": "...", "params": {...}, "depends_on": [...],
    //         "note": "..." },
    //       ...
    //     ]
    //   }

    nlohmann::json     toJson()       const;
    static ProcessPlan fromJson(const nlohmann::json& j);

    bool                              writeFile(const std::filesystem::path& path) const;
    static std::optional<ProcessPlan> readFile(const std::filesystem::path& path);

private:
    std::vector<StepInvocation> m_steps;
};

}  // namespace koocadcam::process
