#pragma once
// @lat: [[engine/skills#Layer 5 LLM adapter]]
//
// _ProcessPlanStub.hpp — header-only stand-in for the real ProcessPlan layer
// (in-progress under `src/process/`).
//
// The shape mirrors `koocadcam::process::ProcessPlan` and
// `koocadcam::process::StepInvocation`:
//
//   struct StepInvocation { skill_id, params, depends_on, note };
//   class  ProcessPlan    { append, replaceParams, steps, toJson, fromJson };
//
// Why a stub?  Layer 5 (this `adapt/` module) is a parallel work-stream with
// Layer 3 (`process/ProcessPlan.cpp`).  At the time of writing the real
// process/ subdir has source files but is NOT yet wired into the top-level
// CMakeLists.txt (no `add_subdirectory(src/process)`).  Linking against an
// un-built library would break the build, so this stub gives `adapt/` an
// independent compile path.  When `koo_process` is wired in, the stub can be
// swapped for `#include "process/ProcessPlan.hpp"` and the using-aliases
// below will pick it up transparently.
//
// The stub's JSON shape MUST stay byte-compatible with the real one so a
// plan written by `koo_process` can be re-read by `koo_adapt` and vice versa.

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace koocadcam::process {

struct StepInvocation
{
    std::string       skill_id;
    nlohmann::json    params      = nlohmann::json::object();
    std::vector<int>  depends_on;
    std::string       note;
};

class ProcessPlan
{
public:
    ProcessPlan() = default;

    // ── Mutation ──────────────────────────────────────────────────────────
    int append(const StepInvocation& step)
    {
        const int idx = static_cast<int>(m_steps.size());
        m_steps.push_back(step);
        return idx;
    }

    bool replaceParams(int index, const nlohmann::json& newParams)
    {
        if (index < 0 || index >= static_cast<int>(m_steps.size())) {
            return false;
        }
        m_steps[index].params = newParams;
        return true;
    }

    // ── Read access ──────────────────────────────────────────────────────
    const std::vector<StepInvocation>& steps() const { return m_steps; }
    int  size()  const { return static_cast<int>(m_steps.size()); }
    bool empty() const { return m_steps.empty(); }

    // ── Serialization ────────────────────────────────────────────────────
    //
    // Matches Layer-3 schema:
    //   { "version": 1, "steps": [ {skill_id, params, depends_on, note}, ... ] }

    nlohmann::json toJson() const
    {
        nlohmann::json out = {
            { "version", 1 },
            { "steps",   nlohmann::json::array() },
        };
        for (const auto& s : m_steps) {
            nlohmann::json step = {
                { "skill_id",   s.skill_id },
                { "params",     s.params },
                { "depends_on", s.depends_on },
                { "note",       s.note },
            };
            out["steps"].push_back(step);
        }
        return out;
    }

    static ProcessPlan fromJson(const nlohmann::json& j)
    {
        ProcessPlan plan;
        if (!j.contains("steps") || !j["steps"].is_array()) return plan;
        for (const auto& stepJ : j["steps"]) {
            StepInvocation step;
            if (stepJ.contains("skill_id") && stepJ["skill_id"].is_string()) {
                step.skill_id = stepJ["skill_id"].get<std::string>();
            }
            if (stepJ.contains("params")) {
                step.params = stepJ["params"];
            } else {
                step.params = nlohmann::json::object();
            }
            if (stepJ.contains("depends_on") && stepJ["depends_on"].is_array()) {
                for (const auto& d : stepJ["depends_on"]) {
                    if (d.is_number_integer()) {
                        step.depends_on.push_back(d.get<int>());
                    }
                }
            }
            if (stepJ.contains("note") && stepJ["note"].is_string()) {
                step.note = stepJ["note"].get<std::string>();
            }
            plan.m_steps.push_back(step);
        }
        return plan;
    }

private:
    std::vector<StepInvocation> m_steps;
};

}  // namespace koocadcam::process
