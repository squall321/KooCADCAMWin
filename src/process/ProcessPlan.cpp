// @lat: [[engine/skills#Layer 3 ProcessPlan]]

#include "ProcessPlan.hpp"

#include <spdlog/spdlog.h>

#include <fstream>

namespace koocadcam::process {

using nlohmann::json;

// ── Mutation ─────────────────────────────────────────────────────────────

int ProcessPlan::append(const StepInvocation& step)
{
    const int idx = static_cast<int>(m_steps.size());
    m_steps.push_back(step);
    return idx;
}

bool ProcessPlan::replaceParams(int index, const json& newParams)
{
    if (index < 0 || index >= static_cast<int>(m_steps.size())) {
        return false;
    }
    m_steps[index].params = newParams;
    return true;
}

// ── Serialization ────────────────────────────────────────────────────────

json ProcessPlan::toJson() const
{
    json out = {
        { "version", 1 },
        { "steps",   json::array() },
    };
    for (const auto& s : m_steps) {
        json step = {
            { "skill_id",   s.skill_id },
            { "params",     s.params },
            { "depends_on", s.depends_on },
            { "note",       s.note },
        };
        out["steps"].push_back(step);
    }
    return out;
}

ProcessPlan ProcessPlan::fromJson(const json& j)
{
    ProcessPlan plan;

    if (!j.contains("steps") || !j["steps"].is_array()) {
        // Treat malformed input as empty plan; caller can detect via size().
        spdlog::warn("ProcessPlan::fromJson: missing or non-array 'steps' field");
        return plan;
    }

    for (const auto& stepJ : j["steps"]) {
        StepInvocation step;
        if (stepJ.contains("skill_id") && stepJ["skill_id"].is_string()) {
            step.skill_id = stepJ["skill_id"].get<std::string>();
        }
        if (stepJ.contains("params")) {
            step.params = stepJ["params"];
        } else {
            step.params = json::object();
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

bool ProcessPlan::writeFile(const std::filesystem::path& path) const
{
    try {
        std::ofstream ofs(path);
        if (!ofs) {
            spdlog::error("ProcessPlan::writeFile: cannot open {}", path.string());
            return false;
        }
        ofs << toJson().dump(2);
        if (!ofs) {
            spdlog::error("ProcessPlan::writeFile: write error on {}", path.string());
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        spdlog::error("ProcessPlan::writeFile: exception on {}: {}",
                      path.string(), e.what());
        return false;
    }
}

std::optional<ProcessPlan> ProcessPlan::readFile(const std::filesystem::path& path)
{
    try {
        std::ifstream ifs(path);
        if (!ifs) {
            spdlog::error("ProcessPlan::readFile: cannot open {}", path.string());
            return std::nullopt;
        }
        json j = json::parse(ifs);
        return fromJson(j);
    } catch (const json::parse_error& e) {
        spdlog::error("ProcessPlan::readFile: JSON parse error in {}: {}",
                      path.string(), e.what());
        return std::nullopt;
    } catch (const std::exception& e) {
        spdlog::error("ProcessPlan::readFile: exception on {}: {}",
                      path.string(), e.what());
        return std::nullopt;
    }
}

}  // namespace koocadcam::process
