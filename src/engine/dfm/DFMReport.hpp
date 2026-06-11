#pragma once
// @lat: [[engine/dfm-rules#DFMReport]]
//
// Shared DFM finding/report types for product-level validation (engine).
// (skill::DFMReport in src/skills/DFM.hpp is the per-skill counterpart.)

#include <string>
#include <utility>
#include <vector>

namespace koocadcam::engine {

struct DFMFinding
{
    std::string code;       // "DFM-002", "DFM-009", ...
    std::string severity;   // "error" | "warning" | "info"
    std::string message;
};

struct DFMReport
{
    bool                     passed = true;   // false if any "error" finding
    std::vector<DFMFinding>  findings;

    // Uniform accumulator (mirrors skill::DFMReport::add) — prerequisite
    // for the product-agnostic DFM rule registry (breakthrough plan B5).
    void add(std::string code, std::string severity, std::string message)
    {
        if (severity == "error") passed = false;
        findings.push_back(DFMFinding{ std::move(code), std::move(severity),
                                       std::move(message) });
    }
};

}  // namespace koocadcam::engine
