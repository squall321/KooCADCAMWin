#pragma once
// @lat: [[engine/dfm-rules#Skill DFM]]
//
// Skill-level DFM types.  Each skill provides its own `validate(input)`
// that returns a DFMReport using these types, applying rules from the
// shared [[engine/dfm-rules]] catalog (DFM-001 … DFM-025).
//
// This is the same shape as koocadcam::engine::{DFMReport, DFMFinding}
// (in WatchFrontModel.hpp) and will be unified in a future cleanup.

#include <string>
#include <vector>

namespace koocadcam::skill {

struct DFMFinding
{
    std::string code;       // "DFM-002"
    std::string severity;   // "error" | "warning" | "info"
    std::string message;
};

struct DFMReport
{
    bool                       passed = true;   // false if any "error"
    std::vector<DFMFinding>    findings;

    void add(const std::string& code,
             const std::string& severity,
             const std::string& message)
    {
        findings.push_back({ code, severity, message });
        if (severity == "error") passed = false;
    }
};

}  // namespace koocadcam::skill
