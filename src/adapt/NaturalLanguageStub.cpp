// @lat: [[engine/skills#Layer 5 LLM adapter]]

#include "NaturalLanguageStub.hpp"

#include "skills/Skill.hpp"   // SkillError

#include <algorithm>
#include <cctype>
#include <optional>
#include <regex>
#include <string>
#include <vector>

namespace koocadcam::adapt {

namespace sk = koocadcam::skill;
using nlohmann::json;

namespace {

// ── String utilities ─────────────────────────────────────────────────────

std::string toLower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

bool contains(const std::string& hay, const std::string& needle)
{
    return hay.find(needle) != std::string::npos;
}

// Find the most recent step (highest index) whose params object contains
// `key`.  Returns -1 if none.
int findLatestStepWithParam(const process::ProcessPlan& plan,
                            const std::string&          key)
{
    const auto& steps = plan.steps();
    for (int i = static_cast<int>(steps.size()) - 1; i >= 0; --i) {
        const auto& p = steps[i].params;
        if (p.is_object() && p.contains(key) && p[key].is_number()) {
            return i;
        }
    }
    return -1;
}

// ── Pattern handlers ─────────────────────────────────────────────────────

// "increase <key> by <amount>" / "<key> ... 늘려서 ..."
//
// Recognized English keys: diameter_mm, depth_mm, position_x_mm,
// position_y_mm.  Korean trigger words: 직경/depth/depth_mm/diameter.
// Returns std::nullopt if the instruction doesn't match this pattern.
std::optional<EditOp> tryIncreaseParam(const std::string&         lower,
                                        const std::string&         original,
                                        const process::ProcessPlan& plan)
{
    // Recognized "increase ... by ..." (English).
    static const std::regex kEng(
        R"(increase\s+([a-z_]+)\s+by\s+([\-+]?[0-9]*\.?[0-9]+))",
        std::regex::icase);

    std::smatch m;
    std::string key;
    double      delta = 0.0;

    if (std::regex_search(lower, m, kEng)) {
        key   = m[1].str();
        delta = std::stod(m[2].str());
    } else if (contains(original, "늘려")) {
        // Korean: "diameter_mm를 0.5 늘려서" — pull out the first known key
        // and the first signed number in the original (case-preserving)
        // string.
        static const std::vector<std::string> kKnown = {
            "diameter_mm", "depth_mm", "position_x_mm", "position_y_mm",
        };
        for (const auto& k : kKnown) {
            if (contains(original, k)) { key = k; break; }
        }
        if (key.empty()) return std::nullopt;

        static const std::regex kNum(R"(([\-+]?[0-9]*\.?[0-9]+))");
        std::smatch m2;
        if (!std::regex_search(original, m2, kNum)) return std::nullopt;
        delta = std::stod(m2[1].str());
    } else {
        return std::nullopt;
    }

    const int idx = findLatestStepWithParam(plan, key);
    if (idx < 0) {
        throw sk::SkillError(
            "not_supported_by_stub: no step in current plan has param '" + key + "'");
    }
    json newParams = plan.steps()[idx].params;
    const double oldVal = newParams[key].get<double>();
    newParams[key] = oldVal + delta;
    return EditOp::replaceParams(idx, newParams);
}

// "remove last step"
std::optional<EditOp> tryRemoveLast(const std::string&         lower,
                                     const process::ProcessPlan& plan)
{
    if (!contains(lower, "remove last step")) return std::nullopt;
    if (plan.empty()) {
        throw sk::SkillError("not_supported_by_stub: plan is empty");
    }
    return EditOp::removeStep(static_cast<int>(plan.steps().size()) - 1);
}

// "add drill_hole at (x, y) diameter D depth D2"
std::optional<EditOp> tryAddDrillHole(const std::string& lower,
                                       const std::string& /*original*/)
{
    if (!contains(lower, "add drill_hole") && !contains(lower, "add drill hole"))
        return std::nullopt;

    static const std::regex kPattern(
        R"(\(\s*([\-+]?[0-9]*\.?[0-9]+)\s*,\s*([\-+]?[0-9]*\.?[0-9]+)\s*\))"
        R"([^0-9]*diameter\s+([\-+]?[0-9]*\.?[0-9]+))"
        R"([^0-9]*depth\s+([\-+]?[0-9]*\.?[0-9]+))",
        std::regex::icase);
    std::smatch m;
    if (!std::regex_search(lower, m, kPattern)) {
        throw sk::SkillError(
            "not_supported_by_stub: 'add drill_hole' needs '(x, y) diameter D depth D2'");
    }
    process::StepInvocation step;
    step.skill_id = "drill_hole";
    step.params = {
        { "position_x_mm", std::stod(m[1].str()) },
        { "position_y_mm", std::stod(m[2].str()) },
        { "diameter_mm",   std::stod(m[3].str()) },
        { "depth_mm",      std::stod(m[4].str()) },
        { "through_hole",  false },
    };
    return EditOp::appendStep(step);
}

}  // namespace

// ── Public entry point ───────────────────────────────────────────────────

EditOp stubNaturalLanguageToEditOp(const std::string&         instruction,
                                    const process::ProcessPlan& currentPlan)
{
    const std::string lower = toLower(instruction);

    if (auto op = tryIncreaseParam(lower, instruction, currentPlan); op.has_value()) {
        return *op;
    }
    if (auto op = tryRemoveLast(lower, currentPlan); op.has_value()) {
        return *op;
    }
    if (auto op = tryAddDrillHole(lower, instruction); op.has_value()) {
        return *op;
    }
    throw sk::SkillError(
        "not_supported_by_stub: instruction '" + instruction +
        "' does not match any slice-1 NL pattern");
}

}  // namespace koocadcam::adapt
