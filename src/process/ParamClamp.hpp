#pragma once
// @lat: [[engine/skills#param-clamp]]
//
// Layer 3 — recovered-parameter clamp (breakthrough plan B3.4).
//
// When a reverse-engineered ProcessPlan is RE-EXECUTED on fresh stock, some
// recovered params over-specify and would crash a skill's apply() — e.g.
// hollow_cavity can recover a wall_thickness larger than the stock, which makes
// the remaining cavity non-positive and throws.  Rather than pre-filtering such
// steps out (the old per-test workaround), the Executor passes every step's
// params through this clamp first: a registry of pure functions that rewrite
// physically-impossible values into the nearest valid ones using the live stock
// extent.  A skill with no entry is a no-op, and a clamp never throws, so the
// layer is inert for every skill except those with an explicit rule.

#include <nlohmann/json.hpp>

#include <functional>
#include <string>
#include <unordered_map>

namespace koocadcam::skill { class Workpiece; }

namespace koocadcam::process {

// Rewrites `params` so the skill's apply() cannot crash on impossible values
// recovered from a foreign STEP.  Pure function of (params, current stock).
using ClampFn = std::function<nlohmann::json(const nlohmann::json& params,
                                             const skill::Workpiece& stock)>;

// Lazily-built singleton registry, keyed by skill_id (mirrors dispatchTable()).
const std::unordered_map<std::string, ClampFn>& clampTable();

// Look up skill_id; if a clamp exists return the clamped params, else return
// `params` unchanged.  Never throws (a failing clamp degrades to a no-op).
nlohmann::json clampParams(const std::string& skill_id,
                           const nlohmann::json& params,
                           const skill::Workpiece& stock);

}  // namespace koocadcam::process
