#pragma once
// @lat: [[engine/skills#Layer 5 LLM adapter]]
//
// NaturalLanguageStub — placeholder for the LLM bridge.
//
// In slice-1 this is a deliberately tiny rule-based mapper: input strings
// are matched against a handful of patterns and converted to a typed EditOp
// for `PlanEditor::apply()`.  Real LLM integration is future work — at
// that point this function is replaced with a system-prompt + tool-call
// invocation, and the rule-based version becomes the offline/fallback path.
//
// Supported phrases (slice-1 stub):
//
//   "increase X by Y"       — X must be a known param name (diameter_mm,
//   "X 늘려서"                  depth_mm, position_x_mm, …).  Finds the
//                              MOST RECENT step containing X in its params,
//                              proposes ReplaceParams with X += Y.
//
//   "remove last step"      — proposes RemoveStep at index = steps.size()-1.
//
//   "add drill_hole at (x, y) diameter D depth D2"
//                           — proposes AppendStep with the parsed params.
//
// All other inputs throw `skill::SkillError` with message
// "not_supported_by_stub" so the caller can fall through to a real LLM
// once one is wired in.

#include "EditOp.hpp"

#include <string>

namespace koocadcam::adapt {

// Map a natural-language instruction to an EditOp against the given plan.
// Throws skill::SkillError("not_supported_by_stub") if the instruction is
// outside the supported patterns above, or if the plan does not contain a
// step matching the requested change (e.g., "remove last step" on empty).
EditOp stubNaturalLanguageToEditOp(const std::string&         instruction,
                                    const process::ProcessPlan& currentPlan);

}  // namespace koocadcam::adapt
