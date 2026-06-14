#pragma once
// @lat: [[engine/reverse-route#compound-grammar]]
//
// Declarative compound-feature grammar (breakthrough plan B1.2, orthodox path).
//
// The flat foreign-CAD cap (Recognizer.cpp) keeps ~750 domain-compound
// recognizers below the inference threshold because their loose geometric
// fallbacks fire on anything; a measured attempt to un-cap them by generic
// "wraps an atom" subsumption was REFUTED (fpscan_test) — wrapping an atom is
// not being the feature.  The orthodox fix is SPECIFIC: recognize a compound
// only when a declared pattern of recognized PRECISE atoms is present in the
// right configuration.  Such a candidate is grounded in measured atoms AND
// specific enough not to false-fire, so it can carry a high confidence.
//
// This module evaluates those grammar rules over the precise atoms that
// re::analyze already recovered.  Each rule is one function; the registry
// grows as patterns are added.  Patterns so far (both grounded in >=3 equal,
// parallel-axis, REVOLUTION-COMPLETE drilled holes):
//   bolt_circle_pattern — centres lie on a common circle;
//   linear_hole_array   — centres are collinear and evenly pitched.

#include "skills/Skill.hpp"

#include <vector>

namespace koocadcam::re {

// Given the candidates re::analyze() produced (precise atoms + capped
// fallbacks), emit grounded COMPOUND candidates for every declared pattern
// that matches.  Pure function of the atoms — no Workpiece access — so it is
// trivially testable and order-independent.
std::vector<skill::RecognizedFeature>
recognizeCompounds(const std::vector<skill::RecognizedFeature>& candidates);

}  // namespace koocadcam::re
