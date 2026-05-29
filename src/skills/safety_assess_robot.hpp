#pragma once
// @lat: [[engine/skills#safety_assess_robot]]
//
// safety_assess_robot — metadata-only record of an ISO 13849 safety
// assessment for a robot cell.  Captures the assigned Performance Level
// (PLd / PLe) and the operational risk class.  No geometric change to
// the workpiece.
//
// Signature pattern:
//   - pattern.kind             = "safety_assess_robot"
//   - pattern.safety_category  = "PLd" | "PLe"   (ISO 13849-1)
//   - pattern.risk_class       = "low" | "medium" | "high"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   safety_category ∈ {PLa,PLb,PLc,PLd,PLe}   (DFM-INPUT error otherwise)
//   safety_category ∈ {PLd, PLe}              (DFM-SAFETY-CAT error if lower —
//                                              human-collaborative cells require PLd+)
//   risk_class ∈ {low, medium, high}          (DFM-INPUT error otherwise)
//   risk_class == "high" && PLd               (DFM-SAFETY-MISMATCH info — prefer PLe)
//
// Synthesis:
//   NO geometric change.  Clone shape+material verbatim, copy prior
//   features, append assessment signature.
//
// Recognize:
//   Metadata-only — replay signatures from workpiece.features().

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace safety_assess_robot {

constexpr const char* kSkillId = "safety_assess_robot";

struct Input
{
    std::string safety_category = "PLd";   // ISO 13849-1: PLa..PLe
    std::string risk_class      = "medium"; // "low" | "medium" | "high"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace safety_assess_robot
}  // namespace koocadcam::skill
