#pragma once
// @lat: [[engine/skills#tablet_press]]
//
// tablet_press — compaction of pharmaceutical powder blend into solid-dose
// tablets via punch-and-die rotary tablet press.
//
//     blend in hopper                  tablet press                tablets out
//   ┌─────────────────┐         ┌─────────────────────┐         ┌─────────────┐
//   │   powder mass   │   →    │  fill die  →  punch  │   →    │  compact    │
//   │   (granulate)   │         │  (force, dwell)      │         │  tablet     │
//   └─────────────────┘         └─────────────────────┘         └─────────────┘
//   Geometry NOT modeled at OCCT B-Rep layer — sub-millimeter pharmaceutical
//   tablet shape is captured by metadata signature (mass, force, dwell).
//
// Signature pattern:
//   pattern.kind             = "tablet_press"
//   pattern.press_force_kn   = double  (compression force kN)
//   pattern.dwell_time_ms    = double  (time at peak force)
//   pattern.tablet_mass_mg   = double  (per-tablet mass)
//   pattern.geometric_change = false
//
// DFM:
//   DFM-INPUT                press_force_kn ≤ 0 (error)
//   DFM-INPUT                dwell_time_ms ≤ 0 (error)
//   DFM-INPUT                tablet_mass_mg ≤ 0 (error)
//   DFM-TP-FORCE-HIGH        press_force_kn > 50 (info, capping risk)
//   DFM-TP-FORCE-LOW         press_force_kn < 2 (info, friable)
//   DFM-TP-MASS-RANGE        tablet_mass_mg < 30 or > 2000 (info, atypical)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tablet_press {

constexpr const char* kSkillId = "tablet_press";

struct Input
{
    double  press_force_kn  = 10.0;   // kN compression force
    double  dwell_time_ms   = 50.0;   // dwell-at-force duration
    double  tablet_mass_mg  = 250.0;  // per-tablet mass
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace tablet_press
}  // namespace koocadcam::skill
