#pragma once
// @lat: [[engine/skills#flare_install]]
//
// flare_install — surface facility installation of a vertical flare stack
// for emergency relief and continuous-flare disposal of hydrocarbon gas.
// Captures stack height, peak design flow, and pilot/ignition type.  This
// is a facility-installation record; the workpiece model (flare riser or
// tip assembly) is NOT geometrically modified.
//
//                  🔥
//                  ▲
//                ──┴── tip @ stack_height_m
//                  │
//                  │  ← max_flow_nm3_h design
//                  │
//                  │  ← ignition_type
//             ─────┴─────
//             ░░░░ pad ░░░░
//
// Signature pattern:
//   - pattern.kind             = "flare_install"
//   - pattern.stack_height_m
//   - pattern.max_flow_nm3_h
//   - pattern.ignition_type    = "pilot" | "flame_front_generator" | "ballistic"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   stack_height_m > 0                          (DFM-INPUT error)
//   stack_height_m < 10                         (DFM-FI-HEIGHT info — short stack)
//   stack_height_m > 120                        (DFM-FI-HEIGHT info — tall stack class)
//   max_flow_nm3_h > 0                          (DFM-INPUT error)
//   ignition_type non-empty                     (DFM-INPUT warning)
//   ignition_type ∈ known set                   (DFM-FI-IGN info if unknown)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace flare_install {

constexpr const char* kSkillId = "flare_install";

struct Input
{
    double       stack_height_m    = 40.0;
    double       max_flow_nm3_h    = 50000.0;       // peak design flow Nm3/hr
    std::string  ignition_type     = "pilot";       // "pilot"|"flame_front_generator"|"ballistic"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace flare_install
}  // namespace koocadcam::skill
