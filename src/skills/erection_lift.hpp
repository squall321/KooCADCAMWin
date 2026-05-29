#pragma once
// @lat: [[engine/skills#erection_lift]]
//
// erection_lift — record of a single block-erection lift in the building
// dock or graving dock.  The pre-outfitted hull block is rigged with
// lifting lugs, hoisted by a goliath/jib crane, swung over the dock, and
// landed on the previously erected adjacent block.  Macroscopic B-rep is
// unchanged at the planner level; we capture the lift logistics.
//
//                  ▲
//                ╱   ╲                   ← crane hook
//              ╱       ╲
//          ┌──┴───────────┴──┐
//          │   hull block    │           ← rigged block at mass_t,
//          │   lift_id=LFT-7 │             hoisted lift_height_m
//          └─────────────────┘
//                  │
//          ════════╪════════              ← dock floor
//
// Signature pattern:
//   - pattern.kind              = "erection_lift"
//   - pattern.lift_id           = <input>
//   - pattern.mass_t            = <input>
//   - pattern.lift_height_m     = <input>
//   - pattern.crane_capacity_t  = <input>
//   - pattern.utilization       = mass_t / crane_capacity_t
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   lift_id            non-empty                      (DFM-INPUT warning)
//   mass_t             > 0                            (DFM-INPUT error)
//   lift_height_m      > 0                            (DFM-INPUT error)
//   crane_capacity_t   > 0                            (DFM-INPUT error)
//   mass_t             ≤ crane_capacity_t             (DFM-LIFT-CAPACITY error
//                                                      — overloaded crane)
//   mass_t / capacity  ≤ 0.85                         (DFM-LIFT-UTIL info if
//                                                      > 0.85 — within limit
//                                                      but at safety margin)
//
// Synthesis: clone shape + material, stamp signature.
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace erection_lift {

constexpr const char* kSkillId = "erection_lift";

struct Input
{
    std::string lift_id          = "LFT-001";
    double      mass_t           = 300.0;
    double      lift_height_m    = 25.0;
    double      crane_capacity_t = 800.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace erection_lift
}  // namespace koocadcam::skill
