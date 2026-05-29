#pragma once
// @lat: [[engine/skills#pwht]]
//
// pwht — post-weld heat treatment.  Heavy-section welded shipyard joints
// (rudder horn, stern frame, thick deck inserts) are slowly heated to a
// hold temperature below the lower critical, soaked, and slow-cooled to
// relieve weld residual stress and temper the heat-affected zone.  As
// with anneal, the macroscopic B-rep is unchanged; the welded joint's
// METALLURGY changes.
//
//        weld bead w/ residual stress              relaxed weld bead
//          ┌──────────────┐    PWHT (T_hold, t_hold,
//          │              │     slow cool ≤ 55 °C/h)
//          │   joint      │  ──────────────────────▶   joint
//          │              │                              (σ_res ↓, HAZ tempered)
//          └──────────────┘
//
// Signature pattern:
//   - pattern.kind             = "pwht"
//   - pattern.weld_id          = <input>
//   - pattern.hold_temp_c      = <input>
//   - pattern.hold_time_h      = <input>
//   - pattern.residual_stress  = "low"
//   - pattern.geometry_changed = false
//
// DFM checks (ASME Section VIII / AWS D1.1 style):
//   weld_id       non-empty                        (DFM-INPUT warning)
//   hold_temp_c   > 0                              (DFM-INPUT error)
//   hold_temp_c   ∈ [550, 730] for C-Mn steel      (DFM-PWHT-TEMP info)
//   hold_time_h   > 0                              (DFM-INPUT error)
//   hold_time_h   ≥ 1 h per 25 mm thickness ≈ 1.0 (DFM-PWHT-TIME info — too
//                                                   short to soak through)
//
// Synthesis: clone shape + material, stamp signature.
// Recognize: metadata-only — relaxed residual stress is sub-OCCT.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pwht {

constexpr const char* kSkillId = "pwht";

struct Input
{
    std::string weld_id     = "WLD-001";
    double      hold_temp_c = 620.0;     // C-Mn steel typical 595–675 °C
    double      hold_time_h = 2.0;       // ≥ 1 h per 25 mm thickness
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pwht
}  // namespace koocadcam::skill
