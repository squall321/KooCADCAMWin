#pragma once
// @lat: [[engine/skills#taper_turn]]
//
// taper_turn — straight conical taper between two radii along an axial range.
//
//                ┌─────────────────┐  R_initial
//                │                 │
//                │   ┌───────┐     │
//        ────────┘   │       │\    └──────── ← z1 with radius r1
//                    │       │ \
//                    │       │  \            (cone frustum cut: r0 at z0,
//                    │       │   \            r1 at z1, both < R_initial)
//        ────────────┘       └────\─────── ← z0 with radius r0
//
// Synthesis: build a cone frustum with outer radii (R_envelope at z0 and
// R_envelope at z1) and inner cylinder representing the taper itself — but
// since the simplest correct cutter is just the *complement* of the cone, we
// instead build an annular cone-ring with `prim::annularConeRing` and cut it.
//
// Concretely: outer wall = straight cylinder of stock radius + overhang, inner
// wall = a CONE frustum from r0 (at z0) up to r1 (at z1).  Cutting that ring
// leaves the cone-frustum as the remaining shape over [z0, z1].
//
// Signature pattern:
//   - 1 conical face with apex along ±Z, half-angle from atan((r1-r0)/(z1-z0))
//   - 2 circular bounding edges at z0 (r=r0) and z1 (r=r1)
//
// DFM checks:
//   DFM-INPUT  : z1 > z0, both r > 0.4 mm
//   DFM-LATHE  : both r < stock outer (taper must remove material)
//   DFM-TAPER  : taper half-angle ≤ 30° (steeper angles need a different op)
//
// Recognize: find a conical face coaxial with Z whose half-angle is ≤ 30°,
//            verify it is bounded by 2 circular edges at distinct Z planes.

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace taper_turn {

constexpr const char* kSkillId = "taper_turn";

struct Input
{
    double  z0_mm = 0.0;     // lower Z bound
    double  r0_mm = 0.0;     // radius at z0
    double  z1_mm = 0.0;     // upper Z bound (> z0)
    double  r1_mm = 0.0;     // radius at z1
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace taper_turn
}  // namespace koocadcam::skill
