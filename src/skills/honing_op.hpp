#pragma once
// @lat: [[engine/skills#honing_op]]
//
// honing_op — PRECISION INTERNAL BORE FINISH.  Even finer than internal
// grinding: typical removal 0.001–0.01 mm radial, Ra ≈ 0.1 µm.  Produces a
// characteristic cross-hatch surface pattern from the reciprocating motion
// of abrasive stones.  Standard for engine cylinder liners, hydraulic
// cylinders, precision bushings.
//
//      Bore:                                    After (radius + removal_mm
//                                                      with cross-hatch
//                                                      surface pattern):
//          ┌─┬─┐                                    ┌──┬───┬──┐
//          │ │ │                                    │  │╳╳╳│  │
//          │ │ │                                    │  │╳╳╳│  │
//          │ │ │              ─→                    │  │╳╳╳│  │
//          │ │ │                                    │  │╳╳╳│  │
//          └─┴─┘                                    └──┴───┴──┘
//          d=R                                      d=R+Δ
//                                                   (Ra 0.1, cross-hatch)
//
// The cross-hatch angle is a SURFACE-PATTERN attribute (not visible at our
// coarse modelling fidelity) recorded in the signature for downstream
// quality and CMM inspection.
//
// Synthesis:
//   Same as internal_grind / ream: a slightly larger cylinder coaxial with
//   the existing bore, cut from the workpiece.  Differs only in metadata.
//
// DFM checks:
//   removal_mm ∈ [0.0005, 0.02]
//   target bore diameter ≥ 3 mm  — hones are physically smaller than
//                                   grinding quills but still need a
//                                   minimum to clear the stone holder.
//
// Recognize:
//   Topologically identical to a wider drill_hole / ream / internal_grind.
//   Recognition is metadata-only; recognize() returns empty.

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace honing_op {

constexpr const char* kSkillId = "honing_op";

struct Input
{
    // EXISTING bore on the workpiece.
    FaceDatum  bore_datum;
    // Default 0.002 mm = 2 µm radial — typical finish hone.
    double     removal_mm = 0.002;
    // Surface pattern attribute (0° = circumferential, 90° = axial).
    // Standard cross-hatch angles are 30°, 45°, 60°.
    double     cross_hatch_angle_deg = 45.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace honing_op
}  // namespace koocadcam::skill
