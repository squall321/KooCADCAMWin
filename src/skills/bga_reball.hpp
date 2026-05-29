#pragma once
// @lat: [[engine/skills#bga_reball]]
//
// bga_reball — strip the old solder spheres from a BGA package and reflow
// fresh balls onto the underside pads.  Each ball is a sub-OCCT spherical
// solid that we DO NOT fuse to the package — recognition therefore stays
// metadata-only.  This skill stamps ball_count, alloy, and ball_dia_mm so
// downstream SMT placement / reflow can pick up the correct stencil profile.
//
//       ┌────────────────────────────┐
//       │       BGA package           │
//       └─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┘
//         ● ● ● ● ● ● ● ● ● ● ● ● ● ●     ← reballed solder spheres (ball_dia_mm)
//
// Signature pattern:
//   - pattern.kind            = "bga_reball"
//   - pattern.ball_dia_mm     = sphere Ø
//   - pattern.ball_count      = total spheres on the array
//   - pattern.ball_alloy      = "SAC305" | "Sn63Pb37" | "SAC405" | ...
//   - pattern.geometry_changed = false
//
// DFM checks:
//   ball_dia_mm  ∈ [0.20, 1.50]      (DFM-BALL-DIA error if outside)
//   ball_count   > 0 and ≤ 2500      (DFM-BALL-COUNT error if outside)
//   ball_alloy   non-empty            (DFM-BALL-ALLOY info)
//
// Synthesis: NO geometric change — clone shape + material, stamp signature.
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bga_reball {

constexpr const char* kSkillId = "bga_reball";

struct Input
{
    double      ball_dia_mm = 0.50;
    int         ball_count  = 256;
    std::string ball_alloy  = "SAC305";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bga_reball
}  // namespace koocadcam::skill
