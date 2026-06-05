#pragma once
// @lat: [[engine/skills#square_case_octagonal_compound]]
//
// square_case_octagonal_compound — VERY HIGH-ORDER COMPOUND ASSEMBLY.
//
// Square octagonal sports watch case (Royal Oak / Nautilus style).  Starts
// from a square block, octagonally chamfers the 4 vertical corners, fuses
// integrated lug bars at 12/3/6/9 o'clock positions, drills 8 screwdown
// bolt holes around the bezel.  Composes pr::cut + pr::fuse + integrated
// lug skill + caseback.
//
// Sub-skill composition chain:
//   1. stock cuboid
//   2. octagonal trim — pr::cutMany of 4 corner triangular prisms
//   3. lug bar fuses at 12 and 6 (and optionally 3/9) via pr::fuseMany
//   4. 8 screwdown bolt-holes around the bezel (pr::cutMany)
//   5. case_back_screw_down (AS568 -016 flat face-seal O-ring)
//
// Output: octagonal square case.  pattern.is_compound = true,
// pattern.is_watch_assembly = true, pattern.watch_style = "square_octagonal",
// subfeature_count = 5.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace square_case_octagonal_compound {

constexpr const char* kSkillId = "square_case_octagonal_compound";

struct Input
{
    double      case_width_mm    = 42.0;   // 40 | 42 typical
    double      case_height_mm   = 11.0;
    int         lug_count        = 8;      // 8 integrated bolts (visible)
    int         screwdown_count  = 8;      // 8 screws around bezel
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace square_case_octagonal_compound
}  // namespace koocadcam::skill
