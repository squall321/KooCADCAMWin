#pragma once
// @lat: [[engine/skills#marble_polish]]
//
// marble_polish — progressive-grit polishing of a marble slab surface to
// the target gloss-unit reading (GU, per ISO 2813 / ASTM D523).  CAD
// geometry is conceptually unchanged at the workpiece B-rep level (the
// removal is sub-micron and not modeled topologically); we stamp the
// process metadata: which grit ladder was used, target gloss, total area.
//
// Signature pattern:
//   - pattern.kind                     = "marble_polish"
//   - pattern.surface_area_m2          = input
//   - pattern.polish_grit_progression  = json array of int grits
//   - pattern.final_gloss_units        = input
//   - pattern.pass_count               = derived from grit ladder
//   - pattern.geometry_changed         = false
//
// DFM checks:
//   DFM-INPUT                  surface_area_m2 ≤ 0 (error)
//   DFM-INPUT                  polish_grit_progression empty (error)
//   DFM-POLISH-GRIT-ORDER      grits not monotonically increasing (warning)
//   DFM-POLISH-FINAL-GRIT      final grit < 1500 but target > 80 GU (warning)
//   DFM-POLISH-GLOSS-RANGE     final_gloss_units outside [0, 100] (info)
//
// Synthesis:
//   NO geometric change at OCCT-resolution.  Clone and stamp.
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

namespace marble_polish {

constexpr const char* kSkillId = "marble_polish";

struct Input
{
    double surface_area_m2 = 1.0;
    std::vector<int> polish_grit_progression =
        { 60, 120, 220, 400, 800, 1500, 3000 };
    double final_gloss_units = 85.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace marble_polish
}  // namespace koocadcam::skill
