#pragma once
// @lat: [[engine/skills#laser_weld]]
//
// laser_weld — high-energy laser-beam welding (LBW).  A focused laser
// drills a vapour "keyhole" into the joint, producing a very NARROW and
// DEEP fusion zone with minimal heat-affected zone.  Used for hermetic
// seals on battery cells, watch case backs, fuel injectors, and any
// application where geometric accuracy matters more than deposition rate.
//
//   Side view — keyhole signature:
//                          ┌─┐
//                          │█│   ← weld_width_mm (0.2 – 1.0 mm)
//   ───────────────────────│█│─────────────  ← entry_face
//                          │█│
//                          │█│   ↕ weld_depth_mm   (>> width;
//                          │█│      typical aspect ratio ≥ 3)
//                          │█│
//                          └─┘   (penetrates DEEP into the joint)
//
// Geometric model:
//   The keyhole's geometric signature on the FINISHED part is a thin bead
//   protruding by `weld_height_mm` (a small surface cap, ~ 0.1 weld_width)
//   PLUS metadata recording the much-larger `weld_depth_mm` that lies
//   below the surface as a sub-surface fusion zone (we do NOT model the
//   sub-surface zone with a Boolean — the material is locally re-melted
//   but topologically intact).  CAPP downstream uses weld_depth_mm for
//   inspection / heat-input scheduling.
//
// Signature pattern:
//   - 2 half-cylinder bead-end faces (narrow)
//   - 2 long planar walls
//   - 1 top planar face (very thin stadium)
//   - pattern.kind = "laser_weld"
//   - depth_to_width ≥ 3.0  ← the KEY differentiator
//
// DFM checks:
//   weld_width_mm  ∈ [0.1, 1.5]    (sub-mm keyhole regime)
//   weld_depth_mm  ≥ 3 × weld_width_mm   (otherwise it's a conduction weld)
//   |end - start| > weld_width_mm
//
// Recognize:
//   Geometric: narrow stadium-shape additive bead AND metadata flag
//   `depth_to_width >= 3` ⇒ classify as laser.  In practice, the metadata
//   replay path is sufficient.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace laser_weld {

constexpr const char* kSkillId = "laser_weld";

struct Input
{
    FaceDatum   entry_face;
    double      start_x_mm     = 0.0;
    double      start_y_mm     = 0.0;
    double      end_x_mm       = 0.0;
    double      end_y_mm       = 0.0;
    double      weld_width_mm  = 0.0;   // typ. 0.2 – 1.0 mm (narrow!)
    double      weld_depth_mm  = 0.0;   // keyhole depth into the joint (deep!)
    double      weld_height_mm = 0.1;   // visible cap above surface (small)
    std::string material       = "stainless";
    double      laser_power_w  = 2000.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace laser_weld
}  // namespace koocadcam::skill
