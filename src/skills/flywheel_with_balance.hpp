#pragma once
// @lat: [[engine/skills#flywheel_with_balance]]
//
// flywheel_with_balance — a disc + central hub bore + N evenly-spaced
// radial "balance drills" near the rim (used during dynamic balancing of
// rotating masses), plus an outer-rim chamfer for safe handling.
//
// Compound feature (≥ 2 cuts):
//   1. (input: workpiece — caller's stock; we FUSE a disc onto it for the
//      flywheel body — net contribution is "+ disc", "- bore", "- 6 drills",
//      "- rim chamfer cone")
//   2. CUT central hub bore (axial, on the disc axis)
//   3. CUT N balance-drill holes at angle θ_i around the rim at
//      radius `balance_pcd_mm / 2`, axial-direction, going partway through
//      thickness.  N is fixed to 6 (per problem spec).
//   4. CUT a small outer-rim chamfer using an annular ring cone-frustum
//      subtraction — produces a 45° chamfer on the outer edge.
//
// Reference standards:
//   ISO 1940-1 (mechanical vibration — balance quality of rotors)
//   AGMA 2015 (gear / flywheel structural minimums)
//
// DFM gate:
//   - outer_dia_mm ≥ 20 mm (smaller flywheels not productive to balance)
//   - thickness_mm ≥ 3 mm
//   - hub_bore_dia_mm ≥ 1 mm and ≤ 0.6 × outer_dia_mm
//   - balance_pcd_mm > hub_bore_dia + 2·balance_drill_dia AND
//     balance_pcd_mm < outer_dia - 2·balance_drill_dia  (must be in the disc body)
//   - balance_drill_dia_mm ≥ 0.8 mm (DFM-002)
//   - balance_drill_depth_mm ≤ 0.9 × thickness_mm (no through-holes during balancing)
//   - chamfer_mm ≥ 0 mm and ≤ thickness_mm / 2 (rim handling chamfer)
//
// Recognition:
//   PRIMARY (geometric): find the outer cylinder (axial Z, largest R),
//     a concentric inner cyl (hub bore), and ≥ 4 small axial cylinders at
//     equal radial distance from the centre (balance drills).  Count them
//     and use that as the recovered N.
//   FALLBACK (metadata): exact match if history present.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace flywheel_with_balance {

constexpr const char* kSkillId = "flywheel_with_balance";

struct Input
{
    double outer_dia_mm           = 80.0;
    double thickness_mm           = 10.0;
    double hub_bore_dia_mm        = 12.0;
    double balance_pcd_mm         = 60.0;  // pitch circle diameter for balance drills
    double balance_drill_dia_mm   = 3.0;
    double balance_drill_depth_mm = 5.0;
    int    balance_count          = 6;     // fixed per spec but configurable
    double chamfer_mm             = 0.5;   // 45° outer rim chamfer (axial × radial)
    double center_x_mm            = 0.0;
    double center_y_mm            = 0.0;
    double center_z_mm            = 0.0;   // bottom face Z; disc extends +Z by thickness_mm
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace flywheel_with_balance
}  // namespace koocadcam::skill
