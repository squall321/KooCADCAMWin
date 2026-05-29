#pragma once
// @lat: [[engine/skills#rail_weld]]
//
// rail_weld — fusion of continuous welded rail (CWR) joints.  Two production
// methods dominate: (1) thermite (aluminothermic) field welds and (2)
// flash-butt welds (depot or mobile flash-butt unit).  KooCADCAM records the
// weld method, rail grade (per UIC / EN 13674), and the count of joints
// fused in one campaign.  The macroscopic B-rep is unchanged — the rail
// shape stays the same, only the metallurgical record is appended.
//
//      ──────────────╲│╱──────────────   ← weld_method (thermite/flash_butt)
//      ──────────────╱│╲──────────────     joint_count joints @ rail_grade
//
// Signature pattern:
//   - pattern.kind             = "rail_weld"
//   - pattern.weld_method      = "thermite" | "flash_butt"
//   - pattern.rail_grade       = "R260" | "R350HT" | "1100" | ...
//   - pattern.joint_count      = >= 1
//   - pattern.geometry_changed = false
//
// DFM checks:
//   weld_method ∈ {"thermite","flash_butt"}   (DFM-RW-METHOD error)
//   rail_grade  non-empty                     (DFM-INPUT error)
//   joint_count >= 1                          (DFM-INPUT error)
//   joint_count > 50                          (DFM-RW-CAMPAIGN info)
//
// Synthesis:
//   NO geometric change. Clone + stamp.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rail_weld {

constexpr const char* kSkillId = "rail_weld";

struct Input
{
    std::string  weld_method = "thermite";   // "thermite" | "flash_butt"
    std::string  rail_grade  = "R260";       // UIC / EN 13674 grade
    int          joint_count = 1;            // joints in this campaign
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rail_weld
}  // namespace koocadcam::skill
