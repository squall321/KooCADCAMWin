#pragma once
// @lat: [[engine/skills#leaf_spring_anchor_compound]]
//
// leaf_spring_anchor_compound — COMPOUND FEATURE (4 primitive sub-cuts).
//
// Realistic leaf-spring anchor mounting on a metal chassis.  A
// rectangular slot receives the spring leaf, a small retention undercut
// captures the bottom face of the leaf (so the leaf can't be pulled
// straight up), and 2 retention-screw pilots hold an anchor strap from
// the top.
//
//   Sub-feature chain:
//     1. Rectangular slot       (length × width × depth box cut)
//     2. Retention undercut     (small overhang ledge — 1 mm horizontal
//                                shelf at the slot bottom on each long
//                                side; combined into one ring tool here)
//     3. Screw pilot 1          (pilot hole for ISO screw_M, offset from
//                                slot center along +X)
//     4. Screw pilot 2          (pilot hole for ISO screw_M, offset from
//                                slot center along -X)
//
//   All four cutters are compounded then a single Boolean cut() removes
//   them together.  The retention undercut is approximated as a thin
//   box flange that extends 1 mm wider than the slot at the slot floor —
//   a slice-1 acceptable approximation to a true T-slot undercut.
//
// Output topology: 4 slot walls + 1 slot bottom + 2 small undercut steps
// + 2 cylindrical pilot holes (each with a flat blind bottom).  Single
// Workpiece, single FeatureSignature with pattern.is_compound = true,
// subfeature_count = 4.
//
// DFM gates (SAE J788 leaf-spring mounting bracket pattern):
//   - slot length and width >= 8 mm (covered spring stock range)
//   - slot depth >= width / 2 (sufficient pin support)
//   - undercut overhang = 1 mm fixed (this is the engineering convention)
//   - screw_M in {M3, M4, M5, M6, M8} (ISO 273 coarse metric series)
//   - screw spacing = slot length × 0.5 (centered pair, 25% inset)
//
// Recognize: metadata replay (compound undercut + multi-pilot pattern
// can mimic many other feature combinations).  Confidence 1.0 from
// history.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace leaf_spring_anchor_compound {

constexpr const char* kSkillId = "leaf_spring_anchor_compound";

struct Input
{
    FaceDatum    face_id;
    double       position_x_mm  = 0.0;
    double       position_y_mm  = 0.0;
    gp_Dir       axis_dir       { 0.0, 0.0, -1.0 };
    double       slot_length_mm = 0.0;          // slot_dim.x
    double       slot_width_mm  = 0.0;          // slot_dim.y
    double       slot_depth_mm  = 0.0;          // slot_dim.z
    std::string  screw_M        = "M4";         // "M3"|"M4"|"M5"|"M6"|"M8"
};

// ISO 273 medium-series clearance hole pilot for screw size.
double pilotForScrew(const std::string& screw_M);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace leaf_spring_anchor_compound
}  // namespace koocadcam::skill
