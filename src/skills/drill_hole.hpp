#pragma once
// @lat: [[engine/skills#drill_hole]]
//
// drill_hole — drill a blind or through cylindrical hole into the workpiece.
//
//                        ┌─ entry_face (planar)
//                        ▼
//                  ──────┬──────────┬──────
//                        │   ▼ axis │
//                        │  ╱│╲     │
//                        │ ╱ │ ╲    │
//                        │╱  │  ╲   │       ↑ depth_mm
//                        │   │   │  │       │
//                        │   │   │  │       │
//                        └───┴───┴──┘       ↓
//                            ↑
//                           dia_mm
//
// Signature pattern (what the skill leaves on the workpiece):
//   - 1 cylindrical face (the bore wall)
//   - 1 circular edge at the entry face (top circle, intersects entry plane)
//   - For blind: 1 planar face (the bottom) + 1 circular edge at the bottom
//   - For through: 1 circular edge at the exit face
//
// DFM checks:
//   DFM-002  : diameter_mm ≥ 0.8 mm
//   depth/dia ratio ≤ 8.0 (peck drilling required above this; flagged warn)
//
// Recognize:
//   Iterates all cylindrical faces; for each, finds 2 bounding circular
//   edges, infers diameter (= 2 × cylinder radius), depth (= distance
//   between circle centers along axis), entry face (adjacent planar face
//   sharing the top circle).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace drill_hole {

constexpr const char* kSkillId = "drill_hole";

struct Input
{
    FaceDatum   entry_face;             // which face to drill into
    double      position_x_mm = 0.0;    // hole center in world XY (drill axis from this point)
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };  // drill direction (into the workpiece)
    double      diameter_mm   = 0.0;
    double      depth_mm      = 0.0;
    bool        through_hole  = false;  // if true, depth_mm is ignored and hole goes all the way
};

// Synthesis: apply this skill to a workpiece.
// Returns a new workpiece (cloned with the hole cut) + the FeatureSignature.
SkillOutput apply(const Workpiece& wp, const Input& in);

// DFM validation.
DFMReport validate(const Workpiece& wp, const Input& in);

// Recognize candidates of this skill in the given workpiece.
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace drill_hole
}  // namespace koocadcam::skill
