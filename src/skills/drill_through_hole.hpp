#pragma once
// @lat: [[engine/skills#drill_through_hole]]
//
// drill_through_hole — COMPOUND MACRO over `drill_hole`.
//
// This skill is the explicit through-hole sibling of `drill_hole`.  It does
// not introduce new geometry beyond what `drill_hole` produces when
// through_hole=true; rather it gives the planner a distinct SIGNATURE so
// that recognize() can flag pure through holes separately from blind holes
// at the catalog level.
//
//                        ┌─ entry_face (planar)
//                        ▼
//                  ──────┬──────────┬──────
//                        │   ▼ axis │
//                        │  ╱│╲     │
//                        │ ╱ │ ╲    │
//                        │╱  │  ╲   │       full workpiece thickness
//                        │   │   │  │
//                        │   │   │  │
//                  ──────┴───┴───┴──┘  (exits opposite face)
//                            ↑
//                           dia_mm
//
// Signature pattern (what the skill leaves on the workpiece):
//   - 1 cylindrical face (the bore wall)
//   - 1 circular edge at the entry face
//   - 1 circular edge at the exit face
//   - NO bottom planar face (it goes all the way through)
//   - signature.skill_id == "drill_through_hole"
//   - signature.pattern.kind == "drill_through_hole"
//
// Implementation: internally delegates to drill_hole::apply with
// through_hole=true.  The resulting FeatureSignature is then re-stamped
// (skill_id + pattern.kind) so the catalog identity is "drill_through_hole".
//
// DFM checks:
//   DFM-002       : diameter_mm ≥ 0.8 mm
//   DFM-THICKNESS : the workpiece bbox extent along axis_dir must be > 0
//                   (otherwise there is no material to drill through).
//
// Recognize:
//   Walk drill_hole::recognize() output and keep only candidates whose
//   `through_hole` flag is true.  Re-stamp them with this skill's id and a
//   slightly different confidence so the catalog can disambiguate pure
//   through-hole intent.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace drill_through_hole {

constexpr const char* kSkillId = "drill_through_hole";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    double      diameter_mm   = 0.0;
    // No depth_mm — depth is determined by the workpiece bbox extent along
    // axis_dir (the hole goes "all the way").
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace drill_through_hole
}  // namespace koocadcam::skill
