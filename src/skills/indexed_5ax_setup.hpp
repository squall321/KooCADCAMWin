#pragma once
// @lat: [[engine/skills#indexed_5ax_setup]]
//
// indexed_5ax_setup — a SETUP skill that records a workpiece rotation, as
// produced by the rotary table (A or B axis) on a 5-axis machine.  Unlike
// the cutting skills, this skill does NOT remove material; it APPLIES a
// rigid-body rotation to the workpiece geometry so that subsequent skills
// see the new orientation, exactly as if the part had been re-fixtured.
//
//                        (before)                   (after rotation)
//
//                         ▲ +Z                          ▲ +Z
//                         │  ┌──┐                       │      ┌──┐
//                         │  │  │   rotate 45°          │     ╱│  │
//                         │  │  │  ─────────────►       │    ╱ │  │
//                         │  └──┘   around +Y           │  ┌╱  └──┘
//                         └────────▶ +X                 │  │
//                                                       └────────▶ +X
//
// This is the "3+2" setup primitive — between cutting passes, you index
// the rotary table to a new orientation, lock it (3+2 = the rotation is
// indexed, not simultaneous), and then run the next 3-axis pass.  In our
// model, we mutate the workpiece shape so the downstream skill's "+Z up"
// assumption still works.
//
// Input:
//   rotation_axis      — gp_Ax1 in world coordinates
//   rotation_angle_deg — degrees ∈ [-360, 360]
//
// Synthesis:
//   gp_Trsf trsf; trsf.SetRotation(rotation_axis, angleRad);
//   BRepBuilderAPI_Transform( workpieceShape, trsf, /*copy=*/true).Shape();
//
// DFM checks:
//   angle ∈ [-360, 360]
//   DFM-005 info — "indexed_5ax_setup requires a 4th/5th rotary axis"
//
// Recognize:
//   Difficult — given an arbitrary workpiece we cannot tell if it has been
//   rotated unless we have a reference orientation.  Heuristic: if the
//   workpiece bbox is NOT axis-aligned with stock-style natural axes
//   (largest-area planar face's normal != +Z), emit a low-confidence
//   indexed_5ax_setup candidate (~0.3) suggesting "this workpiece appears
//   to have been indexed".

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace indexed_5ax_setup {

constexpr const char* kSkillId = "indexed_5ax_setup";

struct Input
{
    gp_Ax1   rotation_axis      { gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0) };  // default = +Y
    double   rotation_angle_deg = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace indexed_5ax_setup
}  // namespace koocadcam::skill
