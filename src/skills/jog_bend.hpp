#pragma once
// @lat: [[engine/skills#jog_bend]]
//
// jog_bend — Z-shaped jog (two opposing bends, very common in sheet-metal
// brackets and electronics enclosures).
//
//                            ┌────────────────  ← far flat
//                           /
//                          / jog_angle
//                         /
//          ──────────────/   jog_offset
//          ↑
//          near flat (z = z_top)
//
// Two adjacent press-brake bends produce a parallel offset between two
// portions of the same sheet without breaking it.  The far flat is shifted
// by `jog_offset_mm` perpendicular to the sheet plane.
//
// Synthesis:
//   1. Split the sheet at bend_axis (line in XY).
//   2. Translate the FAR half along +Z by jog_offset_mm.
//   3. Add two cylindrical bend wedges (inner_radius_mm) of opposite sign
//      at the transitions to model the formed jog.
//   4. Fuse near + jog ramp + far.
//
// DFM:
//   DFM-J-SHEET     : 0.5 ≤ thickness ≤ 6.0 mm
//   DFM-J-ANGLE     : 30 ≤ jog_angle_deg ≤ 60   (typical Z-bend range)
//   DFM-J-OFFSET    : jog_offset_mm > sheet_thickness
//   DFM-J-MIN-R     : inner_radius_mm ≥ 0.5 × thickness
//
// Recognize:
//   Two parallel +Z planar flats separated by `jog_offset` with TWO
//   horizontal-axis cylindrical bend faces between them.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace jog_bend {

constexpr const char* kSkillId = "jog_bend";

struct Input
{
    std::array<double, 2> bend_axis_start_xy { 0.0, 0.0 };
    std::array<double, 2> bend_axis_end_xy   { 0.0, 0.0 };
    double                jog_offset_mm      = 5.0;    // perpendicular offset
    double                jog_angle_deg      = 45.0;   // typical 30–60°
    double                inner_radius_mm    = 1.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace jog_bend
}  // namespace koocadcam::skill
