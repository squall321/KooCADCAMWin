#pragma once
// @lat: [[engine/skills#undercut_relief]]
//
// undercut_relief — small cylindrical pocket cut at a parting-line corner so
// a side-action (slide / lifter) can engage the moulded part.  Subtractive
// feature; axis is parallel to the parting plane (XY) and the relief is
// small relative to the workpiece.
//
//                     ┌────── parting line ──────┐
//                     │                          │
//                     │           (relief)       │
//                     │              ╲           │
//                     │              ◯  ←── small cylindrical pocket
//                     │                          │
//
// Signature pattern:
//   pattern.kind           = "undercut_relief"
//   pattern.dia_mm         = relief diameter
//   pattern.depth_mm       = relief depth (along axis_xy)
//
// DFM:
//   DFM-UC-DIA      : dia ∈ [0.5, 6.0] mm
//   DFM-UC-DEPTH    : depth ∈ [0.3, 8.0] mm
//   DFM-UC-AXIS     : axis_xy must be in XY plane (|Z| < 1e-3)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace undercut_relief {

constexpr const char* kSkillId = "undercut_relief";

struct Input
{
    double      corner_x_mm   = 0.0;
    double      corner_y_mm   = 0.0;
    double      corner_z_mm   = 0.0;
    gp_Dir      axis_xy       { 1.0, 0.0, 0.0 };   // in-plane direction the slide moves
    double      dia_mm        = 0.0;
    double      depth_mm      = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace undercut_relief
}  // namespace koocadcam::skill
