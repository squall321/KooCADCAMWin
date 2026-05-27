#pragma once
// @lat: [[engine/skills#dovetail_slot]]
//
// dovetail_slot — trapezoidal-cross-section slot.  The classic sliding-key
// joint geometry: bottom is wider than top (dovetail) or top is wider than
// bottom (reverse dovetail / male key).  Side walls slope at a controlled
// angle from the vertical.
//
//          entry_face
//              │
//   ───────────┼─────────────────
//              │                              top_width_mm
//              │                            ◄──────────────►
//              │      ╲                                 ╱
//              │       ╲   wall_angle_deg from vertical╱   ↑ depth_mm
//              │        ╲                             ╱    │
//              │         ╲___________________________╱     ↓
//                                bottom_width_mm
//                              ◄──────────────────────►
//                              (> top_width = dovetail)
//
// In cross section the slot is a TRAPEZOID.  We build it as an extruded
// prism: a trapezoidal cross-section wire (4 vertices) is extruded along
// the slot's length axis (start→end), then cut from the workpiece.
//
// Signature pattern:
//   - 2 angled planar wall faces (the dovetail sides; non-vertical normal)
//   - 1 planar flat-bottom face
//   - 2 vertical planar end-cap faces (the trapezoidal end profile)
//
// DFM checks:
//   DFM-002 : top_width_mm    ≥ 0.8 mm
//   DFM-002 : bottom_width_mm ≥ 0.8 mm
//   wall_angle_deg ∈ [15, 75]
//
// Recognize:
//   1. Find 4-wall pockets where the **side** walls are NOT perpendicular
//      to the entry face (i.e. their normals have non-trivial vertical
//      component).  Two opposed angled walls sharing a flat bottom.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace dovetail_slot {

constexpr const char* kSkillId = "dovetail_slot";

struct Input
{
    FaceDatum   entry_face;
    double      start_x_mm        = 0.0;
    double      start_y_mm        = 0.0;
    double      end_x_mm          = 0.0;
    double      end_y_mm          = 0.0;
    gp_Dir      axis_dir          { 0.0, 0.0, -1.0 };
    double      top_width_mm      = 0.0;
    double      bottom_width_mm   = 0.0;
    double      depth_mm          = 0.0;
    double      wall_angle_deg    = 30.0;   // angle of side walls from vertical
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace dovetail_slot
}  // namespace koocadcam::skill
