#pragma once
// @lat: [[engine/skills#busbar_lap_joint]]
//
// busbar_lap_joint — copper bus-bar end with a lap-joint fastener pattern.
// The skill takes a rectangular bar workpiece and adds the standard
// switchgear lap-joint terminal end:
//   1. Two diameter holes (through-pattern) for the fasteners (M10 typical).
//   2. One alignment notch on the long-edge midline (V or rectangular).
//   3. A grid of shallow impressions (slip-resistance surface) on the top
//      face, between the two holes.  Modeled as N×M shallow cylindrical
//      pits.
//
// Sub-feature count: 4 (hole1 + hole2 + notch + dimple grid).
//
//                       lap_length_mm
//                  ◄───────────────────►
//                  ┌───────────────────┐ ↑
//                  │  ◯       ▒▒▒    ◯ │ │ bar_width
//                  │          ▒▒▒      │ │
//                  │  ◯       ▒▒▒    ◯ │ ↓
//                  └────────┘ ▼ └──────┘
//                          notch
//
// DFM (IEEE 605, NEMA CC1):
//   - bar_thickness_mm ≥ 3 mm (current-carrying cross-section min)
//   - bolt_dia_mm ∈ standards [6, 8, 10, 12]
//   - bolt_pitch_mm ≥ 2.5 × bolt_dia_mm (edge distance per NEMA CC1)
//   - notch_depth_mm < bar_width_mm / 4
//   - dimple_dia_mm ≥ 0.5 mm

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace busbar_lap_joint {

constexpr const char* kSkillId = "busbar_lap_joint";

struct Input
{
    FaceDatum   entry_face;
    double      end_x_mm           = 0.0;   // anchor: midpoint of the bar's end edge
    double      end_y_mm           = 0.0;
    gp_Dir      axis_dir           { 0.0, 0.0, -1.0 };  // hole drill direction
    double      lap_length_mm      = 60.0;  // length of the lap region (toward bar body)
    double      bar_width_mm       = 40.0;
    double      bar_thickness_mm   = 5.0;
    double      bolt_dia_mm        = 10.0;  // M10 fastener
    double      bolt_pitch_mm      = 40.0;  // along the bar between two holes
    double      notch_depth_mm     = 6.0;   // along bar axis
    double      notch_width_mm     = 4.0;   // perpendicular
    int         dimple_grid_nx     = 3;     // number of dimples along bar
    int         dimple_grid_ny     = 3;     // number of dimples across
    double      dimple_dia_mm      = 1.5;
    double      dimple_depth_mm    = 0.3;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace busbar_lap_joint
}  // namespace koocadcam::skill
