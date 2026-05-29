#pragma once
// @lat: [[engine/skills#pcb_standoff_array_under_board]]
//
// pcb_standoff_array_under_board — Smart-electronics compound skill that
// adds an array of cylindrical PCB standoff bosses to a base plate (the
// inside surface of a metal-front enclosure), each with a tapped pilot
// hole sized for an M2 / M2.5 / M3 mounting screw.
//
// Each standoff site:
//   1. BOSS         — material ADDED (fuse): cylinder, dia = boss_od_mm,
//                     height = standoff_height_mm, rising off the base
//                     plate (so the PCB sits standoff_height_mm above it).
//   2. SCREW PILOT  — material REMOVED (cut): blind hole, dia from the
//                     mounting-screw lookup table (M2 → 1.6 mm pilot, etc),
//                     depth = standoff_height_mm + pilot_extra_depth_mm,
//                     entering the TOP of the boss.
//   3. PCB CLEARANCE — material REMOVED (cut) in the plate beneath the
//                     screw axis: counterbored relief equal to the PCB
//                     thickness so the screw head can sit flush if needed
//                     (only created when PCB_thickness_mm > 0).
//
// CONTEXT-AWARE: When the supplied base plate's bounding box is smaller
// than the convex hull of the standoff array, the skill AUTO-EXTENDS the
// base plate (adds a thin material plate just large enough to cover all
// sites with a `plate_margin_mm` border) before placing the bosses.  This
// matches the user instruction "Auto-extends base plate where missing."
//
// DFM:
//   IEC 60068 / IPC-2221 — PCB hole-to-pad clearance ≥ 0.5 mm
//   ISO 4762            — M-series tapped pilot per metric coarse table
//   DFM-002             — pilot Ø ≥ 0.8 mm (smallest carbide drill)
//   DFM-BOSS-WALL       — (boss_od − pilot_dia) / 2 ≥ 1.0 mm (wall thickness)
//   DFM-STANDOFF-HEIGHT — standoff_height_mm ≥ 1.5 mm (mfg. minimum)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pcb_standoff_array_under_board {

constexpr const char* kSkillId = "pcb_standoff_array_under_board";

struct StandoffSite
{
    double x_mm = 0.0;
    double y_mm = 0.0;
};

struct Input
{
    FaceDatum                base_face;         // top face of base plate (boss grows from here)
    gp_Dir                   axis_dir            { 0.0, 0.0, 1.0 };   // boss rises along +Z
    std::string              screw_size          = "M3";              // "M2"/"M2.5"/"M3"/"M4"
    double                   standoff_height_mm  = 4.0;
    double                   boss_od_mm          = 5.0;
    double                   pilot_extra_depth_mm = 1.0;
    double                   PCB_thickness_mm    = 0.0;               // 0 → skip clearance
    double                   plate_margin_mm     = 3.0;               // border for auto-extend
    double                   plate_thickness_mm  = 2.0;               // when auto-extending
    std::vector<StandoffSite> sites;                                   // ≥ 1 position
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pcb_standoff_array_under_board
}  // namespace koocadcam::skill
