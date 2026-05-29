#pragma once
// @lat: [[engine/skills#leaf_spring_anchor]]
//
// leaf_spring_anchor — COMPOUND SPRING-RETENTION FEATURE.
//
// A leaf-spring anchor pocket is the receiving cavity that holds a flat
// spring tab captive at one end (snap-in retention).  It chains a
// rectangular slot pocket with a retention undercut (1 mm overhang at
// the top that the spring snaps under) and a chamfer entry that guides
// the spring tab during insertion.
//
// Sub-feature chain (3 primitive cuts):
//   1. main rectangular slot pocket            (length × width × depth)
//   2. undercut box                            (length × (width+2 mm) × 1 mm
//                                              positioned 1 mm below the
//                                              entry plane so it produces a
//                                              1 mm overhanging lip on each
//                                              long edge)
//   3. chamfer entry ring                      (cone frustum 45°, 0.5 mm)
//
//      ──┬─────  entry plane, with 0.5 mm chamfer guide
//        │  ╲chamfer
//      ─ ┤   │ ←   1 mm overhang lip
//        │   │
//        │   │      slot pocket
//        │   │
//        └───┘
//
// The undercut is what makes this a SPRING ANCHOR rather than a plain
// slot.  When the leaf spring is inserted, it deflects past the chamfer
// and snaps into the wider undercut region; the overhanging lip then
// retains it captive.
//
// DFM gates:
//   DFM-INPUT          : slot_length, slot_width > 0
//   DFM-ANCHOR-LIP     : slot_width ≥ 4 mm (need room for 1 mm × 2 lips +
//                        spring travel)
//   DFM-ANCHOR-DEPTH   : depth ≥ 4 mm (undercut + chamfer + working depth)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace leaf_spring_anchor {

constexpr const char* kSkillId = "leaf_spring_anchor";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    double      slot_length_mm = 0.0;   // X extent
    double      slot_width_mm  = 0.0;   // Y extent
    double      depth_mm       = 6.0;   // pocket depth (default 6 mm)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace leaf_spring_anchor
}  // namespace koocadcam::skill
