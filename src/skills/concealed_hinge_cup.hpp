#pragma once
// @lat: [[engine/skills#concealed_hinge_cup]]
//
// concealed_hinge_cup — COMPOUND HINGE FEATURE for the standard Euro-style
// (Blum / Hettich) concealed cabinet hinge.
//
// The hinge "cup" is a 35 mm Ø blind bore (depth ≈ 11.5 mm) drilled into
// the door panel, paired with TWO M3.5 fixing-screw pilot holes spaced
// EXACTLY 32 mm apart on a line offset 6 mm from the door edge — the
// industry-standard "32 mm system" pitch.
//
// Sub-feature chain (3 primitive cuts):
//   1. 35 mm Ø cup blind bore
//   2. left fixing-screw pilot (M3.5, depth 10 mm)
//   3. right fixing-screw pilot (M3.5, depth 10 mm)
//
//      Door edge
//      │                  ↑
//      │      32 mm pitch │
//      │   ◎─────────────◎      ◎ = M3.5 pilots
//      │   │   ╱─────╲   │
//      │   │  │ Ø 35 │  │       cup bore
//      │   │   ╲─────╱   │
//      │
//
// Reference: European 32 mm Cabinet System (DIN 30798, Blum standards).
//
// DFM gates:
//   DFM-INPUT       : position_xy values finite
//   DFM-HINGE-CUP   : panel thickness ≥ 14 mm (cup depth + 2.5 mm margin)
//   DFM-32SYSTEM    : workpiece must contain the 32 mm pitch line fully

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace concealed_hinge_cup {

constexpr const char* kSkillId = "concealed_hinge_cup";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm  = 0.0;   // cup center X
    double      position_y_mm  = 0.0;   // cup center Y
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    // Edge_offset_mm — the cup's perpendicular distance from the door edge.
    // Standard is 4 – 6 mm depending on hinge model; we default to 6.
    double      edge_offset_mm = 6.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace concealed_hinge_cup
}  // namespace koocadcam::skill
