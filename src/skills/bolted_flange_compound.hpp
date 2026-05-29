#pragma once
// @lat: [[engine/skills#bolted_flange_compound]]
//
// bolted_flange_compound — REAL multi-cut compound feature for a bolted
// pipe flange.  Five sub-features are built into a single result solid:
//
//   1. flange disc           (annular boss fused onto the base workpiece)
//   2. central bore          (axial through-hole — pipe ID)
//   3. raised face           (axial protrusion ring around the central bore)
//   4. gasket groove         (toroidal-equivalent annular cut into raised face)
//   5. bolt-hole pattern     (N coaxial drills on a bolt-circle, evenly spaced)
//
// Single output FeatureSignature.is_compound = true,
// subfeature_count = 4 + bolt_count (≥ 6 for 4-bolt, ≥ 10 for 8-bolt).
//
// DFM standard cited:
//   • ASME B16.5 / B16.20 — bolt count ∈ {4, 8, 12}; bolt-hole dia 1.6 mm
//     larger than bolt nominal; raised-face height ≥ 1.6 mm; gasket groove
//     ID/OD per B16.20 ring-joint dimensions; bolt-circle ≥ outer_dia - 30 mm
//     (clearance for nut socket).
//
// Recognize (geometric primary):
//   1. Find the largest cylindrical face whose axis is the flange axis;
//      that defines the central bore.
//   2. Count axisymmetric cylindrical faces sharing this axis at the
//      bolt-circle radius — must be ≥ 4 evenly-spaced.
//   3. Confirm a planar annular raised-face top (planar face with two
//      concentric circular edges + axial offset above the disc top).
//
//   Confidence 0.85 (geometric) / 1.0 (metadata replay).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bolted_flange_compound {

constexpr const char* kSkillId = "bolted_flange_compound";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm        = 0.0;
    double      center_y_mm        = 0.0;
    gp_Dir      axis_dir           { 0.0, 0.0, 1.0 };   // disc grows along +axis
    double      flange_outer_dia_mm = 200.0;             // outer rim of the disc
    double      flange_thickness_mm = 22.0;              // disc thickness
    double      pipe_bore_dia_mm    = 100.0;             // central through-hole
    double      raised_face_dia_mm  = 142.0;             // raised face OD
    double      raised_face_height_mm = 2.0;             // RF protrusion above disc top
    double      gasket_groove_id_mm = 122.0;             // gasket groove inside dia
    double      gasket_groove_od_mm = 130.0;             // gasket groove outside dia
    double      gasket_groove_depth_mm = 1.5;            // depth into raised face
    int         bolt_count          = 8;                 // # of bolt holes
    double      bolt_circle_dia_mm  = 170.0;             // PCD
    double      bolt_hole_dia_mm    = 22.0;              // each bolt hole dia
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bolted_flange_compound
}  // namespace koocadcam::skill
