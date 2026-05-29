#pragma once
// @lat: [[engine/skills#gas_strut_hinge_compound]]
//
// gas_strut_hinge_compound — bracket fitting for a gas spring / strut hinge
// with a frame-side pivot, a door-side pivot, a clevis pocket for the
// strut rod end, and a retention groove for the ball-end snap socket.
//
// REAL geometry chain (5 cuts):
//   1. Frame pivot bore — through-bore at the bracket frame end.
//   2. Door pivot bore — through-bore at the bracket door end (offset).
//   3. Clevis pocket — rectangular pocket sized for the strut rod end
//      bushing, opens to one face (oriented perpendicular to both pivot
//      axes).
//   4. Ball-end retention groove — annular cut concentric with the door
//      pivot bore (on the back face), with a slight undercut so the ball
//      snap can NOT pull straight out.
//   5. Lightening hole — central through-hole between the pivots, reduces
//      mass and gives a visual reference for installation.
//
// Standard:
//   - DIN 71803 ball-and-socket gas spring fitting series.
//   - ISO 8434-2 SAE-style clevis dimensions.
//
// DFM gates:
//   - pivot_bore_dia_mm ≥ 3 mm (M3 strut pin minimum).
//   - frame_door_spacing_mm > pivot_bore_dia × 3 (web wall ≥ 1 dia).
//   - clevis_pocket_width_mm > pivot_bore_dia (else clevis can't enter).
//   - ball_groove_inner_dia_mm > pivot_bore_dia.
//   - lightening_hole_dia_mm <= frame_door_spacing_mm × 0.5.
//
// Signature:
//   - is_compound = true
//   - subfeature_count = 5
//
// Recognize:
//   - PRIMARY (geometric): find 2 large Z-axis cylinders separated by a
//     spacing AND at least one rectangular pocket — those are the two
//     pivots + clevis.  Plus annular face (retention groove) and a small
//     central cylinder.
//   - FALLBACK (metadata replay): confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gas_strut_hinge_compound {

constexpr const char* kSkillId = "gas_strut_hinge_compound";

struct Input
{
    FaceDatum   entry_face;
    double      frame_pivot_x_mm        = 0.0;
    double      frame_pivot_y_mm        = 0.0;
    double      door_pivot_x_mm         = 30.0;     // offset along +X from frame pivot
    double      door_pivot_y_mm         = 0.0;
    gp_Dir      axis_dir                { 0.0, 0.0, -1.0 };
    double      bracket_thickness_mm    = 8.0;      // depth of all -Z bores
    double      pivot_bore_dia_mm       = 6.0;      // common M6 pin
    double      clevis_pocket_length_mm = 14.0;     // along door->frame direction
    double      clevis_pocket_width_mm  = 8.0;      // perpendicular in XY
    double      clevis_pocket_depth_mm  = 5.0;
    double      ball_groove_inner_dia_mm = 9.0;     // > pivot dia
    double      ball_groove_outer_dia_mm = 11.0;
    double      ball_groove_depth_mm    = 1.5;
    double      lightening_hole_dia_mm  = 4.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gas_strut_hinge_compound
}  // namespace koocadcam::skill
