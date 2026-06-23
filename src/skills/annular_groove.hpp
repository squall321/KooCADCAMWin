#pragma once
// @lat: [[engine/skills#annular_groove]]
//
// annular_groove — a plain annular (ring) groove or pocket cut into a flat face.
//
// The general "ring channel" feature: a watch bezel retention pocket, a phone
// camera decorative ring, any concentric ring recess.  Distinct from
// o_ring_groove_face (which is a SEAL seat — it maps the groove to an AS568
// O-ring size and gates on seal width/depth ratios), and from
// bezel_groove_assembly (a 4-cut compound with a tip chamfer + bottom fillet,
// recognised by metadata only).  This is the simple, geometry-recognisable
// primitive: two concentric cylinder walls + a flat annular floor.
//
//   apply():  resolve entry_face -> gp_Ax2 at the ring centre -> pr::annularRing
//             (or pr::annularConeRing when tapered) -> pr::cut.
//
// Recognise (geometric, foreign): a pair of CO-AXIAL cylinder faces of
// different radii, axially overlapping, whose deep ends are joined by a flat
// ANNULAR floor face (normal parallel to the axis, with a central hole).  The
// annular floor is what makes this specific — it distinguishes a real ring
// channel from an O-ring U-groove (rounded floor) and from two unrelated bores.
//
// DFM:
//   - outer_dia_mm > inner_dia_mm + 0.4 mm  (min groove width)
//   - depth_mm > 0

#include "Datum.hpp"
#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace annular_groove {

constexpr const char* kSkillId = "annular_groove";

struct Input
{
    FaceDatum entry_face;                  // planar face the ring is cut into
    double    center_x_mm = 0.0;           // ring centre in face-local XY
    double    center_y_mm = 0.0;
    double    outer_dia_mm = 0.0;
    double    inner_dia_mm = 0.0;          // < outer_dia
    double    depth_mm     = 0.0;
    double    taper_deg    = 0.0;          // > 0 => tapered (conical) outer wall
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace annular_groove
}  // namespace koocadcam::skill
