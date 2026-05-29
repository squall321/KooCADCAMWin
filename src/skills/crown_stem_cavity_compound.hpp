#pragma once
// @lat: [[engine/skills#crown_stem_cavity_compound]]
//
// crown_stem_cavity_compound — COMPOUND FEATURE (3 primitive cuts).
//
// Watch-case crown port: a horizontal cylindrical pocket cut from the case
// side at a specified angular position, with a coaxial through hole for the
// stem and an interior annular O-ring groove (AS568-001 style) at the back
// of the pocket where the crown gasket seats.  Chains:
//
//   1. cavity cut: large-bore cylindrical pocket (Ø ~3 mm) into the side of
//      the case at angle_deg.
//   2. stem cut: smaller-bore through-hole (Ø ~1.5 mm) coaxial with cavity,
//      passing all the way to the case interior.
//   3. O-ring groove: annular ring cut inside the cavity wall (AS568 cross-
//      section ≈ 1.0 mm width × 0.4 mm depth for a 3 mm cavity).
//
// Cavity and stem cylinders OVERLAP (stem passes through cavity bottom), so
// they're FUSED first then cut once.  O-ring groove is a separate cut.
//
// Output: case with 1 stepped horizontal port (cavity + stem stages + groove).
// Single FeatureSignature, pattern.is_compound = true, subfeature_count = 3.
//
// DFM gates:
//   - cavity_dia > stem_dia + 0.8 mm  (radial shelf for O-ring seat)
//   - stem_dia ≥ 0.8 mm (DFM-002)
//   - cavity_depth in [1.5, 5] mm (typical for waterproof crown ports)
//   - O-ring cross-section in AS568 catalog (1.0 mm here = -001 family)
//   - case wall at crown must be ≥ 1.0 mm at thinnest (cavity-to-OD margin)
//
// Recognize: metadata replay (cavity + coaxial through-hole is ambiguous with
// counterbore + drill_hole on geometry alone).  Confidence 1.0 from history.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace crown_stem_cavity_compound {

constexpr const char* kSkillId = "crown_stem_cavity_compound";

struct Input
{
    gp_Ax1   case_axis;
    double   case_outer_radius_mm = 0.0;
    double   angle_deg            = 0.0;   // angular position around case
    double   port_center_z_mm     = 0.0;   // Z height of port axis
    double   cavity_dia_mm        = 3.0;
    double   cavity_depth_mm      = 2.5;   // inward from case OD
    double   stem_dia_mm          = 1.5;
    double   stem_total_length_mm = 8.0;   // cavity_depth + into-case length
    double   o_ring_cs_mm         = 1.0;   // AS568 cross-section width
    double   o_ring_depth_mm      = 0.4;   // groove depth into cavity wall
    double   o_ring_offset_mm     = 1.0;   // axial offset of groove from cavity entry
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace crown_stem_cavity_compound
}  // namespace koocadcam::skill
