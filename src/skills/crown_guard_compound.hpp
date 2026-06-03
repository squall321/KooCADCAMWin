#pragma once
// @lat: [[engine/skills#crown_guard_compound]]
//
// crown_guard_compound — COMPOUND FEATURE (additive, two protrusions).
//
// Crown guard: a pair of flanking material protrusions on the side wall of
// the watch case at the 3 o'clock position, above and below the crown axis,
// designed to protect the crown from lateral impacts.  Classic Panerai-style
// shoulder design.
//
// Each guard is modelled as a small box (thin extrusion off the case side)
// with a chamfered outer edge approximated as a smaller diagonal box cut.
//
// Sub-feature chain:
//   1. fuse upper guard box  (axial+ side of crown)
//   2. fuse lower guard box  (axial- side of crown)
//   3. optional chamfer cut on the outer corner of each guard (slice 1:
//      represented as metadata only; we still record subfeature_count = 2
//      since geometry includes 2 boxes).
//
// Output: case body with two flanking protrusions.  pattern.is_compound,
// is_watch_feature, subfeature_count = 2.
//
// DFM: guard_thickness > 0.4 mm, guard_height > 0, guard_separation > 0,
// guard_separation ≥ crown_dia + 2 × guard_thickness (clearance for crown).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace crown_guard_compound {

constexpr const char* kSkillId = "crown_guard_compound";

struct Input
{
    FaceDatum   case_side_face;
    gp_Ax1      case_axis            { gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1) };
    double      case_outer_radius_mm = 0.0;
    gp_Pnt      crown_axis_origin    { 0.0, 0.0, 0.0 };
    gp_Dir      crown_axis_dir       { 1.0, 0.0, 0.0 };  // radial outward at 3 o'clock
    double      guard_height_mm     = 4.0;   // axial extent of each guard
    double      guard_thickness_mm  = 1.5;   // tangential thickness
    double      guard_radial_extent_mm = 2.5;  // radial protrusion
    double      guard_separation_mm = 6.0;   // centre-to-centre axial spacing
    double      guard_chamfer_mm    = 0.4;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace crown_guard_compound
}  // namespace koocadcam::skill
