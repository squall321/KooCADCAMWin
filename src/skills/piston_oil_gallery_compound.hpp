#pragma once
// @lat: [[engine/skills#piston_oil_gallery_compound]]
//
// piston_oil_gallery_compound — AUTOMOTIVE ICE engine compound: forged/cast
// piston cooling oil gallery (annular cavity under the crown) plus inlet
// jet hole and drain hole on opposite side of the piston skirt.
//
// Sub-feature chain (3 ops):
//   1. CUT annular gallery cavity inside the piston (large OD ring).
//   2. CUT inlet jet hole — small bore from skirt up to gallery.
//   3. CUT drain hole at opposite side — drains warm oil back to sump.
//
// Signature:
//   pattern.kind                  = "piston_oil_gallery_compound"
//   pattern.engine_feature_type   = "piston_oil_gallery"
//   pattern.subfeature_count      = 3
//   pattern.derived_volume_removed_mm3
//
// DFM gates:
//   - gallery_OD > gallery_ID (annular makes sense)
//   - gallery_height > 0
//   - inlet/drain dia positive and < gallery wall thickness
//   - all positive

#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace piston_oil_gallery_compound {

constexpr const char* kSkillId = "piston_oil_gallery_compound";

struct Input
{
    int     face_id            = -1;
    gp_Dir  piston_axis_dir    { 0.0, 0.0, 1.0 };
    gp_Pnt  piston_axis_origin { 0.0, 0.0, 0.0 };
    double  gallery_OD_mm      = 60.0;
    double  gallery_ID_mm      = 40.0;
    double  gallery_height_mm  = 6.0;
    double  inlet_hole_dia_mm  = 3.0;
    double  drain_hole_dia_mm  = 2.5;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace piston_oil_gallery_compound
}  // namespace koocadcam::skill
