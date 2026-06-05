#pragma once
// @lat: [[engine/skills#wing_rib_lightening_hole]]
//
// wing_rib_lightening_hole — Wing-rib lightening holes (aerospace structures).
//
// Wing/spar ribs carry a row of large lightening holes through the web to
// shed weight while preserving shear stiffness; each hole gets a shallow
// counterbore "flange" seat where a rolled/spun flange would be formed for
// edge stiffening.  We model N (bore + flange-counterbore) pairs along a row.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   per hole: 1) through bore   2) shallow flange counterbore
//   subfeature_count = hole_count * 2.
//
// DFM:
//   DFM-INPUT  : every dimension/count must be > 0.
//   DFM-PITCH  : pitch_mm must exceed hole_dia_mm + 2*flange_width_mm
//                (otherwise adjacent flange seats overlap).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace wing_rib_lightening_hole {

constexpr const char* kSkillId = "wing_rib_lightening_hole";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    row_origin      { 0.0, 0.0, 0.0 };   // first hole centre (XY)
    double    hole_dia_mm     { 20.0 };
    double    flange_width_mm { 3.0 };             // radial flange seat width
    int       hole_count      { 3 };
    double    pitch_mm        { 35.0 };            // centre-to-centre along +X
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace wing_rib_lightening_hole
}  // namespace koocadcam::skill
