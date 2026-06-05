#pragma once
// @lat: [[engine/skills#euro_hinge_cup_35mm]]
//
// euro_hinge_cup_35mm — 35 mm European concealed-hinge cup boring
// (slice 16, furniture / cabinetry hardware).
//
// Frameless ("Euro") cabinet doors mount on concealed hinges whose cup
// presses into a flat-bottomed 35 mm bore in the door back, retained by two
// wood screws on either side of the cup.  This skill machines that pattern
// from a flat (top) face of a door panel:
//   1. flat-bottom cup bore (blind 35 mm cylinder cut DOWN from top face)
//   2..3. two screw pilot holes straddling the cup at screw_spacing
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. cup bore             (cylinder cut DOWN from top face)
//   2..3. two pilot holes   (cylinders cut DOWN, non-overlapping the cup)
//
// subfeature_count = 1 + 2 = 3.
//
// DFM:
//   DFM-INPUT  : all dimensions must be > 0.
//   DFM-CUPDIA : cup_dia_mm must be in [25, 40].
//   DFM-PILOT  : pilot holes (at +/- spacing/2) must clear the cup wall.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace euro_hinge_cup_35mm {

constexpr const char* kSkillId = "euro_hinge_cup_35mm";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy           { 0.0, 0.0, 0.0 };
    double      cup_dia_mm          { 35.0 };   // standard Euro cup
    double      cup_depth_mm        { 11.5 };   // blind cup depth
    double      screw_spacing_mm    { 45.0 };   // pilot-to-pilot centre (X)
    double      screw_pilot_dia_mm  { 2.5 };    // pilot drill diameter
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace euro_hinge_cup_35mm
}  // namespace koocadcam::skill
