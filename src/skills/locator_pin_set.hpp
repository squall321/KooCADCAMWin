#pragma once
// @lat: [[engine/skills#locator_pin_set]]
//
// locator_pin_set — compound additive+subtractive fixturing feature.
//
// A locator pin is a precision cylindrical post pressed into a fixture
// plate.  Parts drop over the pin so its body locates them.  The pin has
// 4 geometric features per DIN 6321 / ASME B94.6 — Round pins, ground:
//
//   1. SHOULDER — short large-Ø cylinder at the base (datum reference)
//   2. POST     — taller small-Ø cylinder above the shoulder (the
//                  locating diameter)
//   3. LEAD-IN  — 45° chamfer on top of the post (so parts drop over)
//   4. GROOVE   — annular ring undercut just above the shoulder, for an
//                  E-clip / retaining ring (DIN 471 std)
//
// Geometry chain on the input plate:
//   FUSE  shoulderCyl + postCyl → plate           (build the pin body)
//   CUT   chamferConeFrustum from result          (top lead-in)
//   CUT   grooveAnnularRing  from result          (E-clip undercut)
//
// 4 sub-features → subfeature_count = 4.
//
// DFM:
//   DIN 6321  — post length / post Ø ≤ 6 (else buckling risk)
//   ASME B94.6 — chamfer angle 30-45° typical, < 60° (else weak corner)
//   DFM-002    — any cylinder Ø ≥ 0.8 mm
//   shoulder_dia > post_dia (else no shoulder)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace locator_pin_set {

constexpr const char* kSkillId = "locator_pin_set";

struct Input
{
    FaceDatum   base_face;
    double      position_x_mm        = 0.0;
    double      position_y_mm        = 0.0;
    gp_Dir      axis_dir              { 0.0, 0.0, 1.0 };   // up out of plate
    double      shoulder_dia_mm      = 12.0;
    double      shoulder_height_mm   = 3.0;
    double      post_dia_mm          = 8.0;
    double      post_height_mm       = 20.0;
    double      lead_in_chamfer_mm   = 1.0;   // 45° on top
    double      groove_dia_mm        = 6.0;   // E-clip groove minor Ø
    double      groove_width_mm      = 1.5;   // E-clip groove axial width
    double      groove_z_offset_mm   = 1.0;   // distance above shoulder top
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace locator_pin_set
}  // namespace koocadcam::skill
