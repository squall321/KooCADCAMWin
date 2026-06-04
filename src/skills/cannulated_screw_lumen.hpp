#pragma once
// @lat: [[engine/skills#cannulated_screw_lumen]]
//
// cannulated_screw_lumen — COMPOUND cannulated (hollow) bone-screw feature:
//
//   sub-feature 1: drill central THROUGH lumen (for K-wire guide pin)
//   sub-feature 2: cylindrical screw-head recess at the entry face
//
// Cannulated screws are commonly used in trauma fixation (hip pinning,
// percutaneous fixation) — the central lumen accepts a guide wire so the
// screw can be advanced over the wire.
//
// Standards reference:
//   ISO 5832-1 / ASTM F138 : implant-grade stainless steel
//   ISO 5832-3 / ASTM F136 : Ti-6Al-4V
//   ASTM F543              : metallic medical bone screws (testing)
//   Typical lumen diameter : 1.0 .. 2.0 mm (for 1.0 or 1.6 mm K-wire)
//
// DFM:
//   DFM-CANN-LUMEN     : lumen_dia ≥ outer_dia * 0.5 (wall too thin)
//   DFM-CANN-LUMEN-MIN : lumen_dia < 0.5 mm
//   DFM-CANN-HEAD      : head_dia ≤ outer_dia
//   DFM-CANN-LENGTH    : length_mm ≤ head_depth_mm + 1.0
//
// signature.pattern.kind                  = "cannulated_screw_lumen"
// signature.pattern.is_compound           = true
// signature.pattern.medical_feature_type  = "cannulated_bone_screw"

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cannulated_screw_lumen {

constexpr const char* kSkillId = "cannulated_screw_lumen";

struct Input
{
    FaceDatum   entry_face;
    gp_Dir      screw_axis_dir    { 0.0, 0.0, -1.0 };
    gp_Pnt      screw_axis_origin { 0.0, 0.0, 0.0 };
    double      outer_dia_mm   = 0.0;
    double      lumen_dia_mm   = 0.0;     // typ 1.0 .. 2.0 mm
    double      length_mm      = 0.0;
    double      head_dia_mm    = 0.0;
    double      head_depth_mm  = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cannulated_screw_lumen
}  // namespace koocadcam::skill
