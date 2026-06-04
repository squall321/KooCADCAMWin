#pragma once
// @lat: [[engine/skills#bone_screw_pilot_compound]]
//
// bone_screw_pilot_compound — COMPOUND orthopedic bone-screw seat:
//
//   sub-feature 1: pilot bore (AO/ASIF cortical pilot drill diameter)
//   sub-feature 2: 90° countersink at the entry face so the screw head sits flush
//   sub-feature 3: small entry chamfer at the top of the countersink (de-burr / lead-in)
//
// All three cuts are concentric on the same axis and are FUSED into a
// single connected cutter before subtraction (counterbore pattern).
//
// Standards reference:
//   ISO 5836:1988                Bone screws (terminology + dimensions)
//   ISO 5832-1 / ASTM F138       Stainless steel implant material
//   ISO 5832-3 / ASTM F136       Ti-6Al-4V implant material
//   AO/ASIF                      Cortical screw sizes: 2.0/2.4/2.7/3.5/4.0/4.5/6.5 mm
//
// AO/ASIF cortical pilot drill recommendations (mm OD → pilot dia):
//   2.0 → 1.5,  2.4 → 1.8,  2.7 → 2.0,  3.5 → 2.5,
//   4.0 → 2.9,  4.5 → 3.2,  6.5 → 4.5
//
// DFM:
//   DFM-BONESCREW-SIZE      : screw_size_id not in AO/ASIF cortical set
//   DFM-BONESCREW-COUNTERSINK: countersink_dia ≤ pilot_dia (no cone produced)
//   DFM-BONESCREW-CHAMFER   : chamfer_dia ≤ countersink_dia
//   DFM-BONESCREW-DEPTH     : depth_mm ≤ 0
//
// signature.pattern.kind             = "bone_screw_pilot_compound"
// signature.pattern.is_compound      = true
// signature.pattern.medical_feature_type = "cortical_bone_screw_pilot"

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bone_screw_pilot_compound {

constexpr const char* kSkillId = "bone_screw_pilot_compound";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm    = 0.0;
    double      position_y_mm    = 0.0;
    gp_Dir      axis_dir          { 0.0, 0.0, -1.0 };
    std::string screw_size_id;                 // "2.0" .. "6.5"
    double      depth_mm          = 0.0;       // pilot bore depth (from entry face)
    double      countersink_dia_mm = 0.0;      // CSK top diameter (mm)
    double      chamfer_dia_mm    = 0.0;       // edge chamfer top dia (mm)
};

// AO/ASIF pilot drill diameter (mm) for a given cortical screw size.
// Returns 0.0 if size_id unknown.
double pilotDiaFor(const std::string& screw_size_id);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bone_screw_pilot_compound
}  // namespace koocadcam::skill
