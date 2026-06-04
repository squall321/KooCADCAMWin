#pragma once
// @lat: [[engine/skills#dental_implant_emergence_taper]]
//
// dental_implant_emergence_taper — COMPOUND dental-implant abutment
// emergence profile (transgingival taper):
//
//   sub-feature 1: conical TRANSITION cut from abutment_top_dia
//                  down to abutment_bottom_dia over transition_length
//                  (the transgingival emergence taper)
//   sub-feature 2: axial cylindrical screw bore at abutment center,
//                  diameter taken from M-thread table (_iso_thread_table)
//                  — represents the "tap" of an internal abutment screw
//                  (actual helical thread carved by tap_thread skill).
//
// The two cuts are concentric on implant_axis and FUSED into a single
// cutter before subtraction (counterbore pattern).
//
// Standards reference:
//   ISO 14801          Dental implant — dynamic fatigue test
//   ISO 5832 / ASTM F67/F136 implant Ti / Ti-6Al-4V
//   ASTM F1295         Wrought Ti-6Al-7Nb implant alloy
//
// DFM:
//   DFM-DENTAL-TAPER     : top_dia ≤ bottom_dia (no transition)
//   DFM-DENTAL-TRANS-LEN : transition_length_mm ≤ 0
//   DFM-DENTAL-THREAD    : screw_thread_size not in iso_thread_table
//
// signature.pattern.kind                  = "dental_implant_emergence_taper"
// signature.pattern.is_compound           = true
// signature.pattern.medical_feature_type  = "dental_emergence_profile"

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace dental_implant_emergence_taper {

constexpr const char* kSkillId = "dental_implant_emergence_taper";

struct Input
{
    FaceDatum   entry_face;
    gp_Dir      implant_axis_dir    { 0.0, 0.0, -1.0 };
    gp_Pnt      implant_axis_origin { 0.0, 0.0, 0.0 };   // entry-plane center
    double      abutment_top_dia_mm    = 0.0;
    double      abutment_bottom_dia_mm = 0.0;
    double      transition_length_mm   = 0.0;
    std::string screw_thread_size;          // "M3", "M4", ... (iso_thread_table key)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace dental_implant_emergence_taper
}  // namespace koocadcam::skill
