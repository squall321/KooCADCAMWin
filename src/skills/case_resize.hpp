#pragma once
// @lat: [[engine/skills#case_resize]]
//
// case_resize — record full-length resize + neck-anneal of a fired brass case
// to restore SAAMI dimensions for authorized civilian/commercial small-arms
// reloading.  The macroscopic B-rep delta is below CAD-relevant tolerance,
// so this is captured as a metadata-only operation.  The skill stamps the
// resized neck diameter, trimmed case length, and anneal temperature so
// downstream loading + QC steps can verify the case is in-spec.
//
// Signature pattern:
//   - pattern.kind              = "case_resize"
//   - pattern.case_dia_neck_mm  = resized neck OD
//   - pattern.case_length_mm    = trimmed length
//   - pattern.anneal_temp_c     = neck anneal temperature
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   case_dia_neck_mm  ∈ (3.0, 25.0)        (DFM-AMMO-NECK error if outside)
//   case_length_mm    ∈ (5.0, 100.0)       (DFM-AMMO-CASELEN error if outside)
//   anneal_temp_c     ∈ [350, 550]         (DFM-AMMO-ANNEAL info if outside;
//                                           neck-anneal sweet spot)
//
// Synthesis: metadata-only.
// Recognize: metadata replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace case_resize {

constexpr const char* kSkillId = "case_resize";

struct Input
{
    double case_dia_neck_mm = 9.55;
    double case_length_mm   = 19.15;
    double anneal_temp_c    = 425.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace case_resize
}  // namespace koocadcam::skill
