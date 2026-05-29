#pragma once
// @lat: [[engine/skills#cold_form_section]]
//
// cold_form_section — installation/record skill for a cold-formed light-gauge
// steel framing member.  A flat coil is roll-formed at room temperature into
// a C, Z, or sigma (Σ) section and cut to length.  This skill captures the
// finished section as metadata on the workpiece — the macroscopic OCCT B-rep
// is unchanged because the section is treated as a sub-OCCT structural
// member (separate body in the framing assembly).
//
//        ┌────────────┐       ┌────────────┐       ┌──────┐
//        │  C section │       │  Z section │       │ Σ    │
//        │     ┌──    │       │  ──┐       │       │ ┌─┐  │
//        │     │      │       │    │       │       │ │ │  │
//        │     │      │       │    │       │       │ └─┘  │
//        │     └──    │       │       ──┐  │       │      │
//        └────────────┘       └────────────┘       └──────┘
//
// Signature pattern:
//   - pattern.kind             = "cold_form_section"
//   - pattern.section_profile  = "C" | "Z" | "sigma"
//   - pattern.thickness_mm
//   - pattern.length_m
//   - pattern.geometry_changed = false
//
// DFM checks:
//   thickness_mm   ∈ [0.5, 4.0]         (DFM-CFS-THK  error if outside)
//   length_m       ∈ (0, 18.0]          (DFM-CFS-LEN  error if outside)
//   section_profile ∈ {C,Z,sigma}       (DFM-CFS-PROF error if unknown)

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cold_form_section {

constexpr const char* kSkillId = "cold_form_section";

struct Input
{
    std::string section_profile = "C";     // "C" | "Z" | "sigma"
    double      thickness_mm    = 1.6;
    double      length_m        = 3.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cold_form_section
}  // namespace koocadcam::skill
