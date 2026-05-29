#pragma once
// @lat: [[engine/skills#cut_apparel]]
//
// cut_apparel — record-only pattern cutting for garment manufacturing.
// The B-rep workpiece represents the fabric ply / lay-up; the cut produces
// downstream pattern pieces handled by sewing/embroidery skills.  No
// geometric Boolean is performed (pattern outlines live in 2D nest space,
// not in OCCT); this skill stamps the cut-record signature so MES & CAPP
// can downstream verify ply count, cutter type, and pattern reference.
//
// Signature pattern:
//   - pattern.kind         = "cut_apparel"
//   - pattern.pattern_id   = <input>
//   - pattern.cut_layers   = <input>
//   - pattern.cut_method   = "manual" | "laser" | "die"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   pattern_id      non-empty                       (DFM-INPUT error)
//   cut_layers      ≥ 1                             (DFM-INPUT error)
//   cut_layers      ≤ 250                           (DFM-CUT-LAYERS warning)
//   cut_method      ∈ {manual, laser, die}          (DFM-CUT-METHOD info)
//
// Synthesis:
//   NO geometric change — clone workpiece and stamp the signature.
//
// Recognize:
//   Metadata-only replay from `workpiece.features()`.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cut_apparel {

constexpr const char* kSkillId = "cut_apparel";

struct Input
{
    std::string  pattern_id  = "PATTERN-001";
    int          cut_layers  = 1;
    std::string  cut_method  = "laser";   // "manual" | "laser" | "die"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cut_apparel
}  // namespace koocadcam::skill
