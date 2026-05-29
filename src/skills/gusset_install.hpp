#pragma once
// @lat: [[engine/skills#gusset_install]]
//
// gusset_install — installation record for a triangular/rectangular gusset
// plate that reinforces a steel-framing joint (e.g. brace-to-column, truss
// node).  The plate is welded around its perimeter to the joined members.
// The macroscopic workpiece B-rep is unchanged because the plate is treated
// as a sub-OCCT structural body in the assembly graph.
//
//           ┌──────────────┐
//           │\             │       gusset_size_mm = max edge of plate
//           │ \   gusset   │       weld_length_mm = total perimeter weld
//           │  \           │
//           └───\──────────┘
//                \
//                 ── member
//
// Signature pattern:
//   - pattern.kind             = "gusset_install"
//   - pattern.gusset_size_mm
//   - pattern.weld_length_mm
//   - pattern.geometry_changed = false
//
// DFM checks:
//   gusset_size_mm  ∈ [100, 1200]        (DFM-GUS-SIZE error if outside)
//   weld_length_mm  ≥ 2 × gusset_size_mm (DFM-GUS-WELD error if short — at
//                                         least two edges welded)
//   weld_length_mm  ≤ 4 × gusset_size_mm (DFM-GUS-WELD info  if longer than
//                                         plate perimeter — likely overspec)

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gusset_install {

constexpr const char* kSkillId = "gusset_install";

struct Input
{
    double gusset_size_mm = 300.0;
    double weld_length_mm = 800.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gusset_install
}  // namespace koocadcam::skill
