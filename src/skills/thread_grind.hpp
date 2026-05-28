#pragma once
// @lat: [[engine/skills#thread_grind]]
//
// thread_grind — finishes an existing thread (internal or external) by
// running a precision grind wheel along the helical path.  Removes a very
// small radial amount (typical 0.005-0.02 mm), bringing the thread to a
// tighter tolerance class (e.g., 4H/5g down from 6H/6g).
//
// Geometrically:
//   - For `removal_mm < OCCT_tolerance` (typically < 0.01 mm), the
//     geometric change is below the modeling tolerance — we record only
//     the metadata and leave the shape unchanged.
//   - For larger removal we run a real helical sweep with reduced V-depth
//     and subtract from the workpiece.
//
//            ╲ │ ╱
//             ╲│╱      ← thread crest (before)
//              ▼        — grind wheel removes a few microns of crest+flank
//             ╱│╲
//            ╱ │ ╲      ← thread crest (after — sharper, tighter tolerance)
//
// Signature pattern:
//   - Identical thread topology to the parent thread_mill_external /
//     tap_thread / rigid_tap; the pattern.kind is "thread_grind" and a
//     parent thread feature must exist in `wp.features()` (referenced
//     via `pattern.parent_thread_id`).
//
// DFM checks:
//   DFM-GRIND-MIN     : thread diameter ≥ 5 mm (smaller threads can't
//                       accommodate the grind wheel).
//   DFM-GRIND-AMOUNT  : removal_mm ∈ [0.001, 0.05] mm.
//   DFM-GRIND-PARENT  : workpiece must have a prior thread feature
//                       (tap_thread / rigid_tap / form_tap / thread_mill /
//                       thread_mill_external).
//
// Recognize:
//   Same as thread_mill_external — metadata-driven.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace thread_grind {

constexpr const char* kSkillId = "thread_grind";

struct Input
{
    FaceDatum   cylinder_datum;            // cylindrical face being ground
    std::string thread_size      = "M6";
    double      pitch_mm         = 1.0;
    double      thread_length_mm = 0.0;
    double      start_z_mm       = 0.0;
    double      end_z_mm         = 0.0;
    double      removal_mm       = 0.01;   // radial material removed by grind wheel
    bool        is_external      = true;   // false → internal thread grind
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// OCCT modeling tolerance threshold below which we treat the operation as
// metadata-only (geometric change too small to represent reliably).
constexpr double kSubToleranceRemovalMm = 0.005;

}  // namespace thread_grind
}  // namespace koocadcam::skill
