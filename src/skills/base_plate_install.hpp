#pragma once
// @lat: [[engine/skills#base_plate_install]]
//
// base_plate_install — installation record for the base plate at the foot of
// a steel column.  The plate is welded to the column underside and bolted to
// the concrete foundation through pre-cast anchor bolts.  The workpiece
// B-rep is unchanged; the plate, anchor bolt holes, and weld are sub-OCCT
// structural metadata.
//
//              ┌──── column ────┐
//              │                │
//              │                │
//         ─────┴────────────────┴─────       ← base plate (plate_thick_mm)
//              ° ° ° ° ° ° ° ° °             ← bolt_count anchor bolts
//         ═════════════════════════          ← concrete foundation
//
// Signature pattern:
//   - pattern.kind             = "base_plate_install"
//   - pattern.plate_thick_mm
//   - pattern.bolt_count
//   - pattern.anchor_dia_mm
//   - pattern.geometry_changed = false
//
// DFM checks:
//   plate_thick_mm ∈ [10, 80]            (DFM-BP-THK   error if outside)
//   bolt_count     ∈ [2, 16]             (DFM-BP-COUNT error if outside)
//   bolt_count     odd                   (DFM-BP-COUNT info — symmetry)
//   anchor_dia_mm  ∈ [12, 64]            (DFM-BP-ANCH  error if outside)

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace base_plate_install {

constexpr const char* kSkillId = "base_plate_install";

struct Input
{
    double plate_thick_mm = 25.0;
    int    bolt_count     = 4;
    double anchor_dia_mm  = 24.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace base_plate_install
}  // namespace koocadcam::skill
