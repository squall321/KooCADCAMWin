#pragma once
// @lat: [[engine/skills#oil_change]]
//
// oil_change — maintenance record for draining used lubricating oil and
// refilling with fresh oil on a gearbox, hydraulic reservoir, or engine
// sump.  Oil is a sub-OCCT fluid volume that is NOT modelled as B-rep;
// this skill captures the grade + volume + drain temperature for the
// lubrication-management log.
//
//                ┌─────────────┐
//                │             │
//                │  ░░░░░░░░░  │   ← fresh oil (volume_l, oil_grade)
//                │  ░░░░░░░░░  │
//                │             │
//                └─────┬───────┘
//                      │
//                  ↓ drain plug (at drain_temp_c)
//
// Signature pattern:
//   - pattern.kind             = "oil_change"
//   - pattern.oil_grade        = ISO VG / SAE label (e.g. "ISO VG 46", "5W-30")
//   - pattern.volume_l         = litres added
//   - pattern.drain_temp_c     = sump temperature at drain time
//   - pattern.geometry_changed = false
//
// DFM checks:
//   oil_grade non-empty                       (DFM-INPUT error)
//   volume_l  > 0                             (DFM-INPUT error if ≤ 0)
//   drain_temp_c ∈ [30, 90] °C                (DFM-OIL-TEMP info if outside —
//                                              cold drains are slow, hot
//                                              drains risk operator burns)
//
// Synthesis:
//   NO geometric change.  Clone shape + material and stamp the signature.
//
// Recognize:
//   Metadata-only replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace oil_change {

constexpr const char* kSkillId = "oil_change";

struct Input
{
    std::string oil_grade    = "ISO VG 46";
    double      volume_l     = 10.0;
    double      drain_temp_c = 60.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace oil_change
}  // namespace koocadcam::skill
