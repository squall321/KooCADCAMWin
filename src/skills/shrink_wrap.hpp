#pragma once
// @lat: [[engine/skills#shrink_wrap]]
//
// shrink_wrap — apply a heat-shrink polymer film around the product and
// pass through a shrink tunnel (oven) so the film contracts and tightens
// against the product surface.  Used for retail multipack bundling
// (PVC / POF / PE) and tamper-evident closures.  The geometric envelope is
// unchanged; the film + oven cycle metadata is recorded.
//
//        ┌─────────────┐                  ┌─────────────┐
//        │  loose film │   shrink_wrap    │ taut film   │
//        │  over part  │ ───────────────▶│  on part    │
//        │             │ (oven T, dwell)  │             │
//        └─────────────┘                  └─────────────┘
//        geometry unchanged ; film + cycle metadata stamped
//
// Signature pattern:
//   pattern.kind             = "shrink_wrap"
//   pattern.film_type        = string  ("pvc" | "pof" | "pe")
//   pattern.oven_temp_c      = double
//   pattern.dwell_time_s     = double
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             film_type empty / unknown / oven_temp_c ≤ 0 /
//                         dwell_time_s ≤ 0
//   DFM-SHRINK-TEMP-LOW   oven_temp_c < 100 (info — incomplete shrink)
//   DFM-SHRINK-TEMP-HIGH  oven_temp_c > 220 (info — film degradation risk)
//   DFM-SHRINK-DWELL      dwell_time_s > 30 (info — heat damage risk)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace shrink_wrap {

constexpr const char* kSkillId = "shrink_wrap";

struct Input
{
    std::string  film_type      = "pof";   // "pvc" | "pof" | "pe"
    double       oven_temp_c    = 160.0;
    double       dwell_time_s   = 8.0;
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace shrink_wrap
}  // namespace koocadcam::skill
