#pragma once
// @lat: [[engine/skills#bag_seal]]
//
// bag_seal — heat-seal a flexible bag closed.  The bag (a pouch made of
// laminated film) is closed by fusing two layers of film together along a
// seal line.  Three seal modalities are supported:
//
//   "heat"        — conductive heat from a jaw bar  (most common, lowest cost)
//   "ultrasonic"  — high-frequency vibration generates frictional heat
//                   (very fast cycle, no contamination of seal area)
//   "induction"   — electromagnetic induction through a foil layer
//                   (used for tamper-evident closures and metallised films)
//
//        ┌─────────────┐                ┌─────────────┐
//        │  open bag   │   bag_seal     │ sealed bag  │
//        │             │ ─────────────▶│  (closed at  │
//        │             │ (T,t,modality) │   seal_line) │
//        └─────────────┘                └─────────────┘
//        geometry unchanged ; seal metadata stamped
//
// Signature pattern:
//   pattern.kind             = "bag_seal"
//   pattern.seal_type        = "heat" | "ultrasonic" | "induction"
//   pattern.seal_temp_c      = double
//   pattern.seal_time_s      = double
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             seal_type empty / unknown
//   DFM-SEAL-TIME         seal_time_s ≤ 0
//   DFM-SEAL-TEMP-LOW     seal_temp_c below modality-appropriate minimum (info)
//   DFM-SEAL-TEMP-HIGH    seal_temp_c above modality-appropriate maximum (info)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bag_seal {

constexpr const char* kSkillId = "bag_seal";

struct Input
{
    std::string  seal_type    = "heat";   // "heat" | "ultrasonic" | "induction"
    double       seal_temp_c  = 170.0;
    double       seal_time_s  = 1.5;
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace bag_seal
}  // namespace koocadcam::skill
