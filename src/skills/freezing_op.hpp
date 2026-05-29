#pragma once
// @lat: [[engine/skills#freezing_op]]
//
// freezing_op — rapid freezing of food (IQF, blast freezing).
//
//   Drop product temperature below the freezing point quickly enough to
//   form small ice crystals that preserve cell structure and texture.
//   IQF (Individual Quick Freezing) treats particulate streams; blast
//   freezing handles bulk packs and trays.  Geometry is treated as
//   unchanged at the CAD/CAM layer; the stamp records the freezing
//   recipe so cold-chain documentation can be replayed.
//
//        ┌─────────────┐                ┌─────────────┐
//        │ chilled     │   freezing_op  │ frozen      │
//        │  product    │  ──────────▶  │  product    │
//        └─────────────┘                └─────────────┘
//        geometry unchanged ; core temperature ≤ final_temp_c
//
// Signature pattern:
//   pattern.kind              = "freezing_op"
//   pattern.final_temp_c      = double  (typical -40 … -18 °C)
//   pattern.freezing_time_min = double  (cycle minutes)
//   pattern.process_family    = "food_processing"
//   pattern.geometric_change  = false
//
// DFM:
//   DFM-INPUT             final_temp_c ≥ 0 °C (must be sub-zero)
//   DFM-FREEZE-TIME       freezing_time_min outside (0, 720] minutes
//   DFM-FREEZE-TEMP-LOW   final_temp_c < -80 °C (info — below practical
//                         industrial range)
//   DFM-FREEZE-SLOW       freezing_time_min > 240 (info — slow freezing
//                         risks large ice crystals + texture damage)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace freezing_op {

constexpr const char* kSkillId = "freezing_op";

struct Input
{
    double  final_temp_c      = -25.0;   // °C (must be < 0)
    double  freezing_time_min = 30.0;    // minutes
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace freezing_op
}  // namespace koocadcam::skill
