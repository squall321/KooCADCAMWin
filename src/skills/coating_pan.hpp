#pragma once
// @lat: [[engine/skills#coating_pan]]
//
// coating_pan — rotary drug-coating pan for film / sugar / enteric coating
// of compressed tablets.  Spray suspension + warm air → dried polymer film.
//
//     tablets in              coating pan                 coated tablets
//   ┌─────────────┐      ┌────────────────────┐         ┌─────────────────┐
//   │   cores     │  →  │  spray + tumble +   │   →    │  film-coated     │
//   │             │      │  warm-air drying    │         │  tablet          │
//   └─────────────┘      └────────────────────┘         └─────────────────┘
//   Geometry NOT modeled at B-Rep layer (sub-mm film, micron-scale).
//
// Signature pattern:
//   pattern.kind              = "coating_pan"
//   pattern.coat_type         = "film" | "enteric" | "sugar"
//   pattern.coat_pickup_mg    = double  (mg of dried film per tablet)
//   pattern.drying_temp_c     = double  (°C inlet/bed temperature)
//   pattern.geometric_change  = false
//
// DFM:
//   DFM-INPUT                 coat_pickup_mg ≤ 0 (error)
//   DFM-INPUT                 drying_temp_c outside (10, 100] (error)
//   DFM-CP-TYPE               coat_type not recognized (info)
//   DFM-CP-PICKUP-HIGH        coat_pickup_mg > 50 (info, excess film)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace coating_pan {

constexpr const char* kSkillId = "coating_pan";

struct Input
{
    std::string  coat_type        = "film";   // "film" | "enteric" | "sugar"
    double       coat_pickup_mg   = 10.0;     // mg of dried coat per tablet
    double       drying_temp_c    = 50.0;     // °C
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace coating_pan
}  // namespace koocadcam::skill
