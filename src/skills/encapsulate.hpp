#pragma once
// @lat: [[engine/skills#encapsulate]]
//
// encapsulate — capsule-filling for pharma solid- or liquid-fill capsules.
// Two-piece hard gelatin / HPMC capsule body + cap filled with powder,
// granule, pellet, or liquid via dosator / tamping / liquid-fill machine.
//
//   bulk fill          encapsulate              filled capsules
//  ┌──────────┐     ┌─────────────────────┐    ┌──────────────┐
//  │  powder  │ →  │  fill body  →  cap  │ → │  closed       │
//  │  pellets │     │  + close + eject     │    │  capsule     │
//  └──────────┘     └─────────────────────┘    └──────────────┘
//   Capsule body/cap are sub-mm pharma geometry; modelled only by metadata.
//
// Signature pattern:
//   pattern.kind              = "encapsulate"
//   pattern.capsule_size      = "0" | "1" | "2" | "3" | "4" | "00" | "000"
//   pattern.fill_weight_mg    = double  (target fill mass per capsule)
//   pattern.fill_type         = "powder" | "pellet" | "granule" | "liquid"
//   pattern.geometric_change  = false
//
// DFM:
//   DFM-INPUT                 fill_weight_mg ≤ 0 (error)
//   DFM-ENC-SIZE              capsule_size not in known set (error)
//   DFM-ENC-TYPE              fill_type not recognized (info)
//   DFM-ENC-OVERFILL          fill_weight_mg > size-specific capacity (info)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace encapsulate {

constexpr const char* kSkillId = "encapsulate";

struct Input
{
    std::string  capsule_size    = "0";        // "000".."4"
    double       fill_weight_mg  = 400.0;      // mg per capsule
    std::string  fill_type       = "powder";   // "powder" | "pellet" | "granule" | "liquid"
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace encapsulate
}  // namespace koocadcam::skill
