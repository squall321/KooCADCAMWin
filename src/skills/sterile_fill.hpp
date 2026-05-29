#pragma once
// @lat: [[engine/skills#sterile_fill]]
//
// sterile_fill — aseptic liquid filling of sterile injectable / parenteral
// containers (vials, syringes, ampoules) in an ISO-classified cleanroom
// or isolator environment.
//
//   sterile drug         aseptic fill            filled & stoppered
//  ┌────────────┐    ┌──────────────────┐      ┌────────────────────┐
//  │ filtered   │ → │ dose into vial /  │  →  │ stoppered, capped   │
//  │ solution   │    │ syringe in ISO 5 │      │ sterile container   │
//  └────────────┘    └──────────────────┘      └────────────────────┘
//   Container geometry sub-mm — modeled only via metadata signature.
//
// Signature pattern:
//   pattern.kind              = "sterile_fill"
//   pattern.fill_volume_ml    = double  (mL per container)
//   pattern.container_type    = "vial" | "syringe" | "ampoule" | "cartridge"
//   pattern.env_class         = "ISO_5" | "ISO_7"
//   pattern.geometric_change  = false
//
// DFM:
//   DFM-INPUT             fill_volume_ml ≤ 0 (error)
//   DFM-SF-ENV            env_class not "ISO_5" or "ISO_7" (error — aseptic
//                          fill requires classified environment)
//   DFM-SF-CONTAINER      container_type not recognized (info)
//   DFM-SF-ENV-CLASS      env_class == "ISO_7" (info — Annex 1 requires
//                          ISO 5 at point of fill for terminally non-sterilized)
//   DFM-SF-VOLUME-LARGE   fill_volume_ml > 500 (info — atypical for unit-dose)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sterile_fill {

constexpr const char* kSkillId = "sterile_fill";

struct Input
{
    double       fill_volume_ml = 5.0;        // mL per container
    std::string  container_type = "vial";     // "vial" | "syringe" | "ampoule" | "cartridge"
    std::string  env_class      = "ISO_5";    // "ISO_5" | "ISO_7"
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace sterile_fill
}  // namespace koocadcam::skill
