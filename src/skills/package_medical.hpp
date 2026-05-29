#pragma once
// @lat: [[engine/skills#package_medical]]
//
// package_medical — medical-grade barrier packaging per ISO 11607.
//
//   barrier_class:
//     "tyvek_pouch"   → Tyvek/film sterile barrier system (SBS)
//     "foil_blister"  → aluminum foil blister (high moisture barrier)
//     "rigid_tray"    → thermoformed PETG/PE rigid tray + Tyvek lid
//     "double_pouch"  → SBS + protective overpouch (high-risk implants)
//
//        ┌─────────────┐                  ┌──────────────────────┐
//        │ device      │  package_medical │  device sealed in    │
//        │             │ ───────────────▶│  Tyvek pouch          │
//        │             │                  │  + label + ISO 11607  │
//        └─────────────┘                  └──────────────────────┘
//        geometry unchanged ; package metadata stamped
//
// Signature pattern:
//   pattern.kind                 = "package_medical"
//   pattern.barrier_class        = <input>
//   pattern.label_compliance     = "ISO 11607" | "ISO 11607-1" | "ISO 11607-2"
//                                  | "FDA-21-CFR-820" | …
//   pattern.shelf_life_months    = double
//   pattern.geometric_change     = false
//
// DFM:
//   DFM-INPUT              barrier_class empty / unknown
//   DFM-PACKAGE-SHELF      shelf_life_months ∈ (0, 120]  (10-year cap)
//   DFM-PACKAGE-LABEL      label_compliance not in known list (info)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace package_medical {

constexpr const char* kSkillId = "package_medical";

struct Input
{
    std::string  barrier_class       = "tyvek_pouch";
    std::string  label_compliance    = "ISO 11607";
    double       shelf_life_months   = 36.0;
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace package_medical
}  // namespace koocadcam::skill
