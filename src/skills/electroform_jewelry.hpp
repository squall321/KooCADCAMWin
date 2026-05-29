#pragma once
// @lat: [[engine/skills#electroform_jewelry]]
//
// electroform_jewelry — build a hollow jewelry piece by electrolytically
// depositing metal onto a sacrificial conductive mandrel.  Used for
// large, lightweight earrings, beads, and pendants where solid casting
// would be too heavy / costly.  After deposition the mandrel is melted
// or chemically dissolved out, leaving a thin metal shell.  Metadata-
// only — the shell B-rep is upstream; this skill stamps the deposition
// record (metal, target wall thickness, mandrel reference).
//
//        ┌─────────────┐                          ┌─────────────┐
//        │  conductive │   electroform_jewelry    │  hollow     │
//        │  mandrel    │ ───────────────────────▶│  shell      │
//        │  (wax+Ag)   │  (metal, thickness, id)  │ (Au/Ag/Cu)  │
//        └─────────────┘                          └─────────────┘
//
// Signature pattern:
//   pattern.kind             = "electroform_jewelry"
//   pattern.deposit_metal    = "gold" | "silver" | "copper" | "nickel"
//   pattern.thickness_um     = double  (target wall thickness in µm)
//   pattern.mandrel_id       = string  (traceable mandrel SKU / serial)
//   pattern.deposit_rate_um_h = double (computed; metal-dependent)
//   pattern.expected_hours   = double
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             metal or mandrel_id empty (error)
//   DFM-DEPOSIT-METAL     metal not in known set (error)
//   DFM-THICKNESS         thickness_um ≤ 0 (error)
//   DFM-THICKNESS-LO      thickness_um < 25 — too thin, shape collapse risk (info)
//   DFM-THICKNESS-HI      thickness_um > 1000 — switch to casting, time prohibitive (info)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace electroform_jewelry {

constexpr const char* kSkillId = "electroform_jewelry";

struct Input
{
    std::string  deposit_metal = "gold";   // gold | silver | copper | nickel
    double       thickness_um  = 150.0;    // target wall thickness, µm
    std::string  mandrel_id    = "MND-0001";
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace electroform_jewelry
}  // namespace koocadcam::skill
