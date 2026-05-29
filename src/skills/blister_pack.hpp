#pragma once
// @lat: [[engine/skills#blister_pack]]
//
// blister_pack — thermoformed blister cavity with a lidding film.  The
// product sits in a vacuum-formed plastic pocket (PVC, PETG, or PP) and is
// closed by heat-sealing a lidding sheet (aluminum foil, paper-foil, or
// printable polymer).  Cavity count controls the number of unit doses per
// card (1 for single-dose IV implants; 10–30 for OTC tablets).
//
//        ┌─────────────┐                  ┌──────────────────────┐
//        │  product    │   blister_pack   │  product in N cavity │
//        │             │ ───────────────▶│  blister + lidding    │
//        └─────────────┘                  └──────────────────────┘
//        geometry unchanged ; cavity + lidding metadata stamped
//
// Signature pattern:
//   pattern.kind             = "blister_pack"
//   pattern.cavity_count     = int
//   pattern.lidding_material = string  ("aluminum_foil" | "paper_foil" | "polymer_film")
//   pattern.sealing_temp_c   = double
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             cavity_count ≤ 0 or sealing_temp_c ≤ 0
//   DFM-BLISTER-LID       lidding_material empty
//   DFM-BLISTER-COUNT     cavity_count > 60 (info — unusual high-count card)
//   DFM-BLISTER-TEMP      sealing_temp_c outside [120, 220] °C (info)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace blister_pack {

constexpr const char* kSkillId = "blister_pack";

struct Input
{
    int          cavity_count       = 10;
    std::string  lidding_material   = "aluminum_foil";
    double       sealing_temp_c     = 170.0;
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace blister_pack
}  // namespace koocadcam::skill
