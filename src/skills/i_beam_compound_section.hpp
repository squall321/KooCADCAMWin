#pragma once
// @lat: [[engine/skills#i_beam_compound_section]]
//
// i_beam_compound_section — COMPOUND structural-member skill that REBUILDS
// the workpiece as an I-section (W-shape) by FUSING three boxes:
//
//   sub-feature 1: top flange   (length × flange_w × flange_t)
//   sub-feature 2: central web  (length × web_t    × (height - 2·flange_t))
//   sub-feature 3: bottom flange(length × flange_w × flange_t)
//
//                ┌──────────────────────┐ ▲
//                │      top flange      │ │ flange_t
//                └─────────┬────┬───────┘ ▼
//                          │ web│         ▲
//                          │ ←─→│         │ height - 2·flange_t
//                          │web_t│        │
//                          ├────┤         │
//                ┌─────────┴────┴───────┐ ▼
//                │     bot flange       │ ▲
//                └──────────────────────┘ ▼ flange_t
//                ←─────── flange_w ──────→
//
// Total height = 2 × flange_t + web_height = `height`.
// Length runs along +X.  Section sits centered on (Y,Z) origin.
//
// Implementation strategy:
//   1. Build three pr::box() shapes positioned so they fuse along common faces.
//   2. pr::fuseMany() to a single TopoDS_Compound, then attach to the
//      workpiece (replacing the stock — this skill REBUILDS).
//   3. Stamp ONE FeatureSignature whose pattern carries kind / is_compound /
//      subfeature_count + all geometric parameters.
//
// DFM (AISC W-shape, ISO 657-15 hot-rolled):
//   DFM-IBEAM-INPUT     : any non-positive dimension
//   DFM-IBEAM-WEB-PROP  : web height / web_t > 100  (slender web buckles)
//   DFM-IBEAM-FLANGE    : (flange_w/2 - web_t/2) / flange_t > 8  (slender flange)
//   DFM-IBEAM-GEOM      : flange_t × 2 >= height (web disappears)
//
// Recognize: metadata replay (compound features are hard to fingerprint
// geometrically); confidence 1.0 when in feature history, otherwise empty.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace i_beam_compound_section {

constexpr const char* kSkillId = "i_beam_compound_section";

struct Input
{
    double length_mm   = 1000.0;
    double height_mm   = 200.0;
    double flange_w_mm = 150.0;
    double flange_t_mm = 10.0;
    double web_t_mm    = 6.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace i_beam_compound_section
}  // namespace koocadcam::skill
