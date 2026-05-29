#pragma once
// @lat: [[engine/skills#upsetting]]
//
// upsetting — axial compression of a billet between flat dies; the length
// decreases and the cross-section grows in a volume-preserving manner.
// Used for fastener heads (bolt-head upset), bearing seats, gear blanks,
// and as a preparatory step before closed-die forging.
//
//        ┌──┐                       ┌──────────┐
//        │  │                       │          │
//        │L0│  upper die ↓          │   L1     │
//        │  │   ───────────────►    │ (shorter,│
//        │  │  lower die ↑          │ wider)   │
//        │  │                       │          │
//        └──┘                       └──────────┘
//
// Synthesis strategy:
//   Volume preservation: A0·L0 = A1·L1 ⇒ A1 = A0 · L0/L1 (= A0 · cratio).
//   The dominant axis is the smallest bbox axis only if the stock is
//   cuboid-like; for KooCADCAM upsetting we always treat **Z** as the
//   compression axis and assume a CUBOID stock (rebuild as new box):
//
//     L1 = L0 / cratio    (where cratio = compression_ratio = L0 / L1)
//     A_lat_new = A_lat_old · cratio
//
//   In slice-1 we keep the X/Y aspect ratio of the original bbox and scale
//   both XY extents uniformly by √cratio so the new cross-section area
//   equals the target.  Volume is preserved within float tolerance.
//
// Signature pattern:
//   - kind                = "upsetting"
//   - compression_ratio    (L0 / L1; must be > 1.0)
//   - original_length_mm  / final_length_mm
//   - lateral_scale        (= √cratio)
//
// DFM checks:
//   DFM-INPUT       : compression_ratio ≤ 1.0; degenerate bbox — error
//   DFM-UPS-BUCKLE  : L0 / D0 > 2.5  AND  cratio > 1.5 — error
//                     (slender column buckles before upsetting)
//   DFM-UPS-HEAVY   : compression_ratio > 3.0 — info (multi-stage typical)
//
// Recognize:
//   Metadata replay; otherwise pure cuboid → no geometric tell.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace upsetting {

constexpr const char* kSkillId = "upsetting";

struct Input
{
    double  compression_ratio = 2.0;   // L0 / L1; > 1.0
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace upsetting
}  // namespace koocadcam::skill
