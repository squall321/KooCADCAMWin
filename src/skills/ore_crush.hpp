#pragma once
// @lat: [[engine/skills#ore_crush]]
//
// ore_crush — comminution stage that reduces ROM ore from quarry feed size to
// a target P80 product size by jaw, cone, gyratory, or impact crusher.  The
// workpiece geometry in KooCADCAM is unchanged — comminution targets a bulk
// size distribution that lives outside the B-rep — but the size-reduction
// ratio and crusher selection are stamped so downstream flotation / leach
// stages can replay the upstream conditions.
//
//        ▓▓▓▓▓ (feed_size_mm)  →   ▓▓▓ (target_p80_mm)
//                            crusher
//
// Signature pattern:
//   pattern.kind             = "ore_crush"
//   pattern.feed_size_mm     = double
//   pattern.target_p80_mm    = double
//   pattern.crusher_type     = "jaw" | "cone" | "gyratory" | "impact"
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             feed_size_mm ≤ 0 or target_p80_mm ≤ 0
//   DFM-CRUSHER-TYPE      crusher_type not in {jaw, cone, gyratory, impact}
//   DFM-CRUSH-RATIO       target_p80_mm ≥ feed_size_mm (no reduction)
//   DFM-CRUSH-RATIO-HIGH  feed/target > 12 (single stage cannot achieve, info)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ore_crush {

constexpr const char* kSkillId = "ore_crush";

struct Input
{
    double      feed_size_mm   = 800.0;   // ROM / primary feed
    double      target_p80_mm  = 100.0;
    std::string crusher_type   = "jaw";   // "jaw" | "cone" | "gyratory" | "impact"
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ore_crush
}  // namespace koocadcam::skill
