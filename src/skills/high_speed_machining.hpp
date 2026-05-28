#pragma once
// @lat: [[engine/skills#high_speed_machining]]
//
// high_speed_machining — HSM strategy parameters.  Metadata wrapper around
// a conventional milling operation; the geometry result is identical to
// `mill_rect_pocket` / `mill_circular_pocket` but the strategy uses very
// high spindle RPM (10 000 – 60 000 rpm) and small radial engagement so
// chip thinning makes the SFM × feed product very high.
//
// Distinguishing properties vs conventional milling:
//   - tool_type        = "end_mill_hsm"
//   - cutting_speed_sfm ≈ 1500 – 3000  (vs 400 – 700 conventional)
//   - feed_per_tooth_mm ≈ 0.04 – 0.10  (vs 0.02 – 0.03 conventional)
//   - radial_engagement_ratio ≤ 0.10  (chip-thinning regime)
//   - axial_doc usually = full flute length
//
// This skill is METADATA-ONLY: the wrapped sub-skill (drill / mill / pocket)
// produces the geometry; we just stamp the HSM strategy + tooling info.
//
// Signature pattern:
//   - pattern.kind                    = "high_speed_machining"
//   - pattern.sub_skill_id            = "mill_rect_pocket" | …
//   - pattern.spindle_rpm
//   - pattern.radial_engagement_ratio
//   - pattern.feed_per_tooth_mm
//   - pattern.axial_doc_mm
//   - tooling.tool_type = "end_mill_hsm"
//
// DFM checks:
//   DFM-INPUT       : sub_skill_id empty → error
//   DFM-HSM-RPM     : spindle_rpm ∉ [10000, 60000] → error
//   DFM-HSM-RAE     : radial_engagement_ratio > 0.10 → error
//                     (chip thinning lost — conventional regime)
//   DFM-HSM-RAE     : radial_engagement_ratio ≤ 0 → error
//   DFM-HSM-FEED    : feed_per_tooth_mm ∉ [0.01, 0.20] → warning

#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace high_speed_machining {

constexpr const char* kSkillId = "high_speed_machining";

struct Input
{
    std::string  sub_skill_id              = "";       // "mill_rect_pocket" | …
    double       spindle_rpm               = 24000.0;  // [10000, 60000]
    double       radial_engagement_ratio   = 0.05;     // ae/D, ≤ 0.10
    double       feed_per_tooth_mm         = 0.06;
    double       axial_doc_mm              = 0.0;      // 0 = full flute
    double       tool_dia_mm               = 4.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace high_speed_machining
}  // namespace koocadcam::skill
