#pragma once
// @lat: [[engine/skills#knurl]]
//
// knurl — surface-texture operation that imprints a regular pattern (diamond,
// straight, helical) onto a cylindrical region of the workpiece.
//
// **Metadata-only.**  OCCT cannot model the fine knurl helices analytically
// without extreme computation cost (each pitch produces a sub-millimetre
// helical groove).  We record the pattern parameters in the FeatureSignature
// and leave the geometry untouched.  Downstream visualisers / CAM toolpaths
// rebuild the pattern from the metadata.
//
//                ┌─────────────────┐
//                │░░░░░░░░░░░░░░░░░│ ← knurled region annotated
//                │░░░ DIAMOND ░░░░░│    over [start_z, end_z] at radius ≈ R_outer
//                │░░░░░░░░░░░░░░░░░│
//        ────────┘                 └────────
//
// Signature pattern (metadata-only):
//   - kind = knurl, axis = Z, plus pitch/depth/pattern.
//   - geometry_changed = false (purely advisory)
//
// DFM checks:
//   DFM-INPUT  : end_z > start_z, pitch > 0, depth > 0
//   DFM-INPUT  : pattern ∈ { diamond, straight, helical_left, helical_right }
//   DFM-KNURL  : depth_mm ≤ pitch_mm/2 (otherwise helices intersect)
//   DFM-LATHE  : start_z/end_z within stock Z range
//
// Recognize: metadata replay only.  Geometry is unchanged so geometric
//            detection is impossible.

#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace knurl {

constexpr const char* kSkillId = "knurl";

struct Input
{
    double       start_z_mm = 0.0;
    double       end_z_mm   = 0.0;
    double       pitch_mm   = 1.0;    // distance between adjacent ridges
    double       depth_mm   = 0.3;    // radial depth of each ridge
    std::string  pattern    = "diamond";
    // "diamond" | "straight" | "helical_left" | "helical_right"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace knurl
}  // namespace koocadcam::skill
