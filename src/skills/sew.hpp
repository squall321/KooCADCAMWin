#pragma once
// @lat: [[engine/skills#sew]]
//
// sew — record-only stitching operation joining cut pattern pieces.
// Stitch geometry (polyline through fabric) is sub-OCCT — no Boolean is
// performed.  This skill stamps the sew-record signature so MES, quality
// control, and cost planning can downstream verify stitch density and
// thread cost.
//
// Signature pattern:
//   - pattern.kind             = "sew"
//   - pattern.stitch_type      = "lockstitch" | "chainstitch" | "overlock" | …
//   - pattern.stitches_per_in  = <input>
//   - pattern.thread_color     = <input>
//   - pattern.geometry_changed = false
//
// DFM checks:
//   stitch_type       non-empty             (DFM-INPUT error)
//   stitches_per_in   ∈ [4, 30]             (DFM-SEW-DENSITY warning if outside)
//   stitches_per_in   > 0                   (DFM-INPUT error)
//   thread_color      non-empty             (DFM-INPUT warning)
//
// Synthesis:
//   NO geometric change — clone workpiece and stamp the signature.
//
// Recognize:
//   Metadata-only replay from `workpiece.features()`.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sew {

constexpr const char* kSkillId = "sew";

struct Input
{
    std::string  stitch_type      = "lockstitch";   // lockstitch | chainstitch | overlock | zigzag
    double       stitches_per_in  = 12.0;           // SPI (typical apparel: 8-14)
    std::string  thread_color     = "black";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sew
}  // namespace koocadcam::skill
