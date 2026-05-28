#pragma once
// @lat: [[engine/skills#paint_op]]
//
// paint_op — liquid paint application.  A spray-applied paint film is
// deposited on the target face (typical wet film 10 – 500 μm), then
// cured (air-dry or oven cure).
//
//        ┌─────────────┐
//        │ ▓▓▓▓▓▓▓▓▓▓▓ │   ← paint film (10 – 500 μm)
//        │             │
//        │  (substrate)│   ← workpiece (unchanged)
//        │             │
//        └─────────────┘
//
// Signature pattern:
//   - pattern.kind         = "paint_op"
//   - pattern.target_face_id
//   - pattern.paint_type   = "wet" (only; "powder" → see powder_coat)
//   - pattern.color
//   - pattern.thickness_um
//   - pattern.cure_temp_c
//   - pattern.cure_time_min
//   - geometry IDENTICAL to input
//
// DFM checks:
//   thickness_um ∈ [10, 500]      (DFM-PAINT-THK error if outside)
//   cure_temp_c ∈ [10, 200]       (DFM-PAINT-CURE info if outside typ range)
//   cure_time_min ≥ 1             (info)
//   paint_type == "powder"        (DFM-PAINT-TYPE warning — route to powder_coat)
//
// Synthesis:
//   NO geometric change.  Returns a clone with the signature stamped.
//
// Recognize:
//   Impossible from B-rep.  Metadata-only replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace paint_op {

constexpr const char* kSkillId = "paint_op";

struct Input
{
    FaceDatum   entry_face;
    std::string paint_type    = "wet";       // "wet" only; "powder" warns
    std::string color         = "black";
    double      thickness_um  = 50.0;
    double      cure_temp_c   = 80.0;        // mid-bake epoxy / acrylic
    double      cure_time_min = 20.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace paint_op
}  // namespace koocadcam::skill
