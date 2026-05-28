#pragma once
// @lat: [[engine/skills#powder_coat]]
//
// powder_coat — electrostatic powder application + heat cure.  Dry polymer
// powder is sprayed onto a grounded part, then baked to fuse a uniform
// film (typical 50 – 300 μm) of the chosen color and texture.
//
//        ┌─────────────┐
//        │ ▓▓▓▓▓▓▓▓▓▓▓ │   ← powder film, cured (50 – 300 μm)
//        │             │
//        │  (substrate)│   ← workpiece (unchanged)
//        │             │
//        └─────────────┘
//
// Signature pattern:
//   - pattern.kind          = "powder_coat"
//   - pattern.target_face_id
//   - pattern.color
//   - pattern.thickness_um
//   - pattern.cure_temp_c
//   - pattern.cure_time_min
//   - pattern.texture       = "smooth" | "matte" | "wrinkle" | "metallic"
//   - geometry IDENTICAL to input
//
// DFM checks:
//   thickness_um ∈ [50, 300]            (DFM-PWD-THK error if outside)
//   cure_temp_c ∈ [180, 220]            (DFM-PWD-CURE info if outside)
//   texture in {smooth, matte, wrinkle, metallic}  (DFM-PWD-TEXTURE info)
//
// Synthesis:
//   NO geometric change.  Returns a clone with the signature stamped.
//
// Recognize:
//   Metadata-only replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace powder_coat {

constexpr const char* kSkillId = "powder_coat";

struct Input
{
    FaceDatum   entry_face;
    std::string color         = "black";
    double      thickness_um  = 75.0;
    double      cure_temp_c   = 200.0;     // typical polyester/epoxy ~ 180-220
    double      cure_time_min = 15.0;
    std::string texture       = "smooth";  // see header table
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace powder_coat
}  // namespace koocadcam::skill
