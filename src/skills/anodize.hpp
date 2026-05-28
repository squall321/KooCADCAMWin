#pragma once
// @lat: [[engine/skills#anodize]]
//
// anodize — electrochemical surface treatment for aluminum.  Grows a thin
// porous aluminum-oxide layer on the target face (typically 5 – 50 μm
// thick), optionally dyed and sealed.
//
//        ┌─────────────┐
//        │ ▓▓▓▓▓▓▓▓▓▓▓ │   ← Al₂O₃ layer (5 – 50 μm, colored if dyed)
//        │             │
//        │   (Al alloy │   ← base aluminum substrate (unchanged)
//        │    base)    │
//        └─────────────┘
//
// Signature pattern:
//   - pattern.kind                = "anodize"
//   - pattern.target_face_id      = face that was coated
//   - pattern.coating_thickness_um
//   - pattern.color               = "natural" | "black" | "gold" | "red" | …
//   - pattern.seal_type           = "hot_water" | "nickel_acetate" | "none"
//   - geometry IDENTICAL to input
//
// DFM checks:
//   coating_thickness_um ∈ [5, 75]      (DFM-ANODIZE-THK error if outside)
//   color in known palette              (DFM-ANODIZE-COLOR info if unknown)
//   seal_type in {hot_water, nickel_acetate, none}  (info)
//   Workpiece material is aluminum-family            (info)
//
// Synthesis:
//   NO geometric change — clones the workpiece with the signature stamped.
//
// Recognize:
//   Impossible from B-rep (oxide layer is sub-OCCT).  Returns ONLY signatures
//   filtered from `workpiece.features()`, confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace anodize {

constexpr const char* kSkillId = "anodize";

struct Input
{
    FaceDatum   entry_face;
    double      coating_thickness_um = 25.0;     // typical Type II thickness
    std::string color                = "natural";
    std::string seal_type            = "hot_water";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace anodize
}  // namespace koocadcam::skill
