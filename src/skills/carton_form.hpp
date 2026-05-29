#pragma once
// @lat: [[engine/skills#carton_form]]
//
// carton_form — form a folded paperboard carton (secondary packaging).  A
// pre-creased blank is run through a carton-erector that folds along the
// score lines and bonds the closure flaps with hot-melt glue, cold glue,
// or self-locking tabs.  Three standard styles are common: reverse-tuck-end
// (RTE), straight-tuck-end (STE), and auto-lock-bottom (ALB).
//
//        ┌─────────────┐                  ┌─────────────┐
//        │  flat blank │   carton_form    │  3D carton  │
//        │             │ ───────────────▶│  (W×H×D)    │
//        │             │ (fold + glue)    │             │
//        └─────────────┘                  └─────────────┘
//        geometry unchanged ; carton metadata stamped
//
// Signature pattern:
//   pattern.kind             = "carton_form"
//   pattern.carton_size_mm   = [W, H, D] (3-element array)
//   pattern.fold_pattern     = string  ("rte" | "ste" | "alb")
//   pattern.glue_type        = string  ("hot_melt" | "cold_glue" | "tab_lock")
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             carton_size_mm has non-positive dimension /
//                         fold_pattern empty / glue_type empty
//   DFM-CARTON-PATTERN    fold_pattern unknown (info)
//   DFM-CARTON-GLUE       glue_type unknown (info)
//   DFM-CARTON-SIZE       any dimension > 600 mm (info — non-standard size)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace carton_form {

constexpr const char* kSkillId = "carton_form";

struct Input
{
    std::array<double, 3> carton_size_mm  = { 80.0, 120.0, 40.0 }; // W, H, D
    std::string           fold_pattern    = "rte";   // "rte" | "ste" | "alb"
    std::string           glue_type       = "hot_melt"; // "hot_melt" | "cold_glue" | "tab_lock"
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace carton_form
}  // namespace koocadcam::skill
