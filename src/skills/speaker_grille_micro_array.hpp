#pragma once
// @lat: [[engine/skills#speaker_grille_micro_array]]
//
// speaker_grille_micro_array — PHONE compound: shallow rectangular recess
// (the grille "frame") on a face, plus a dense micro-hole array drilled
// through the bottom of that recess for sound transmission.
//
// Sub-features:
//   1. shallow rectangular pocket  (cut) — grille_length × grille_width ×
//      pocket_depth_mm (≈ hole_dia_mm × 1.5, capped at 0.6 mm).
//   2. micro-hole array            (cut) — N drilled holes through the
//      bottom of the recess. Pattern = "hex" (60-degree staggered) or
//      "grid" (orthogonal).  Pitch = hole_pitch_mm.
//
// DFM:
//   DFM-INPUT      : all dims > 0; pattern ∈ {"hex","grid"}
//   DFM-MIN-HOLE   : hole_dia_mm ≥ 0.3 (drillable on small endmill / EDM)
//   DFM-MIN-WALL   : (hole_pitch_mm − hole_dia_mm) ≥ 0.4 (rib stiffness)
//   DFM-PHONE-RNG  : hole_dia_mm ∈ [0.5, 1.0]; hole_pitch_mm ∈ [1.0, 1.5]
//                    (warning if outside typical phone range)
//
// Signature pattern:
//   pattern.kind                 = "speaker_grille_micro_array"
//   pattern.is_compound          = true
//   pattern.is_phone_feature     = true
//   pattern.phone_feature_type   = "speaker_grille"
//   pattern.hole_count           = N
//   pattern.pattern_type         = "hex" | "grid"
//   pattern.hole_dia_mm          = hole_dia_mm
//   pattern.derived_volume_removed_mm3
//
// Reference: slice-12 fill_pattern style hex/grid generator.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>

namespace koocadcam::skill {

class Workpiece;

namespace speaker_grille_micro_array {

constexpr const char* kSkillId = "speaker_grille_micro_array";

struct Input
{
    int         face_id          = -1;
    double      center_x_mm      = 0.0;
    double      center_y_mm      = 0.0;
    double      grille_length_mm = 8.0;
    double      grille_width_mm  = 3.0;
    double      hole_dia_mm      = 0.8;
    double      hole_pitch_mm    = 1.2;
    std::string pattern          = "hex";   // "hex" | "grid"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace speaker_grille_micro_array
}  // namespace koocadcam::skill
