#pragma once
// @lat: [[engine/skills#speaker_grille_slot_array]]
//
// speaker_grille_slot_array — PHONE compound: linear array of speaker slots
// through a face (typically bottom edge of a phone frame).
//
// Sub-features:
//   For i in [0, N):
//     cut rectangular thru-slot of slot_length × slot_width × depth,
//     centred at (center_x + (i − (N-1)/2) × pitch, center_y).
//   All N tools fused into a single pr::cutMany.
//
// DFM:
//   DFM-INPUT          : slot_count > 0, slot_length > 0, slot_width > 0,
//                        pitch_mm > 0, depth_through > 0.
//   DFM-SLOT-OVERLAP   : pitch_mm > slot_width_mm (no overlap)
//   DFM-SLOT-COUNT     : slot_count ∈ [3, 30] typical phone range

#include "Datum.hpp"
#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace speaker_grille_slot_array {

constexpr const char* kSkillId = "speaker_grille_slot_array";

struct Input
{
    int    face_id        = -1;
    double center_x_mm    = 0.0;
    double center_y_mm    = 0.0;
    int    slot_count     = 6;
    double slot_length_mm = 4.0;
    double slot_width_mm  = 0.8;
    double pitch_mm       = 1.5;
    double depth_through  = 1.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace speaker_grille_slot_array
}  // namespace koocadcam::skill
