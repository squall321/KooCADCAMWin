#pragma once
// @lat: [[engine/skills#panel_fastener_dzus_pocket]]
//
// panel_fastener_dzus_pocket — COMPOUND aerospace Dzus quarter-turn
// receptacle pocket (MS27983 / MS27984).  Used on access panels in aircraft
// skin: the rotor stud on the panel cams into the receptacle's spring wire
// after a 90° quarter-turn.  Receptacle is mounted in the FIXED structure;
// this skill cuts the pocket that accepts the receptacle and the two
// opposing key slots that allow the stud's cross-pin to drop through and
// then engage by quarter-turn.
//
//   sub-feature 1: stud bore (cylindrical through-hole, stud_dia)
//   sub-feature 2: receptacle pocket (wider counterbore for receptacle body
//                  housing — receptacle_pocket_dia × receptacle_depth)
//   sub-feature 3: two opposing key slots 90° apart (rectangular slots at
//                  the entry face, length × width, centered on stud, going
//                  to receptacle_depth)
//
// Sequential pr::cut(wp, cutter) loop — receptacle pocket overlaps the stud
// bore concentrically, so we MUST avoid cutMany overlap bug.  Each cut is
// done individually against the running workpiece.
//
// Signature:
//   pattern.kind                   = "panel_fastener_dzus_pocket"
//   pattern.is_compound            = true
//   pattern.aerospace_feature_type = "dzus_quarter_turn_receptacle"
//   pattern.fastener_dia           = stud_dia_mm
//   pattern.derived_volume_removed_mm3
//
// DFM gates:
//   DFM-INPUT         : all dimensions > 0
//   DFM-STUD-DIA      : stud_dia ∈ {6.35, 9.5} mm (1/4", 3/8")
//   DFM-POCKET-WIDTH  : pocket_dia > stud_dia + 1.5 mm
//   DFM-POCKET-DEPTH  : 0 < depth ≤ 8.0 mm
//   DFM-KEY-SLOT      : 0 < slot_width < pocket_dia
//                       slot_length > stud_dia and < pocket_dia × 1.5

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace panel_fastener_dzus_pocket {

constexpr const char* kSkillId = "panel_fastener_dzus_pocket";

struct Input
{
    int    face_id                  = -1;
    double center_x_mm              = 0.0;
    double center_y_mm              = 0.0;
    double stud_dia_mm              = 6.35;   // 6.35 / 9.5
    double receptacle_pocket_dia_mm = 10.0;
    double receptacle_depth_mm      = 3.0;
    double key_slot_width_mm        = 1.6;
    double key_slot_length_mm       = 8.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace panel_fastener_dzus_pocket
}  // namespace koocadcam::skill
