#pragma once
// @lat: [[engine/skills#pcb_card_edge_socket]]
//
// pcb_card_edge_socket — receptacle slot for a PCB card edge connector
// (JEDEC MO-189 / SO-DIMM family).  Compound feature with 3 sub-cuts:
//   1. Rectangular slot pocket    (the main card-edge gap)
//   2. Per-finger contact relief  (N small rectangular pockets running
//      along ONE long wall of the slot at the chosen pitch — these accept
//      the spring contact fingers).  Default N = 22.
//   3. Key tab notch              (one larger notch at a fixed position
//      along the slot — this is the keying feature that ensures the card
//      is inserted in the correct orientation).
//
// Sub-feature count: 3 (slot + finger-array + key tab).
//
//      ┌───────────────────────────────────────────┐
//      │  ◯◯◯◯◯◯◯◯◯ key ◯◯◯◯◯◯◯◯◯◯◯◯              │ ← contact reliefs row
//      │  ────────────────────────────────────     │ ← slot floor
//      └───────────────────────────────────────────┘
//
// DFM (JEDEC MO-189):
//   - pitch_mm ∈ { 1.27, 2.00, 2.54 }
//   - finger_count > 0
//   - finger_dia_mm < pitch_mm × 0.8   (clearance)
//   - slot_width_mm ≥ 0.8 mm
//   - key_tab_offset_mm ∈ (0, slot_length)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pcb_card_edge_socket {

constexpr const char* kSkillId = "pcb_card_edge_socket";

struct Input
{
    FaceDatum   entry_face;
    double      start_x_mm        = 0.0;   // slot start on the entry face
    double      start_y_mm        = 0.0;
    double      end_x_mm          = 0.0;   // slot end
    double      end_y_mm          = 0.0;
    gp_Dir      axis_dir          { 0.0, 0.0, -1.0 };
    double      slot_width_mm     = 1.5;   // PCB card thickness clearance
    double      slot_depth_mm     = 5.0;   // depth into the socket body
    int         finger_count      = 22;    // number of contact reliefs
    double      pitch_mm          = 2.54;  // 2.54 (DIMM) or 1.27 (SO-DIMM)
    double      finger_dia_mm     = 1.5;   // contact finger relief diameter
    double      finger_depth_mm   = 4.5;   // depth of each relief
    double      key_tab_width_mm  = 2.0;   // notch width (along slot)
    double      key_tab_depth_mm  = 1.5;   // notch depth (perpendicular)
    double      key_tab_offset_mm = 15.0;  // position from slot start
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pcb_card_edge_socket
}  // namespace koocadcam::skill
