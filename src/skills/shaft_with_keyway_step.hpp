#pragma once
// @lat: [[engine/skills#shaft_with_keyway_step]]
//
// shaft_with_keyway_step — COMPOUND macro that finishes a turned shaft
// with secondary milling/grooving operations:
//
//   sub-cuts (3):
//     1. step (turn-down) from full_dia to step_dia    (annular ring removal
//                                                       at the +Z end)
//     2. sled keyway on the large-Ø section            (rect slot, axis-aligned)
//     3. retaining-ring snap groove at snap_ring_z     (circumferential
//                                                       annular groove)
//
// Common shaft anatomy: small bearing-seat at +Z, large drive section
// further down, keyway for power transfer, snap-ring groove to lock the
// bearing axially.
//
// DFM gates (DIN 471 snap ring + DIN 6885 key):
//   DFM-INPUT, DFM-002
//   DFM-SHAFT-STEP    step_dia >= full_dia
//   DFM-SHAFT-STEPZ   step_z   <= 0 OR >= shaft_length
//   DFM-SHAFT-SNAPZ   snap_ring_z outside shaft span
//   DFM-KEYWAY-FIT    key_w >= step_dia (key too wide for shaft)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace shaft_with_keyway_step {

constexpr const char* kSkillId = "shaft_with_keyway_step";

struct Input
{
    double full_dia_mm        = 0.0;   // shaft large Ø
    double shaft_length_mm    = 0.0;   // stock total length (Z extent)
    double step_z_mm          = 0.0;   // Z where the turn-down begins (from 0)
    double step_dia_mm        = 0.0;   // small Ø at the +Z end of shaft
    double key_w_mm           = 0.0;
    double key_h_mm           = 0.0;
    double key_len_mm         = 0.0;
    double key_center_z_mm    = 0.0;   // Z center of the sled keyway
    double snap_ring_z_mm     = 0.0;   // Z center of the snap-ring groove
    double snap_ring_w_mm     = 1.2;   // axial width of snap-ring groove
    double snap_ring_depth_mm = 0.5;   // radial depth of snap-ring groove
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace shaft_with_keyway_step
}  // namespace koocadcam::skill
