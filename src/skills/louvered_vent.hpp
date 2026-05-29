#pragma once
// @lat: [[engine/skills#louvered_vent]]
//
// louvered_vent — COMPOUND, SPEC-DRIVEN feature
//
// Cuts N angled (tilted) slots in a face, forming a directional louvered
// vent that admits airflow while shedding rain / blocking ingress at angle.
//
//         ┌──────────────────────────────────┐
//         │  ╲──╲   ╲──╲   ╲──╲   ╲──╲   ╲──╲│ ← N tilted slots
//         │   ╲──╲   ╲──╲   ╲──╲   ╲──╲   ╲──╲   tilt_angle ° from face
//         └──────────────────────────────────┘
//
// Each slot is a thin box rotated about the slot_axis (in-plane) by
// tilt_angle so the cut faces a downward / sideways direction.
//
// Spec library (industry louver families):
//   "L-15" : tilt_angle=15°, slot_length=20 mm, slot_width=3 mm, pitch=6 mm   (std)
//   "L-30" : tilt_angle=30°, slot_length=30 mm, slot_width=4 mm, pitch=8 mm   (high airflow)
//   "L-45" : tilt_angle=45°, slot_length=40 mm, slot_width=5 mm, pitch=10 mm  (max airflow)
//
// Compound chain:
//   for i in 0..N-1:    build_tilted_slot_box(i)  (thin angled box)
//   fuse_many           → single cutter compound
//   cut                 → subtract from workpiece (single Boolean cut)
//
// DFM gates:
//   DFM-LOUVER-SP   : unknown spec key.
//   DFM-LOUVER-N    : louver_count < 2.
//   DFM-LOUVER-ANG  : tilt_angle outside [5°, 75°] — flat ≈ slot, near-90° ≈ wall.
//   DFM-LOUVER-FA   : projected free area < 50 % of bbox face.
//   DFM-LOUVER-FIT  : array span > face bbox dim.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace louvered_vent {

constexpr const char* kSkillId = "louvered_vent";

struct Input
{
    FaceDatum   face;
    int         louver_count       = 0;
    std::string spec_key;                   // "L-15" | "L-30" | "L-45" | ""
    // Optional overrides used when spec_key is empty.
    double      tilt_angle_deg     = 0.0;
    double      slot_length_mm     = 0.0;
    double      slot_width_mm      = 0.0;
    double      pitch_mm           = 0.0;
    // Slot long-axis (about which louvers tilt).  Defaults to +X.
    gp_Dir      slot_axis          { 1.0, 0.0, 0.0 };
    double      center_x_mm        = 0.0;
    double      center_y_mm        = 0.0;
    double      wall_thickness_mm  = 0.0;   // defaults to 3 mm
};

struct LouverSpec {
    double tilt_angle_deg = 0.0;
    double slot_length_mm = 0.0;
    double slot_width_mm  = 0.0;
    double pitch_mm       = 0.0;
};

LouverSpec louverSpecFor(const std::string& key);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace louvered_vent
}  // namespace koocadcam::skill
