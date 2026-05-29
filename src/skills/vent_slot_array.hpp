#pragma once
// @lat: [[engine/skills#vent_slot_array]]
//
// vent_slot_array — COMPOUND, SPEC-DRIVEN feature
//
// Cuts N elongated through slots in a planar face for ventilation.
//
//             ┌──────────────────────────────────────────┐
//             │                                            │
//             │ ████  ████  ████  ████  ████  ████  ████ │  ← N slots
//             │                                            │
//             └──────────────────────────────────────────┘
//                ←slot_len→    ←pitch→
//                ←▒w▒  ← slot_width
//
// Spec library:
//   "VS-S"  : slot_length=20 mm, slot_width=2 mm, pitch=4 mm   (small enclosure)
//   "VS-M"  : slot_length=40 mm, slot_width=3 mm, pitch=6 mm   (medium)
//   "VS-L"  : slot_length=60 mm, slot_width=4 mm, pitch=8 mm   (large)
//
// Compound chain:
//   for i in 0..N-1:    build_slot_box(i)   (rectangular cutter through wall)
//   fuse_many           → single cutter compound
//   cut                 → subtract from workpiece (single Boolean cut)
//
// DFM gates:
//   DFM-VENT-SP   : unknown spec key.
//   DFM-VENT-N    : slot_count < 2.
//   DFM-VENT-FA   : free area / face bbox area < 50 % → fails airflow gate.
//                   (Vent purpose is airflow; ≥ 50 % open area is the
//                    industry rule for cooling enclosures.)
//   DFM-VENT-FIT  : N · pitch > face bbox length along slot axis.
//   DFM-VENT-WIDTH: slot_width < 0.8 mm (DFM-002 derived).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace vent_slot_array {

constexpr const char* kSkillId = "vent_slot_array";

struct Input
{
    FaceDatum   face;
    int         slot_count       = 0;
    std::string spec_key;                  // "VS-S" | "VS-M" | "VS-L" | ""
    // Optional overrides used when spec_key is empty.
    double      slot_length_mm   = 0.0;
    double      slot_width_mm    = 0.0;
    double      pitch_mm         = 0.0;
    // Slot long-axis direction (in face plane).  Defaults to +X.
    gp_Dir      slot_axis        { 1.0, 0.0, 0.0 };
    // Center of the slot ARRAY (XY of the face).
    double      center_x_mm      = 0.0;
    double      center_y_mm      = 0.0;
    // Wall thickness through which slots are cut.  Defaults to 3 mm.
    double      wall_thickness_mm = 0.0;
};

struct SlotSpec {
    double slot_length_mm = 0.0;
    double slot_width_mm  = 0.0;
    double pitch_mm       = 0.0;
};

SlotSpec slotSpecFor(const std::string& key);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace vent_slot_array
}  // namespace koocadcam::skill
