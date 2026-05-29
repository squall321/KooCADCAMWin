#pragma once
// @lat: [[engine/skills#din_rail_mount_slot]]
//
// din_rail_mount_slot — COMPOUND MECHANICAL FEATURE for a DIN 35 standard
// rail (top-hat 35 mm × 7.5 mm) mounting receiver milled into a plate.
//
//   ┌─ top of plate (entry_face)
//   ▼
//   ────────┬─────────────────────────────────────────┬───────
//           │  ┌────────────────────────────────────┐ │      ↑
//           │  │ top rail receiver groove (DIN 35)  │ │      │ rcv_d (7.5)
//           │  │ width = 35 mm, depth = 7.5 mm     │ │      ↓
//           │  └────────────────────────────────────┘ │
//           │                                          │
//           │  ┌─◯─────────────────────────────────◯─┐ │ ← 2 × M4 retention holes
//           │  │                                     │ │
//           │  │ bottom spring-finger relief slot   │ │      ↑
//           │  │ width = 27 mm, depth = 1.5 mm      │ │      │ fin_d
//           │  └─────────────────────────────────────┘ │      ↓
//           └─────────────────────────────────────────────────
//
// Sub-features (4):
//   1. Top rail receiver groove (rectangular slot): 35 mm wide × 7.5 mm deep,
//      length = rail_length.
//   2. Bottom spring-finger relief slot (parallel below it, narrower):
//      27 mm wide × 1.5 mm deep — gives the spring-loaded foot of the DIN
//      rail clip somewhere to flex into.
//   3. Two M4 retention through holes at 25 mm spacing along the slot
//      length (rail end retention).
//   4. (counted as part of subfeature_count) The pair of holes is
//      enumerated as 2 sub-features so subfeature_count = 1 + 1 + 2 = 4.
//
// Input:
//   - face_id      : entry face (top of plate)
//   - rail_dir     : axis direction the rail runs along (+X or +Y)
//   - rail_length  : how long the seat groove is (mm)
//
// DFM:
//   DFM-DIN35  rail_length < 60 mm (must hold ≥ 2 retention holes + flange)
//   DFM-DIN35  rail_length > 500 mm (single-piece milling impractical)
//   DFM-INPUT  rail_dir not aligned with ±X or ±Y in slice 1
//
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace din_rail_mount_slot {

constexpr const char* kSkillId = "din_rail_mount_slot";

struct Input
{
    FaceDatum  entry_face;
    double     start_x_mm   = 0.0;   // start of the rail seat (center-line)
    double     start_y_mm   = 0.0;
    gp_Dir     rail_dir { 1.0, 0.0, 0.0 };  // rail runs along +X by default
    gp_Dir     axis_dir { 0.0, 0.0, -1.0 }; // milling axis (into plate)
    double     rail_length_mm = 0.0;
    double     plate_t_mm     = 10.0;  // plate thickness (for M4 through holes)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace din_rail_mount_slot
}  // namespace koocadcam::skill
