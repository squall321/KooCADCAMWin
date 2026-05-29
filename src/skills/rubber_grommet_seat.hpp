#pragma once
// @lat: [[engine/skills#rubber_grommet_seat]]
//
// rubber_grommet_seat — COMPOUND MECHANICAL FEATURE for a panel hole that
// hosts a rubber wiring grommet (SAE / DIN 89280 / VDE 0470 panel grommets).
//
//                    ┌─ entry_face (planar, top of plate)
//                    ▼
//              ──────┬───────────────────┬──────
//                    │chamfer_top (45°)  │      ↑ chamfer_d
//                    │ ╲             ╱   │      ↓
//                    │  ╲           ╱    │
//                    │   │         │     │      ↑
//                    │   │         │     │      │
//                    │   │  retention   │      │
//                    │   │  groove (annular)   │ plate_t  (through)
//                    │   │  ╳         ╳   │     │
//                    │   │         │     │      │
//                    │   │         │     │      │
//                    │   │  ╱     ╲     │      │
//                    │   ╳        ╳     │      ↓ chamfer_bot (lead-in for wires)
//                    └───┴─────────┴─────┘
//
// Sub-features (3):
//   1. Through hole at hole_dia (the grommet through-bore).
//   2. Top chamfer of 45° × chamfer_d (chamfer_d = hole_dia × 0.08, per
//      DIN 89280 grommet-seat best practice; chamfer must be inside the
//      grommet flange OD).
//   3. Inside retention groove — an annular cut at mid-depth, groove_od =
//      hole_dia + 2 × 0.4 mm (groove_depth = 0.4 mm, groove_width = 1.0 mm).
//      The grommet's retention lip snaps into this groove.
//
// Plus a lead-in chamfer on the inside top of the hole (45° × 0.3 mm) for
// the wires.
//
// Input:
//   - face_id  : entry face (top of plate)
//   - position : axis center XY
//   - hole_dia : grommet through-bore diameter (mm)
//   - plate_t  : plate thickness (mm) — used to position the retention groove
//                at mid-depth
//
// DFM:
//   DFM-002      hole_dia ≥ 0.8 mm
//   DFM-INPUT    plate_t ≤ 0
//   DFM-GROMMET  hole_dia < 3 mm (too small to host a standard grommet)
//   DFM-GROMMET  plate_t < 1.2 mm (no room for retention groove at mid-depth)
//
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rubber_grommet_seat {

constexpr const char* kSkillId = "rubber_grommet_seat";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir { 0.0, 0.0, -1.0 };

    double      hole_dia_mm = 0.0;
    double      plate_t_mm  = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rubber_grommet_seat
}  // namespace koocadcam::skill
