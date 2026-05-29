#pragma once
// @lat: [[engine/skills#banana_socket_compound]]
//
// banana_socket_compound — electrical 4 mm banana socket (DIN 42220 family).
// Compound feature with 4 distinct sub-cuts on a common axis:
//   1. Cylindrical bore (main contact bore, diameter ≈ 4.0 mm).
//   2. Four axial slits cut through the bore wall, equispaced at 90° — these
//      form the spring fingers that grip the inserted plug pin.
//   3. Retention groove — a shallow annular ring at depth equal to the
//      detent location; gives the plug a tactile "click".
//   4. Insulation step — a wider, shallow counterbore at the entry face
//      that accepts the plug's insulation shroud (per DIN 42220 safety).
//
//                       ┌─ entry_face
//                       ▼
//                   ┌──────────┐                ↑ insulation_step_depth
//                   │   ───    │  ← step (wider)│
//                   ├──┬────┬──┤  ◄── slit      ↓
//                   │  │////│  │                ↑
//                   │  │    │  │  ← bore        │ bore_depth
//                   ├──┤    ├──┤  ← groove      │
//                   │  │////│  │                │
//                   │  └────┘  │                ↓
//                   └──────────┘
//
// Sub-feature count: 4 (bore + 4 slits + groove + step) → compound, ≥2.
//
// DFM (DIN 42220 / IEC 61010 inspired):
//   - bore_dia_mm ∈ [3.8, 4.2]            (banana 4 mm class)
//   - 4 slits (fixed), slit_width ≥ 0.4 mm (laser-cuttable minimum)
//   - retention_groove_depth_mm ≥ 0.2 mm  (must give detect "click")
//   - insulation_step depth ≥ 1.0 mm      (touch-safe per IEC 61010 cl 6.6)
//   - bore_depth_mm > insulation_step_depth_mm

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace banana_socket_compound {

constexpr const char* kSkillId = "banana_socket_compound";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm              = 0.0;
    double      position_y_mm              = 0.0;
    gp_Dir      axis_dir                   { 0.0, 0.0, -1.0 };
    double      bore_dia_mm                = 4.0;     // DIN 42220 banana
    double      bore_depth_mm              = 14.0;    // typical pin insertion
    double      slit_width_mm              = 0.6;     // spring-finger gap
    double      slit_depth_mm              = 10.0;    // axial slit length
    double      retention_groove_dia_mm    = 4.6;     // groove OD (small bump)
    double      retention_groove_depth_mm  = 0.4;     // axial groove width
    double      retention_groove_pos_mm    = 8.0;     // axial pos from entry
    double      insulation_step_dia_mm     = 7.0;     // wider shroud seat
    double      insulation_step_depth_mm   = 2.0;     // depth of step from entry
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace banana_socket_compound
}  // namespace koocadcam::skill
