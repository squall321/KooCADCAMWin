#pragma once
// @lat: [[engine/skills#plunge_milling]]
//
// plunge_milling — vertical-only end-mill plunge cut.
//
// A simple downward plunge cut that uses an END-MILL (flat bottom), not a
// twist drill (cone tip).  Distinct from drill_hole (which leaves a conical
// tool-tip bottom) and mill_circular_pocket (which sweeps laterally after
// the plunge).  Plunge milling drops straight down with no lateral motion
// — the bottom-cutting flutes do all the work.
//
//                       entry_face (planar)
//                            │
//               ─────────────┼──────────────
//                            │     ▼ axis
//                            │
//                 ┌──────────┴──────────┐    ↑ depth_mm
//                 │  (flat bottom)      │    │
//                 └─────────────────────┘    ↓
//                            ↑
//                           dia_mm
//
// Geometrically identical to `mill_circular_pocket` with
// `bottom_corner_r_mm = 0`.  Differs from drill_hole in the bottom shape:
// flat (end-mill) vs cone (twist drill).  The differentiation lives in
// `signature.pattern.kind` ("plunge_milling") and `tooling.cutting_mode`.
//
// Signature pattern:
//   - 1 cylindrical face (the bore wall)
//   - 1 planar face (flat bottom)
//   - 1 circular edge at entry, 1 at bottom
//
// DFM checks:
//   DFM-002 : dia_mm  ≥ 0.8 mm
//   DFM-PLUNGE-AR : depth/dia ≤ 3.0
//          (above this the plunge gets risky — chip evacuation, axial load,
//          tool deflection.  Pure plunging cannot exceed this safely; use
//          drill_hole for deep narrow holes.)
//
// Recognize:
//   Same topology as drill_hole / mill_circular_pocket (1 cyl + 2 circles +
//   flat bottom).  Differentiate by aspect ratio: plunge cuts have
//   depth/dia ≤ 3.  We claim recognition when the bottom is flat (= π r²)
//   AND depth/dia ≤ 3.  Confidence ~0.7 — overlaps with mill_circular_pocket.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace plunge_milling {

constexpr const char* kSkillId = "plunge_milling";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    double      dia_mm        = 0.0;
    double      depth_mm      = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace plunge_milling
}  // namespace koocadcam::skill
