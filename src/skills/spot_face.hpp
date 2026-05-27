#pragma once
// @lat: [[engine/skills#spot_face]]
//
// spot_face — shallow flat circular pocket creating a FLAT seat for a
// fastener (bolt head, washer) on a non-flat surface (cast, curved, or
// pre-milled rough).
//
// Spot-facing is a specialty operation:
//   - Always shallow (typically 0.5 – 2 mm).
//   - Intent: create a small flat patch perpendicular to the bolt axis on
//     an otherwise irregular surface, so the fastener seats flat.
//   - Tool: usually a cylindrical end-mill with bottom-cutting flutes, or
//     a dedicated spot-face cutter (interchangeable pilot, large flat
//     bottom).
//
//                      entry_face  (original — irregular)
//                            │
//                ────────────┼───────────────
//                            │     ▼ axis
//                            │
//                 ╔══════════╧══════════╗   ↑ depth_mm (shallow!)
//                 ║   (NEW FLAT seat)   ║   │
//                 ╚═════════════════════╝   ↓
//                            ↑
//                          dia_mm
//
// Geometrically identical to mill_circular_pocket; the differentiation
// lives in:
//   - signature.pattern.kind   = "spot_face"
//   - tooling.tool_type        = "spot_face_cutter"
//   - intent: provide a flat seating area, not a real cavity
//
// Signature pattern:
//   - 1 cylindrical face (the side wall, only 0.5-2 mm tall)
//   - 1 planar face (the flat seat)
//   - 2 circular edges (top + bottom)
//
// DFM checks:
//   DFM-002 : dia_mm ≥ 3.0 mm    (smallest spot-face cutter)
//   DFM-SPOTFACE-DEPTH : depth_mm ∈ [0.3, 5.0]
//                       (shallower than 0.3 → not enough seat; deeper than 5
//                       defeats the purpose — that's just a counterbore or
//                       pocket).
//
// Recognize:
//   Shallow (depth/dia < 0.5) circular pockets.  Confidence 0.65, overlaps
//   with mill_circular_pocket and counterbore.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace spot_face {

constexpr const char* kSkillId = "spot_face";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    double      dia_mm        = 0.0;
    double      depth_mm      = 0.0;   // typically 0.5 – 2 mm
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace spot_face
}  // namespace koocadcam::skill
