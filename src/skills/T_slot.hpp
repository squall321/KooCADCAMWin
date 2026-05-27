#pragma once
// @lat: [[engine/skills#T_slot]]
//
// T_slot — T-shaped cross-section slot.  Narrow opening at top (the
// "neck"), wider chamber underneath (the "flange").  Common on machine
// tables and toolmaker fixtures for fastener heads to slide along while
// resisting pull-out.
//
//          entry_face (planar)
//                │
//   ─────────────┼─────────────────
//                │
//                │     ▼     neck_width_mm        ↑ neck_depth_mm
//                │  ┌──────┐                      │
//                │  │      │                      ↓
//                │ ┌┴──────┴┐  flange_width_mm    ↑ flange_depth_mm
//                │ │        │                     │
//                │ └────────┘                     ↓
//                              ◄────────────►
//
// Implementation: fuse two boxes (a thin upper neck + a wider lower flange)
// into a T-shaped prism, then cut from the workpiece.  Both boxes share the
// same start→end axis and length.
//
// Signature pattern:
//   - 2 narrow planar walls at the neck level
//   - 2 wider planar walls at the flange level
//   - 1 planar flat-bottom face
//   - 2 horizontal "step" faces where neck meets flange
//
// DFM checks:
//   DFM-002 : neck_width_mm  ≥ 0.8 mm
//   DFM-002 : flange_width_mm ≥ 0.8 mm
//   flange_width_mm > neck_width_mm  (this is what makes it a T-slot)
//
// Recognize:
//   1. Find pairs of parallel planar walls at TWO different in-pocket Z
//      levels with the same in-plane axis direction.
//   2. Verify a flat bottom face at the deepest level and step faces at
//      the neck/flange transition.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace T_slot {

constexpr const char* kSkillId = "T_slot";

struct Input
{
    FaceDatum   entry_face;
    double      start_x_mm        = 0.0;
    double      start_y_mm        = 0.0;
    double      end_x_mm          = 0.0;
    double      end_y_mm          = 0.0;
    gp_Dir      axis_dir          { 0.0, 0.0, -1.0 };
    double      neck_width_mm     = 0.0;  // narrow opening at top
    double      flange_width_mm   = 0.0;  // wider chamber below
    double      neck_depth_mm     = 0.0;  // depth of the neck (from entry face)
    double      flange_depth_mm   = 0.0;  // depth of the flange (below the neck)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace T_slot
}  // namespace koocadcam::skill
