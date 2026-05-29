#pragma once
// @lat: [[engine/skills#spring_contact_clip]]
//
// spring_contact_clip — receptacle pocket for a U-shaped spring contact
// (Hirschmann KOSTAL style automotive/electronics blade socket).  Compound
// of three real sub-cuts:
//   1. U-shape pocket   : a slot whose floor accepts the U bend of the clip.
//   2. Back-stop pocket : a perpendicular shallow stop pocket at the closed
//                         end of the U so the inserted blade does not over-
//                         travel.
//   3. Retention barb slot : a slim window in one side wall that captures
//                            the clip's barb (prevents pull-out).
//
//                    ┌──────────────────┐
//                    │   ┌──── U ──┐    │   ← U-pocket (length × width × depth)
//                    │   │         │    │
//                    │   │         │    │
//                    │   └─────────┘    │
//                    │       │          │   ← back-stop pocket (smaller box)
//                    │       │   ▒▒     │   ← barb slot (side window)
//                    └───────┴──────────┘
//
// DFM (UL 486A / UL 310 inspired):
//   - clip_width_mm  ≥ 1.5 mm  (sub-mm-class clips need EDM, not milling)
//   - clip_depth_mm  ≥ 1.0 mm
//   - back_stop_depth_mm in (0, clip_depth_mm)
//   - barb_slot_width_mm ≥ 0.5 mm
//
// Sub-feature count: 3 → compound.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace spring_contact_clip {

constexpr const char* kSkillId = "spring_contact_clip";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm        = 0.0;   // center of U-pocket
    double      position_y_mm        = 0.0;
    gp_Dir      axis_dir             { 0.0, 0.0, -1.0 };
    // Orientation of the U opening (unit vector in XY) — opening faces +xLoc.
    double      orientation_deg      = 0.0;   // 0 = +X opening
    double      clip_length_mm       = 8.0;   // along orientation axis
    double      clip_width_mm        = 4.0;   // perpendicular
    double      clip_depth_mm        = 4.0;   // into the workpiece
    double      back_stop_width_mm   = 2.0;   // narrower back stop
    double      back_stop_depth_mm   = 1.0;   // shallower
    double      barb_slot_width_mm   = 1.5;   // along orientation axis (depth dim of side window)
    double      barb_slot_height_mm  = 0.8;   // vertical (axial) extent of the side window
    double      barb_slot_depth_mm   = 1.0;   // into the side wall, perpendicular to clip
    double      barb_slot_z_offset_mm = 1.5;  // from entry face, axially
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace spring_contact_clip
}  // namespace koocadcam::skill
