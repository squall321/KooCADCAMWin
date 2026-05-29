#pragma once
// @lat: [[engine/skills#shim_pocket]]
//
// shim_pocket — COMPOUND MACRO for a precision shim seat.
//
// Sub-feature chain (4 sub-cuts):
//   1. Main shim pocket            — rectangular pocket of (shim_l × shim_w)
//                                    at depth = shim_thk + 0.05 mm clearance.
//                                    Sized as a slip-fit (H8/h7).
//   2. Retention edge step         — shallow narrower pocket on the bottom
//                                    of the main pocket; serves as a relief
//                                    edge so the shim seats flat on its full
//                                    rim instead of bridging on tool marks.
//                                    Step depth = 0.1 mm; step inset = 0.5 mm.
//   3. Retention tab A             — small rectangular notch on one short
//                                    side, accepts the shim's "finger" tab.
//   4. Retention tab B             — diametrically opposed retention notch.
//
//                       ┌──── entry_face ──────────┐
//                       │ ┌────────────────────┐   │
//                       │ │     main pocket    │   │
//                       │ │  ┌──────────────┐  │   │
//                       │ │  │ relief step  │  │   │
//                       │ │  └──────────────┘  │   │
//                       │ │ [tab A]   [tab B]  │   │
//                       │ └────────────────────┘   │
//                       └──────────────────────────┘
//
// Standard:
//   - DIN 988 (shim ring) slip fit: pocket + 0.05 mm radial clearance.
//   - Retention finger geometry: 1.0 mm wide × 0.5 mm deep notches.
//
// DFM:
//   - shim_thk in [0.05, 5.0] mm (shim industry range).
//   - shim_w ≥ 4 mm, shim_l ≥ 4 mm (smaller is impractical to handle).
//   - retention step inset < min(shim_w, shim_l) / 4 (else step swallows
//     more than half the floor).
//
// Signature: is_compound = true, subfeature_count = 4.
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace shim_pocket {

constexpr const char* kSkillId = "shim_pocket";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm  = 0.0;
    double      position_y_mm  = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    double      shim_thk_mm    = 0.0;             // pocket depth ≈ shim_thk + clearance
    double      shim_w_mm      = 0.0;
    double      shim_l_mm      = 0.0;
    double      retention_step_depth_mm = 0.1;
    double      retention_step_inset_mm = 0.5;
    double      tab_notch_w_mm = 1.0;
    double      tab_notch_d_mm = 0.5;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace shim_pocket
}  // namespace koocadcam::skill
