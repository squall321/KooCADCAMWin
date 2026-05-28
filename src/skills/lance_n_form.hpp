#pragma once
// @lat: [[engine/skills#lance_n_form]]
//
// lance_n_form — combination operation: a partial slit ("lance") is cut on
// three sides of a small rectangular tab; the still-attached fourth side
// acts as the hinge along which the tab is bent up to a target form
// height.  Common in EMI shielding fingers, ventilation louvers,
// stand-off tabs.
//
//             ┌───────────────────┐         ← lance slot (3-sided)
//             │ ◄── lance_length  │
//             ╲                   ╱
//              ╲ formed tab       ╱
//               ╲ (bent up)      ╱
//             ─────hinge edge───────  ← bend line (sheet surface)
//
// Geometry strategy (slice-1):
//   1. Subtractive slot: a thin rectangular slot is cut on three sides of
//      the tab footprint (leaving the hinge edge intact).
//   2. Additive tab: a small upright box (the bent tab) is fused on top of
//      the sheet at the hinge.
//   The result is a single solid that approximates a lanced-and-formed
//   feature without modeling the curved bend zone.
//
// Signature pattern:
//   - kind                  = "lance_n_form"
//   - tab_length_mm
//   - tab_width_mm
//   - form_height_mm        (height of the bent tab above the sheet)
//   - lance_kerf_mm         (slit width)
//   - hinge_side            = "+X" | "-X" | "+Y" | "-Y"
//
// DFM checks:
//   DFM-LNF-SHEET     : sheet_thickness ≤ 5 mm
//   DFM-LNF-MIN-TAB   : tab_length & tab_width ≥ 2 × sheet_thickness
//   DFM-LNF-MAX-FORM  : form_height ≤ 4 × sheet_thickness (tear risk above)
//   DFM-LNF-KERF      : kerf ≥ 0.5 × thickness (must be cuttable)
//   DFM-INPUT         : positive dimensions
//
// Recognize:
//   Metadata replay + spot-check for a small upright box at the recorded
//   coordinates (we do not attempt to recover this geometrically).

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lance_n_form {

constexpr const char* kSkillId = "lance_n_form";

enum class HingeSide { PlusX, MinusX, PlusY, MinusY };

inline std::string hingeSideToString(HingeSide h)
{
    switch (h) {
        case HingeSide::PlusX:  return "+X";
        case HingeSide::MinusX: return "-X";
        case HingeSide::PlusY:  return "+Y";
        case HingeSide::MinusY: return "-Y";
    }
    return "+X";
}

struct Input
{
    double      tab_center_x_mm = 0.0;
    double      tab_center_y_mm = 0.0;
    double      tab_length_mm   = 0.0;    // X extent of footprint
    double      tab_width_mm    = 0.0;    // Y extent of footprint
    double      form_height_mm  = 0.0;    // bent tab height above sheet
    double      lance_kerf_mm   = 0.2;    // slit kerf width
    HingeSide   hinge_side      = HingeSide::PlusY;  // which edge stays attached
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lance_n_form
}  // namespace koocadcam::skill
