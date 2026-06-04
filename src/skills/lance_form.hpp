#pragma once
// @lat: [[engine/skills#lance_form]]
//
// lance_form — Lance-and-form: a narrow slit + an adjacent raised tab formed
// in ONE press operation.  Common for self-engaging tabs, louver fins,
// cable-tie anchors, and EMI shield mounting tongues.
//
//                                       ┌──┐  ← raised tab (formed up)
//                                       │  │
//          ──────────────────────  ╲    │  │
//                                   │   ├──┤
//                                   ├───┘  └────  ← parent sheet
//                                   │      slit (no material removed apart
//                                   ↓      from the tab pocket itself)
//          slit_start_xy   slit_end_xy
//
// Synthesis:
//   1. Cut a slit along [slit_start_xy → slit_end_xy] through the sheet.
//   2. Fuse a rectangular box adjacent to one side of the slit — this is
//      the formed tab, extruded UP by form_height_mm out of the sheet plane.
//
// DFM:
//   DFM-L-SHEET     : 0.5 ≤ thickness ≤ 6.0 mm
//   DFM-L-SLIT-LEN  : slit_length > 5 × thickness   (avoid edge tear)
//   DFM-L-HEIGHT    : form_height_mm ≥ thickness AND ≤ 4 × thickness
//   DFM-L-WIDTH     : form_width_mm  ≥ 2 × thickness
//
// Recognize:
//   A small RAISED rectangular planar face above the sheet plane, with
//   an adjacent through-cut planar slit at the original sheet level.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lance_form {

constexpr const char* kSkillId = "lance_form";

struct Input
{
    std::array<double, 2> slit_start_xy  { 0.0, 0.0 };
    std::array<double, 2> slit_end_xy    { 0.0, 0.0 };
    double                form_height_mm = 3.0;     // raise above sheet top
    double                form_width_mm  = 5.0;     // tab width
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lance_form
}  // namespace koocadcam::skill
