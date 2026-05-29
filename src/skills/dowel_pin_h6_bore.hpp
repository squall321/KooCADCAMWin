#pragma once
// @lat: [[engine/skills#dowel_pin_h6_bore]]
//
// dowel_pin_h6_bore — ISO 286-1 H6 bore for dowel-pin location.
//
// COMPOUND macro: drill + ream to H6 dia + lead-in chamfer.
//
// H6 is one grade TIGHTER than H7 — used for dowel-pin locating holes
// where positional repeatability under ±0.01 mm matters.
//
// ISO 286-1 H6 table (lower 0, upper = IT6 µm):
//   Ø 1–3   → +6
//   Ø 3–6   → +8
//   Ø 6–10  → +9
//   Ø 10–18 → +11
//   Ø 18–30 → +13
//   Ø 30–50 → +16
//   Ø 50–80 → +19
//   Ø 80–120→ +22
//
// Standard dowel-pin sizes (m6 shaft): Ø 2, 3, 4, 5, 6, 8, 10 mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace dowel_pin_h6_bore {

constexpr const char* kSkillId = "dowel_pin_h6_bore";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm   = 0.0;
    double      position_y_mm   = 0.0;
    gp_Dir      axis_dir        { 0.0, 0.0, -1.0 };
    double      nominal_dia_mm  = 0.0;
    double      depth_mm        = 0.0;
    double      chamfer_mm      = 0.2;    // small chamfer; pin entry needs precision
    std::string spec_key        = "H6";
};

double upperDeviationMicronsH6(double nominal_dia_mm);
bool   isStandardDowelSize(double nominal_dia_mm);

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace dowel_pin_h6_bore
}  // namespace koocadcam::skill
