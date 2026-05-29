#pragma once
// @lat: [[engine/skills#slip_fit_h11_bore_spec]]
//
// slip_fit_h11_bore_spec — ISO 286-1 H11 loose/slip-fit bore.
//
// COMPOUND macro: drill at nominal + ream to oversized (IT11) dia + chamfer.
//
// ISO 286-1 H11 table (lower 0, upper = IT11 tolerance, micrometres):
//   Ø 1–3   → +60
//   Ø 3–6   → +75
//   Ø 6–10  → +90
//   Ø 10–18 → +110
//   Ø 18–30 → +130
//   Ø 30–50 → +160
//   Ø 50–80 → +190
//   Ø 80–120→ +220
//
// H11 is much LOOSER than H7 — used for clearance holes / non-precision
// guide holes.  Boring bar / single-pass drill is sufficient (no reamer).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace slip_fit_h11_bore_spec {

constexpr const char* kSkillId = "slip_fit_h11_bore_spec";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm   = 0.0;
    double      position_y_mm   = 0.0;
    gp_Dir      axis_dir        { 0.0, 0.0, -1.0 };
    double      nominal_dia_mm  = 0.0;
    double      depth_mm        = 0.0;
    double      chamfer_mm      = 0.5;    // generous chamfer for slip-fit
    std::string spec_key        = "H11";
};

double upperDeviationMicronsH11(double nominal_dia_mm);

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace slip_fit_h11_bore_spec
}  // namespace koocadcam::skill
