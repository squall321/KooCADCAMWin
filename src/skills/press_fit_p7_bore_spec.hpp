#pragma once
// @lat: [[engine/skills#press_fit_p7_bore_spec]]
//
// press_fit_p7_bore_spec — ISO 286-1 P7 interference-fit bore.
//
// COMPOUND macro: drill at nominal + ream to deviated dia + lead-in chamfer.
//
// ISO 286-1 P7 table (both deviations NEGATIVE — bore is undersize):
//   Ø 1–3   → upper -6 µm, lower -16 µm
//   Ø 3–6   → upper -8 µm, lower -20 µm
//   Ø 6–10  → upper -9 µm, lower -24 µm
//   Ø 10–18 → upper -11 µm,lower -29 µm
//   Ø 18–30 → upper -14 µm,lower -35 µm
//   Ø 30–50 → upper -17 µm,lower -42 µm
//   Ø 50–80 → upper -21 µm,lower -51 µm
//   Ø 80–120→ upper -24 µm,lower -59 µm
//
// Press fit assumption: shaft is at H6/h7 nominal, bore at P7 → interference
// of ~upper_dev_um of magnitude.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace press_fit_p7_bore_spec {

constexpr const char* kSkillId = "press_fit_p7_bore_spec";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm   = 0.0;
    double      position_y_mm   = 0.0;
    gp_Dir      axis_dir        { 0.0, 0.0, -1.0 };
    double      nominal_dia_mm  = 0.0;
    double      depth_mm        = 0.0;
    double      chamfer_mm      = 0.5;    // larger chamfer for press start
    std::string spec_key        = "P7";
};

struct P7Deviation { double upper_um; double lower_um; bool valid; };
P7Deviation deviationFor(double nominal_dia_mm);

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace press_fit_p7_bore_spec
}  // namespace koocadcam::skill
