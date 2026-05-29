#pragma once
// @lat: [[engine/skills#locating_g6_bore]]
//
// locating_g6_bore — ISO 286-1 G6 locating-fit bore.
//
// G6 fit class: slight running clearance over h-grade shaft, used for
// precision locating bushings and jig bushings.  Both deviations POSITIVE
// (bore is OVERSIZE by a small fixed offset, then has H6-style precision).
//
// COMPOUND macro: drill at nominal + ream to G6 dia + lead-in chamfer.
//
// ISO 286-1 G6 table (both deviations POSITIVE, micrometres):
//   Ø 1–3   → upper +8,  lower +2
//   Ø 3–6   → upper +12, lower +4
//   Ø 6–10  → upper +14, lower +5
//   Ø 10–18 → upper +17, lower +6
//   Ø 18–30 → upper +20, lower +7
//   Ø 30–50 → upper +25, lower +9
//   Ø 50–80 → upper +29, lower +10
//   Ø 80–120→ upper +34, lower +12

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace locating_g6_bore {

constexpr const char* kSkillId = "locating_g6_bore";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm   = 0.0;
    double      position_y_mm   = 0.0;
    gp_Dir      axis_dir        { 0.0, 0.0, -1.0 };
    double      nominal_dia_mm  = 0.0;
    double      depth_mm        = 0.0;
    double      chamfer_mm      = 0.3;
    std::string spec_key        = "G6";
};

struct G6Deviation { double upper_um; double lower_um; bool valid; };
G6Deviation deviationFor(double nominal_dia_mm);

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace locating_g6_bore
}  // namespace koocadcam::skill
