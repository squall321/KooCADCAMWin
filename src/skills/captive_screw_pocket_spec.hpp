#pragma once
// @lat: [[engine/skills#captive_screw_pocket_spec]]
//
// captive_screw_pocket_spec — clearance hole sized for a metric screw PLUS a
// retaining-ring (DIN 6799 / "E-clip") groove that holds the screw captive
// (so the screw cannot fall out of the assembly during service).
//
//   • thread_size : "M3" .. "M30"   (ISO 273 medium fit drives the clearance)
//   • Spec table  : DIN 6799 retaining-ring shaft groove geometry:
//                    groove_dia    = nominal shaft dia − groove_depth × 2
//                    groove_width  = ring thickness (table value)
//                    groove_depth  = (shaft − groove_dia) / 2  (per spec)
//                   Embedded statically.
//   • Compound cuts (3):
//       1) clearance through-cylinder for the screw shank,
//       2) entry chamfer for screw insertion,
//       3) DIN 6799 retaining-ring annular groove.
//
// Signature pattern:
//   { kind, is_compound=true, subfeature_count=3, thread_size,
//     clearance_dia_mm, groove_dia_mm, groove_width_mm, groove_offset_mm }
//
// DFM gates:
//   DFM-METRIC-UNKNOWN : thread_size not in spec → error
//   DFM-CAPTIVE-OFFSET : groove_offset_mm < 1.0 mm → warning (too close to face)
//   DFM-CAPTIVE-WIDTH  : groove_width < 0.2 mm → error

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace captive_screw_pocket_spec {

constexpr const char* kSkillId = "captive_screw_pocket_spec";

struct Input
{
    FaceDatum    entry_face;
    double       position_x_mm    = 0.0;
    double       position_y_mm    = 0.0;
    gp_Dir       axis_dir         { 0.0, 0.0, -1.0 };
    std::string  thread_size      = "M6";
    double       chamfer_size_mm  = 0.5;
    double       groove_offset_mm = 2.0;   // offset from entry face to groove top
};

// Exposed for tests.
double clearanceDiameterFor(const std::string& thread_size);
double grooveDiameterFor   (const std::string& thread_size);
double grooveWidthFor      (const std::string& thread_size);

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace captive_screw_pocket_spec
}  // namespace koocadcam::skill
