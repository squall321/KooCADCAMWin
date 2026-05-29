#pragma once
// @lat: [[engine/skills#threaded_through_with_chamfers]]
//
// threaded_through_with_chamfers — through tapped hole with deburring
// chamfers on BOTH faces and a thread-relief groove just before the bottom
// (when the operator wants the tap to chase a relief instead of running
// into solid stock).  Compound of 4 cuts:
//   1) pilot cylinder (full-through, ISO 965 tap-drill)
//   2) entry chamfer (60° tap lead, near the entry face)
//   3) exit chamfer  (60°, on the opposite face)
//   4) bottom thread relief — a slightly larger-diameter undercut groove,
//      half the pitch wide, just below the entry chamfer (used as a chip-
//      escape groove for tapping operations that need it).
//
//   • thread_size : "M3" .. "M30"  (ISO 965 lookup)
//   • Spec-driven: relief dia = pilot_dia + 2·pitch, relief width = pitch.
//
// Signature pattern:
//   { kind, is_compound=true, subfeature_count=4,
//     thread_size, pilot_dia_mm, pitch_mm, relief_dia_mm, relief_width_mm }
//
// DFM gates:
//   DFM-METRIC-UNKNOWN : thread_size not in ISO 965 → error
//   DFM-RELIEF-WIDTH   : relief_width_mm < 0.2 mm → error (cutter clearance)
//   DFM-THROUGH-LEN    : stock_thickness < pilot_dia × 1.5 → warning

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace threaded_through_with_chamfers {

constexpr const char* kSkillId = "threaded_through_with_chamfers";

struct Input
{
    FaceDatum    entry_face;
    double       position_x_mm    = 0.0;
    double       position_y_mm    = 0.0;
    gp_Dir       axis_dir         { 0.0, 0.0, -1.0 };
    std::string  thread_size      = "M6";
    double       chamfer_size_mm  = 0.5;     // both ends
    bool         add_relief       = true;
    double       relief_offset_mm = 2.0;     // relief sits this far from entry
};

double pilotDiameterFor(const std::string& thread_size);

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace threaded_through_with_chamfers
}  // namespace koocadcam::skill
