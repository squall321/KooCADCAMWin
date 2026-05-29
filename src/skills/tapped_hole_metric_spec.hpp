#pragma once
// @lat: [[engine/skills#tapped_hole_metric_spec]]
//
// tapped_hole_metric_spec — metric tapped hole (blind or through) driven by
// thread_size; the pilot (tap drill) diameter comes from ISO 965 / ISO 2306,
// not from the caller.
//
//   • thread_size : "M3" .. "M30"
//   • through     : bool — tap goes all the way through if true
//   • tap_depth   : usable thread depth (if blind)
//   • Produces 2 cuts: pilot drill + entry chamfer (60° standard tap lead).
//
// Spec lookup (ISO 965-1, 1ˢᵗ choice coarse-pitch nominal tap-drill dia):
//   M3=2.5, M4=3.3, M5=4.2, M6=5.0, M8=6.8, M10=8.5, M12=10.2,
//   M16=14.0, M20=17.5, M24=21.0, M30=26.5
//
// Tap thread itself is NOT modelled in geometry — it is carried as metadata
// for CAPP (matches drill_and_tap convention).  The compound geometry that
// IS modelled = pilot column + entry chamfer to deburr the thread start.
//
// Signature pattern:
//   { kind, is_compound=true, subfeature_count=2, thread_size,
//     pilot_dia_mm, tap_depth_mm, through, axis_dir }
//
// DFM gates:
//   DFM-METRIC-UNKNOWN : thread_size not in ISO 965 table → error
//   DFM-TAP-DEPTH      : tap_depth_mm <= 0 for blind → error
//   DFM-TAP-PECK       : tap_depth/pilot_dia > 3 → warning (chip evacuation)
//   DFM-MARGIN         : pilot_dia < 0.8 mm (tap drill min) → error

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tapped_hole_metric_spec {

constexpr const char* kSkillId = "tapped_hole_metric_spec";

struct Input
{
    FaceDatum    entry_face;
    double       position_x_mm   = 0.0;
    double       position_y_mm   = 0.0;
    gp_Dir       axis_dir        { 0.0, 0.0, -1.0 };
    std::string  thread_size     = "M6";
    double       tap_depth_mm    = 8.0;     // usable threads, ignored if through
    bool         through         = false;
    double       chamfer_size_mm = 0.4;     // entry chamfer leg
    double       pilot_extra_depth_mm = 1.0;  // drill goes deeper than tap
};

// Exposed for tests.
double pilotDiameterFor(const std::string& thread_size);

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tapped_hole_metric_spec
}  // namespace koocadcam::skill
