#pragma once
// @lat: [[engine/skills#rack_tooth_cut]]
//
// rack_tooth_cut — cut a single rack tooth GAP (rectangular notch) on a
// flat surface.  A rack is a gear with infinite radius, so each tooth gap
// is a straight rectangular notch oriented perpendicular to the rack
// length.  The slice-1 approximation uses a plain rectangular box cut;
// the trapezoidal flank angle of a true ISO rack is recorded as
// pattern metadata.
//
//          ┌───────────────────────────────────────┐  ← top of rack stock
//          │                                       │
//          │      ╔══════╗      ╔══════╗           │   ← tooth gap notches
//          │      ║      ║      ║      ║           │     (rectangular)
//          │      ║      ║      ║      ║           │
//          │──────╝      ╚──────╝      ╚──────…────│
//          │                                       │
//          └───────────────────────────────────────┘
//
// Signature pattern:
//   - 2 planar flank walls (perpendicular to rack length axis)
//   - 1 planar root face (parallel to rack top)
//   - module / pressure_angle / pitch recorded
//
// DFM checks:
//   DFM-GEAR-MOD : module in [0.2, 50.0] mm
//   DFM-GEAR-PA  : pressure_angle in [14.5, 25.0] deg
//   depth > 0

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rack_tooth_cut {

constexpr const char* kSkillId = "rack_tooth_cut";

struct Input
{
    FaceDatum   entry_face;
    // Position of the tooth gap center on the rack top face (world XY).
    double      position_x_mm     = 0.0;
    double      position_y_mm     = 0.0;
    // Rack geometric parameters.
    double      module_mm         = 1.0;
    double      pressure_angle_deg = 20.0;
    double      depth_mm          = 0.0;     // notch depth (= tooth height)
    double      width_mm          = 0.0;     // notch width (= circular pitch / 2)
    double      length_mm         = 0.0;     // notch length across the rack face
    gp_Dir      length_dir        { 1.0, 0.0, 0.0 };   // direction the notch runs
    gp_Dir      axis_dir          { 0.0, 0.0, -1.0 };  // cut direction (into face)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rack_tooth_cut
}  // namespace koocadcam::skill
