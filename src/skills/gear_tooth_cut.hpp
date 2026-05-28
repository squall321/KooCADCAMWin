#pragma once
// @lat: [[engine/skills#gear_tooth_cut]]
//
// gear_tooth_cut — cut a single involute gear tooth gap radially into a
// cylindrical stock (= remove the SPACE between two teeth).  Real involute
// flank geometry is far too involved for the synthesis layer; this skill
// approximates the cut as a wedge box whose width is the circular tooth
// gap and whose depth is (tip_radius − root_radius) along the radial
// direction.  The full involute parameters (module, pressure_angle, helix
// angle, root/tip radii, number of teeth) are recorded in the signature
// pattern verbatim so downstream CAPP / recognize() can match the lobe.
//
//                          tip_radius
//                              │
//                              ▼ (outermost)
//                       ╱─tooth gap (wedge cut)
//                      ╱ ╲
//                     ╱   ╲
//                    │     │   ← wedge approximation
//                    │     │
//                    └─────┘
//                        ▲
//                    root_radius
//
// Signature pattern:
//   - 2 planar flank walls (the two flat sides of the wedge — approximating
//     the involute flanks)
//   - 1 planar root face (bottom of the wedge, at root_radius)
//   - module / pressure_angle / helix_angle / Nteeth recorded as
//     pattern metadata
//
// DFM checks:
//   DFM-GEAR-MOD : module must be in [0.2, 50.0] mm
//   DFM-GEAR-PA  : pressure_angle in [14.5, 25.0] deg
//   DFM-GEAR-RAD : root_radius < tip_radius

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gear_tooth_cut {

constexpr const char* kSkillId = "gear_tooth_cut";

struct Input
{
    FaceDatum   entry_face;
    // Angular position of the tooth GAP CENTER (radians around +Z).
    double      angle_rad         = 0.0;
    // Gear geometric parameters.
    double      module_mm         = 1.0;
    double      pressure_angle_deg = 20.0;
    double      helix_angle_deg   = 0.0;     // 0 = spur
    int         num_teeth         = 20;
    double      root_radius_mm    = 9.0;     // inner radius of the wedge
    double      tip_radius_mm     = 11.0;    // outer radius of the wedge
    double      face_width_mm     = 5.0;     // axial length of the tooth (along +Z)
    gp_Dir      axis_dir          { 0.0, 0.0, 1.0 };   // gear axis (rotation axis)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gear_tooth_cut
}  // namespace koocadcam::skill
