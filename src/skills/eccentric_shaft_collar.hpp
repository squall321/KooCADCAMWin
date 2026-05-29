#pragma once
// @lat: [[engine/skills#eccentric_shaft_collar]]
//
// eccentric_shaft_collar — a cylindrical collar (outer disc) with an
// off-axis through bore, plus a radial set-screw threaded hole into the
// outer wall.  Used to lock a shaft into the collar with a measurable
// eccentricity (cam follower take-up rings, paper-feed pinch rolls).
//
// Compound feature: 3 primitive operations on cylindrical stock:
//   1. (input is a cylindrical stock or a flat disc — caller supplies)
//   2. CUT a through cylinder OFFSET from the collar centre by `eccentricity_mm`
//      along +X (the eccentric bore).
//   3. CUT a radial cylinder into the outer wall at +Y at half-height
//      (the set-screw threaded hole — modelled as a plain pilot bore).
//
// DFM gate (DIN 705 collar geometry + ISO 68-1 pilot diameters):
//   - bore_dia_mm ≥ 1.0 mm and ≤ 0.6 × outer_dia_mm  (DIN 705 wall ratio)
//   - eccentricity_mm ≥ 0 and bore_dia/2 + eccentricity ≤ 0.45·outer_dia_mm
//     (keeps ≥ 0.05·outer_dia between bore wall and OD)
//   - set_screw_dia_mm ≥ 0.8 mm (DFM-002, smallest standard tap drill)
//   - set_screw_dia_mm ≤ outer_dia/2 - bore_dia/2 - eccentricity (must clear bore)
//
// Recognition:
//   PRIMARY (geometric): find a cylindrical face that is not concentric
//     with the workpiece centroid by ≥ tolerance → that's the eccentric
//     bore.  Plus a third cylindrical face whose axis is perpendicular
//     and intersects the outer wall → that's the set-screw hole.
//   FALLBACK (metadata): if recovered from feature history, confidence
//     = 1.0.
//
// gp_Ax2(P, V, Vx) reminder: V is local Z (axial direction of the
// cylinder primitive), Vx is local X.  All three cuts use the same
// principle: pick V along the cylinder axis.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace eccentric_shaft_collar {

constexpr const char* kSkillId = "eccentric_shaft_collar";

struct Input
{
    double outer_dia_mm       = 30.0;    // collar OD
    double thickness_mm       = 12.0;    // collar axial length
    double bore_dia_mm        = 10.0;    // eccentric through-bore diameter
    double eccentricity_mm    = 2.0;     // offset of bore centre from collar centre, along +X
    double set_screw_dia_mm   = 3.3;     // radial set-screw pilot (M4 = 3.3)
    double set_screw_depth_mm = 6.0;     // how deep the radial hole bites
    // The collar centre in the world frame.
    double center_x_mm        = 0.0;
    double center_y_mm        = 0.0;
    double center_z_mm        = 0.0;     // bottom face Z; collar extends +Z by thickness_mm
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace eccentric_shaft_collar
}  // namespace koocadcam::skill
