#pragma once
// @lat: [[engine/skills#dome_boss]]
//
// dome_boss — a freeform spherical-cap dome fused onto a planar face.
//
// The natural feature for a watch's sapphire glass dome, a raised sensor dome,
// or any convex curved cap on a flat surface.  Built with pr::domeSolid (a
// BRepOffsetAPI_ThruSections loft of stacked circular sections following a
// spherical-cap profile r(z) = sqrt(Rs^2 - (z - zc)^2)), so the side wall is a
// genuine BSpline surface — NOT a true GeomAbs_Sphere.
//
// This is the apply()-side partner the dome recognizer needs: domeSolid lived
// only as an engine primitive inside the product builders, so a recovered dome
// had nothing to regenerate INTO.  With this skill, a dome round-trips like any
// other additive feature (generation vocabulary == recognition vocabulary).
//
//   apply():  resolve entry_face -> origin + outward normal -> gp_Ax2 at the
//             face-local centre -> pr::domeSolid(ax, base_radius, height) ->
//             pr::fuse onto the workpiece.
//
// DFM:
//   - base_radius_mm > 0, height_mm > 0
//   - sections >= 3 (ThruSections profile fidelity)

#include "Datum.hpp"
#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace dome_boss {

constexpr const char* kSkillId = "dome_boss";

struct Input
{
    FaceDatum entry_face;                  // planar face the dome sits on
    double    center_x_mm    = 0.0;        // dome base centre in face-local XY
    double    center_y_mm    = 0.0;
    double    base_radius_mm = 0.0;        // base circle radius
    double    height_mm      = 0.0;        // dome rise above the face
    int       sections       = 6;          // domeSolid loft fidelity
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace dome_boss
}  // namespace koocadcam::skill
