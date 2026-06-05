#pragma once
// @lat: [[engine/skills#blade_root_tbolt_compound]]
//
// blade_root_tbolt_compound — Wind turbine blade-root T-bolt joint
// (slice 16, wind domain).
//
// The T-bolt (barrel-nut) joint is the standard root attachment for modern
// fibreglass wind-turbine blades: an AXIAL stud is threaded into the
// laminate along the blade axis, and a TRANSVERSE cross bore receives a
// cylindrical barrel nut.  The two bores are PERPENDICULAR and INTERSECTING
// — the barrel nut captures the stud where they cross.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. axial stud bore   (cylinder cut DOWN from top face along +Z axis)
//   2. transverse barrel-nut cross bore (cylinder cut along +X, intersecting)
//
// subfeature_count = 2.
//
// DFM:
//   DFM-INPUT   : every dimension must be > 0.
//   DFM-THREAD  : stud_thread_key must exist in central metric thread table.
//   DFM-INTERSECT: barrel bore depth must reach the stud bore axis so the two
//                  bores actually intersect (joint is non-functional otherwise).
//   DFM-STUD-FIT : stud bore must clear the thread minor (>= nominal dia).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace blade_root_tbolt_compound {

constexpr const char* kSkillId = "blade_root_tbolt_compound";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      axis_origin       { 0.0, 0.0, 0.0 };  // XY of stud axis, Z ignored
    double      stud_bore_dia_mm  = 24.0;
    double      stud_depth_mm     = 120.0;
    double      barrel_nut_dia_mm = 30.0;
    double      barrel_depth_mm   = 60.0;             // transverse bore depth
    std::string stud_thread_key   = "M24";            // M-thread in _iso_thread_table
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace blade_root_tbolt_compound
}  // namespace koocadcam::skill
