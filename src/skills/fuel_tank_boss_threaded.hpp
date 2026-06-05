#pragma once
// @lat: [[engine/skills#fuel_tank_boss_threaded]]
//
// fuel_tank_boss_threaded — Integral fuel-tank boss with threaded port and
// AS568 O-ring face seal (aerospace structures).
//
// Integral wing/fuselage fuel tanks have machined bosses that raise a local
// pad on the tank wall, into which a fitting is threaded; an O-ring face
// groove on the boss top seals the fitting flange.  This is a NET-ADDITIVE
// feature: the raised boss adds more material than the threaded bore + groove
// remove.
//
// Sub-features (boss FUSE first, then SEQUENTIAL pr::cut):
//   1. raised boss (fuse a cylinder onto the top face)
//   2. threaded port bore (M-thread tap-pilot from _iso_thread_table.hpp)
//   3. AS568 O-ring face groove (annular ring on the boss top)
//   subfeature_count = 3.
//
// DFM:
//   DFM-INPUT  : every dimension must be > 0.
//   DFM-THREAD : thread_key must exist in _iso_thread_table (metric).
//   DFM-ORING  : o_ring_size_key must exist in _as568_table.
//   DFM-FIT    : O-ring groove must fit on the boss top (boss_dia > groove OD).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace fuel_tank_boss_threaded {

constexpr const char* kSkillId = "fuel_tank_boss_threaded";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy      { 0.0, 0.0, 0.0 };   // boss centre (XY)
    double      boss_dia_mm    { 30.0 };
    double      boss_height_mm { 8.0 };
    std::string thread_key;                          // e.g. "M12" (_iso_thread_table)
    std::string o_ring_size_key;                     // e.g. "-016" (_as568_table)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace fuel_tank_boss_threaded
}  // namespace koocadcam::skill
