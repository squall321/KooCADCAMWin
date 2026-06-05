#pragma once
// @lat: [[engine/skills#linear_rail_carriage_pocket]]
//
// linear_rail_carriage_pocket — Profiled linear-guide CARRIAGE underside
// (slice 16, robotics / automation).
//
// The underside of a profiled linear-guide carriage (block) carries two
// re-circulating ball-groove channels that ride the rail's Gothic-arch
// raceways, flanked by N tapped mounting holes that fasten the carriage to
// the moving member.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. ball-groove channel A (box cut, parallel to rail axis)
//   2. ball-groove channel B (box cut, parallel to rail axis)
//   3..N+2. N mounting through-holes (tapped, from _iso_thread_table.hpp)
//
// subfeature_count = 2 + mount_hole_count.
//
// DFM:
//   DFM-INPUT          : every dimension must be > 0.
//   DFM-THREAD         : mount_thread_key must exist in central metric table.
//   DFM-ROBOTICS-RAIL  : 2*groove_width + rail clearance must fit rail_width.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace linear_rail_carriage_pocket {

constexpr const char* kSkillId = "linear_rail_carriage_pocket";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy        { 0.0, 0.0, 0.0 };
    double      rail_width_mm     = 15.0;
    double      groove_width_mm   = 3.0;
    double      groove_depth_mm   = 2.0;
    int         mount_hole_count  = 4;
    std::string mount_thread_key  = "M4";   // from _iso_thread_table.hpp
    double      mount_pitch_mm    = 26.0;   // hole-to-hole spacing along rail
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace linear_rail_carriage_pocket
}  // namespace koocadcam::skill
