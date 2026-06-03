#pragma once
// @lat: [[engine/skills#jic_37_flare_port]]
//
// jic_37_flare_port — JIC 37° flare-seat port per SAE J514.  Straight
// UN/UNF thread above a 37° conical seat (74° included angle).
//
// COMPOUND of:
//   1. Straight central flow bore.
//   2. 37° conical seat (cone frustum).
//   3. UN/UNF clearance cylinder above the seat (cosmetic thread).
//
// Spec from central table `_hydraulic_ports.hpp` (kJicFlare).  Dash sizes
// -4 / -6 / -8 / -12 / -16.
//
// DFM gates:
//   DFM-JIC-CODE: tube_size not in table.
//   DFM-JIC-FACE: target face not planar.
//   DFM-JIC-MAT : face XY span < 1.5 × flare_seat_od (insufficient material).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace jic_37_flare_port {

constexpr const char* kSkillId = "jic_37_flare_port";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm  = 0.0;
    double      center_y_mm  = 0.0;
    gp_Dir      axis_dir     { 0.0, 0.0, -1.0 };
    std::string tube_size;                         // "-4" .. "-16"
    double      bore_depth_mm = 0.0;               // 0 = derive from bore dia × 2
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace jic_37_flare_port
}  // namespace koocadcam::skill
