#pragma once
// @lat: [[engine/skills#npt_taper_port]]
//
// npt_taper_port — NPT (ANSI B1.20.1) tapered hydraulic / pneumatic port.
//
// Slice-13 hydraulic version (separate from slice-9 `threaded_npt_port`):
// drives directly off the central `_hydraulic_ports_table.hpp` spec.
//
// COMPOUND of:
//   1. Tapered bore (cone frustum, half-angle ≈ 1.7899°/side per ANSI NPT).
//   2. Entry chamfer (45° face bevel) at the mouth.
//   3. Small face counterbore relief at the mouth.
//
// DFM gates:
//   DFM-INPUT     : non-positive sizes.
//   DFM-NPT-CODE  : npt_size not in {1/16, 1/8, 1/4, 3/8, 1/2, 3/4, 1}.
//   DFM-NPT-DEPTH : face thickness < depth + 2 mm safety margin.
//
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace npt_taper_port {

constexpr const char* kSkillId = "npt_taper_port";

struct Input
{
    FaceDatum   face_id;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir     { 0.0, 0.0, -1.0 };
    std::string npt_size;                        // "1/16" … "1"
    double      depth_mm = 12.0;                 // total bore depth into part
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace npt_taper_port
}  // namespace koocadcam::skill
