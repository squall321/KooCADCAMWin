#pragma once
// @lat: [[engine/skills#npt_taper_port_b120]]
//
// npt_taper_port_b120 — NPT tapered thread port per ANSI/ASME B1.20.1
// (slice-13b extension).  The taper is 1:16 (≈ 1.7857°/ft total; half-angle
// per side ≈ 1.7899°).
//
// COMPOUND of:
//   1. Straight pilot drill at recommended ANSI tap-drill dia for full
//      thread length (depth = L1 hand-tight engagement).
//   2. Conical bore at the NPT taper angle (overlay over pilot — wider at
//      the face, narrower at the bottom).
//
// Spec from central table `_hydraulic_ports.hpp` (kNpt).  Sizes: 1/8, 1/4,
// 3/8, 1/2, 3/4.
//
// DFM gates:
//   DFM-NPT-CODE: thread_size not in table.
//   DFM-NPT-FACE: target face not planar.
//   DFM-NPT-MAT : face XY span < 1.5 × pitch_dia (insufficient material).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace npt_taper_port_b120 {

constexpr const char* kSkillId = "npt_taper_port_b120";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm  = 0.0;
    double      center_y_mm  = 0.0;
    gp_Dir      axis_dir     { 0.0, 0.0, -1.0 };
    std::string thread_size;                       // "1/8" .. "3/4"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace npt_taper_port_b120
}  // namespace koocadcam::skill
