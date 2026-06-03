#pragma once
// @lat: [[engine/skills#bsp_g_thread_port]]
//
// bsp_g_thread_port — BSPP / ISO 228-1 G-thread parallel hydraulic port.
//
// COMPOUND of:
//   1. Face spotface (counterbore) for sealing washer / bonded seal.
//   2. Main tap drill bore.
//   3. Cosmetic / approximated thread groove on the bore wall (chamfered
//      cylinder pulses approximating a real helical groove).
//   4. Entry chamfer.
//
// Spec from central table `_hydraulic_ports.hpp` (kBspG).  Supported sizes:
// G1/8, G1/4, G3/8, G1/2, G3/4, G1.
//
// DFM gates:
//   DFM-BSPG-CODE: thread_size not in table.
//   DFM-BSPG-FACE: target face not planar.
//   DFM-BSPG-MAT : face XY span < 1.5 × spotface dia (insufficient material).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bsp_g_thread_port {

constexpr const char* kSkillId = "bsp_g_thread_port";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm  = 0.0;
    double      center_y_mm  = 0.0;
    gp_Dir      axis_dir     { 0.0, 0.0, -1.0 };
    std::string thread_size;                       // "G1/8" .. "G1"
    double      depth_override_mm = 0.0;           // 0 = use spec default
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bsp_g_thread_port
}  // namespace koocadcam::skill
