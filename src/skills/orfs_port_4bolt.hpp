#pragma once
// @lat: [[engine/skills#orfs_port_4bolt]]
//
// orfs_port_4bolt — ORFS (O-Ring Face Seal) flat-face 4-bolt manifold port.
//
// COMPOUND of:
//   1. Central port flow bore.
//   2. Concentric captive face O-ring groove.
//   3. Four bolt-clearance holes on a PCD around the port.
//
// Spec from central table `_hydraulic_ports.hpp` (kOrfsPort).  Dash sizes
// -4 / -6 / -8 / -12 / -16 / -20.
//
// DFM gates:
//   DFM-ORFS-CODE: dash_size not in table.
//   DFM-ORFS-FACE: target face not planar.
//   DFM-ORFS-MAT : face XY span < 1.5 × bolt_circle_dia.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace orfs_port_4bolt {

constexpr const char* kSkillId = "orfs_port_4bolt";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm  = 0.0;
    double      center_y_mm  = 0.0;
    gp_Dir      axis_dir     { 0.0, 0.0, -1.0 };
    std::string dash_size;                          // "-4" .. "-20"
    double      depth_mm = 12.0;                    // port bore depth
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace orfs_port_4bolt
}  // namespace koocadcam::skill
