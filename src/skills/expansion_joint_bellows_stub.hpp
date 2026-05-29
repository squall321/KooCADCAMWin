#pragma once
// @lat: [[engine/skills#expansion_joint_bellows_stub]]
//
// expansion_joint_bellows_stub — REAL multi-cut compound for a piping
// expansion-joint stub end: a short pipe with N circumferential
// corrugations (approximated as N stacked toroids fused onto the pipe).
//
// Sub-features:
//   1. base end-flange disc
//   2. central pipe core (cylinder)
//   3. N stacked toroidal corrugations (N ≥ 2)
//   4. internal flow bore (through the entire stack)
//
// subfeature_count = 3 + N  (≥ 5 for the default 2 corrugations).
//
// DFM standard cited:
//   • EJMA-10 (Expansion Joint Manufacturers Association) —
//     corrugation pitch ≥ 1.5 × convolution_height; min convolutions = 2;
//     flange thickness ≥ 1.5 × bore_dia / 16; pipe-wall thickness ≥ 0.5 mm.
//
// Recognize:
//   1. Find a stack of ≥ 2 toroidal faces sharing a common axis with
//      monotonically increasing Z.
//   2. Same-axis large cylindrical face below = pipe core.
//   3. Same-axis planar disc face at the base = end flange.
//   Confidence 0.8 (geometric) / 1.0 (replay).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace expansion_joint_bellows_stub {

constexpr const char* kSkillId = "expansion_joint_bellows_stub";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm        = 0.0;
    double      center_y_mm        = 0.0;
    gp_Dir      axis_dir           { 0.0, 0.0, 1.0 };  // bellows grows along +axis
    double      flange_outer_dia_mm = 150.0;
    double      flange_thickness_mm = 14.0;
    double      pipe_outer_dia_mm   = 80.0;
    double      pipe_inner_dia_mm   = 70.0;
    double      pipe_length_mm      = 60.0;            // pipe core length above flange
    int         corrugation_count   = 3;
    double      corrugation_pitch_mm = 12.0;           // axial spacing between corrugations
    double      corrugation_outer_dia_mm = 100.0;      // bellows OD (toroid outer rim)
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace expansion_joint_bellows_stub
}  // namespace koocadcam::skill
