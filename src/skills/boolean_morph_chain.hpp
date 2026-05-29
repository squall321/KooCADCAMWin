#pragma once
// @lat: [[engine/skills#boolean_morph_chain]]
//
// boolean_morph_chain — Compound skill that applies an ordered SEQUENCE of
// Boolean operations (cut / fuse) using primitive tool shapes (cylinder,
// box, sphere/cone) to morph the workpiece in N steps.  Every step is a
// real geometric op against the running shape.
//
// Pipeline (REAL multi-cut, ≥ 2 sub-features):
//   For each Op in operations:
//       tool = makePrimitive(Op.shape, Op.params)
//       workpiece = (Op.kind == "cut") ? cut(workpiece, tool)
//                                       : fuse(workpiece, tool)
//
// The skill emits one FeatureSignature covering the WHOLE morph chain,
// recording the sequence so it can be replayed deterministically.
//
// DFM:
//   DFM-INPUT          : operations must be non-empty (subfeature_count ≥ 2
//                        required for "compound" classification).
//   DFM-CHAIN-VOLUME   : final post-chain workpiece must have positive
//                        volume (chains that cut away the whole stock are
//                        rejected).  Agent-chosen rule motivated by DIN
//                        8580 "remaining material" gate.

#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace boolean_morph_chain {

constexpr const char* kSkillId = "boolean_morph_chain";

// Primitive shape for one chain step.
enum class ShapeKind { Cylinder, Box, Cone };

// "cut" subtracts the tool; "fuse" adds the tool.
enum class OpKind    { Cut, Fuse };

struct Op
{
    OpKind     op    = OpKind::Cut;
    ShapeKind  shape = ShapeKind::Cylinder;
    gp_Pnt     origin{ 0.0, 0.0, 0.0 };          // tool base point
    gp_Dir     axis  { 0.0, 0.0, 1.0 };           // local Z axis of the tool
    // Cylinder:  size_a = radius,    size_b = height,   size_c unused
    // Box     :  size_a = dx,        size_b = dy,       size_c = dz
    // Cone    :  size_a = r1_bottom, size_b = r2_top,   size_c = height
    double     size_a = 0.0;
    double     size_b = 0.0;
    double     size_c = 0.0;
};

struct Input
{
    std::vector<Op>  operations;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace boolean_morph_chain
}  // namespace koocadcam::skill
