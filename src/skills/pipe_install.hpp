#pragma once
// @lat: [[engine/skills#pipe_install]]
//
// pipe_install — installation record for a run of process / utility pipe
// (water, gas, steam, drainage).  The macroscopic B-rep of the building
// structure does NOT change; the pipe and its joints are a separate
// sub-OCCT routing entity captured purely as metadata.
//
// Signature pattern:
//   - pattern.kind            = "pipe_install"
//   - pattern.pipe_dia_mm     = nominal Ø
//   - pattern.pipe_material   = "copper" | "pvc" | "steel" | "stainless" | …
//   - pattern.length_m        = run length
//   - pattern.joint_count     = total joints (couplings, elbows, tees)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   pipe_dia_mm    ∈ (0, 600]                (DFM-PIPE-DIA error if outside)
//   length_m       > 0                       (DFM-INPUT error if not)
//   joint_count    ≥ 0                       (DFM-INPUT error if negative)
//   joint_density  ≤ 1 joint / 0.5 m         (DFM-PIPE-JOINTS info)
//   pipe_material  ∈ known set               (DFM-PIPE-MATERIAL info)

#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pipe_install {

constexpr const char* kSkillId = "pipe_install";

struct Input
{
    double      pipe_dia_mm   = 25.0;
    std::string pipe_material = "copper";
    double      length_m      = 10.0;
    int         joint_count   = 4;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pipe_install
}  // namespace koocadcam::skill
