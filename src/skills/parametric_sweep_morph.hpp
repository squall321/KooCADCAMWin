#pragma once
// @lat: [[engine/skills#parametric_sweep_morph]]
//
// parametric_sweep_morph — Compound skill that sweeps a CHANGING profile
// along a straight path from `path_start` to `path_end`.  The profile is a
// circle whose radius morphs linearly from `start_radius` at the path
// start to `end_radius` at the path end.  Built as a series of cross
// sections fed to BRepOffsetAPI_ThruSections (equivalent to a PipeShell
// with a per-station radius), then CUT from the workpiece.
//
// Pipeline (REAL multi-cut, ≥ 2 sub-features):
//   1. Build start_circle wire at path_start with start_radius.
//   2. Build N − 2 intermediate circles along the path with lerped radii.
//   3. Build end_circle wire at path_end with end_radius.
//   4. ThruSections({wires...}) → swept morph solid.
//   5. prim::cut(workpiece, sweep_solid) → final cavity.
//
// DFM:
//   DFM-INPUT          : start_radius / end_radius > 0; path length > 0;
//                        intermediate_count ≥ 2.
//   DFM-SWEEP-MIN-D    : both radii ≥ 0.4 mm (DFM-002 derivative —
//                        smaller-than-cutter rejection).
//   DFM-SWEEP-TAPER    : warn when radius ratio > 4:1 along the path
//                        (CAM tool change requirement, agent-chosen
//                        threshold motivated by ISO 13399 cutter
//                        envelope guidance).

#include "Skill.hpp"

#include <gp_Pnt.hxx>

namespace koocadcam::skill {

class Workpiece;

namespace parametric_sweep_morph {

constexpr const char* kSkillId = "parametric_sweep_morph";

struct Input
{
    gp_Pnt   path_start    { 0.0, 0.0, 0.0 };
    gp_Pnt   path_end      { 0.0, 0.0, 0.0 };
    double   start_radius  = 0.0;
    double   end_radius    = 0.0;
    int      intermediate_count = 3;   // number of ThruSections waypoints
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace parametric_sweep_morph
}  // namespace koocadcam::skill
