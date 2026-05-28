#pragma once
// @lat: [[engine/skills#surface_grind]]
//
// surface_grind — PRECISION FINISH on a planar face.  Removes a very thin
// layer (typically 0.005–0.2 mm) to improve flatness and surface finish.
// Geometrically equivalent to a very shallow face_milling, but the process
// metadata differs: a surface grinder uses a bonded-abrasive wheel rather
// than a rotating end mill.  Output Ra is typically 0.2–0.8 µm.
//
//      Before:                                After (removal_mm):
//
//          ┌─────────────┐                       ┌─────────────┐    ↑ removal
//          │  (Ra 1.6)   │                       │ (Ra 0.2)    │    │
//          ├─────────────┤                       ├─────────────┤    ↓
//          │             │                       │             │
//          │             │                       │             │
//          └─────────────┘                       └─────────────┘
//
// Synthesis strategy:
//   Geometrically identical to face_milling(depth = removal_mm).  We delegate
//   to face_milling::apply() and restamp the FeatureSignature with this
//   skill's id + grinding-specific tooling metadata.
//
// DFM checks:
//   removal_mm ∈ [0.005, 0.2]            (out-of-range is hard error)
//   removal_mm ≤ workpiece_thickness × 0.1  (geometric sanity)
//
// Recognize:
//   Topologically a finished surface looks like a face_milled face — at
//   our sub-tenth-mm modelling fidelity the surface_grind result is
//   indistinguishable from a fine face mill pass.  recognize() returns
//   empty (process-metadata is required to identify a ground surface).

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace surface_grind {

constexpr const char* kSkillId = "surface_grind";

struct Input
{
    FaceDatum     entry_face;            // a planar face on the workpiece
    double        removal_mm = 0.02;     // material removal (default 0.02 mm)
    std::string   surface_finish = "ra_0.4";  // e.g. "ra_0.2" / "ra_0.4" / "ra_0.8"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace surface_grind
}  // namespace koocadcam::skill
