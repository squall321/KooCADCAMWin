#pragma once
// @lat: [[engine/skills#face_milling_5ax]]
//
// face_milling_5ax — resurface an angled (non-axis-aligned) face.
//
// Same intent as `face_milling`, but applies to a face whose normal is NOT
// necessarily aligned with the global +Z direction.  On a 3-axis machine
// only horizontal-top faces can be face-milled in a single setup; an
// angled face (e.g. on a wedge-shaped part, or a chamfer-like flat) needs
// the tool to tilt — that's a 5-axis (or 3+2) operation.
//
//   Before:                       After (depth = d):
//
//        ╱─────────────╲                ╱─────────────╲    ↑
//        ╲             ╱                ╲             ╱    │
//        ╱  entry_face ╲                ╱   (new face,╲    │ d
//        ╲   (angled)  ╱                ╲    parallel) ╱   ↓
//        ╱─────────────╲                ╲─────────────╱
//        (face has arbitrary normal,    (face offset
//         not parallel to any axis)      inward by d
//                                        along its own normal)
//
// Signature pattern:
//   Same as face_milling — the original entry face has been replaced by a
//   parallel face offset by `depth_mm` along its inward normal.  The key
//   marker is `pattern.entry_face_normal`: NOT parallel to any global axis
//   (no component close to ±1) → flags 5-axis.
//
// DFM checks:
//   depth_mm > 0
//   depth_mm < 5 mm  → warning if greater
//   DFM-005 info  — "5-axis required if entry_face normal is not parallel
//                    to a global axis" (always emitted; the orchestrator
//                    decides whether 3+2 is sufficient).
//
// Recognize (limited):
//   Same heuristic as face_milling but the candidate face's normal must NOT
//   be parallel to any global axis.  Low confidence (0.45) — without prior
//   state, depth_mm is unrecoverable.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace face_milling_5ax {

constexpr const char* kSkillId = "face_milling_5ax";

struct Input
{
    FaceDatum entry_face;        // a planar face with arbitrary normal
    double    depth_mm = 0.0;    // how much material to remove
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace face_milling_5ax
}  // namespace koocadcam::skill
