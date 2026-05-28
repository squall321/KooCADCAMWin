#pragma once
// @lat: [[engine/skills#thread_chase]]
//
// thread_chase — repair / cleanup of a damaged thread.  The chaser tool
// follows the existing helix and pushes any deformed material back into
// shape; NO new material is added or substantially removed.  Used to
// rescue cross-threaded fasteners, clean rust off threads, or knock down
// burrs after a separate operation.
//
// This is the canonical METADATA-ONLY skill in the threading family: the
// geometric change is effectively zero (a few microns of burr removal, far
// below modeling tolerance), so synthesis is a NO-OP on the shape and the
// signature exists purely to drive process planning / CAM toolpath
// generation.
//
//          ╲│╱  ← damaged crest with burr
//           ▼
//           ░░  ← (no geometric change after chase)
//          ╱│╲
//
// Signature pattern:
//   - pattern.kind == "thread_chase"
//   - pattern.target_thread_id : index of the prior thread feature this
//     chase is restoring (or -1 if no parent was discoverable).
//
// DFM checks:
//   DFM-CHASE-PARENT  : workpiece must contain a previous thread feature
//                       (tap_thread / rigid_tap / form_tap / thread_mill /
//                       thread_mill_external / thread_grind).  Without one
//                       there is no thread to chase.
//   DFM-CHASE-PASSES  : chase_depth_passes ∈ [1, 5].  More than 5 passes
//                       means the thread is too damaged for chasing; the
//                       caller should re-tap or re-mill.
//
// Recognize:
//   Metadata-only by definition (no geometric change).  We replay any
//   `thread_chase` entries from `wp.features()`.

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace thread_chase {

constexpr const char* kSkillId = "thread_chase";

struct Input
{
    FaceDatum existing_thread_datum;       // cylindrical face of the existing thread
    int       chase_depth_passes = 1;      // number of chasing passes
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace thread_chase
}  // namespace koocadcam::skill
