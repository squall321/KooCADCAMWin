#pragma once
// @lat: [[engine/skills#rigid_tap]]
//
// rigid_tap — internal thread cut by a tap with RIGID spindle/feed
// synchronization (no floating tap holder).  Geometrically identical to
// `tap_thread`; the distinguishing characteristic is in the toolpath and
// the metadata that downstream CAPP / CAM uses to emit G84.2 / G84 with
// rigid-tap mode rather than tension-compression mode.
//
//                         ┌─ existing_hole_datum (cylindrical face)
//                         ▼
//                   ──────┬──────────┬──────
//                         │   ▼ axis │
//                         │  ╱│╲     │
//                         │ ╱ │ ╲    │      ↑ thread_depth_mm
//                         │╱  │  ╲   │      │
//                         │   │   │  │      ↓  (rigid tap — spindle is
//                         └───┴───┴──┘          synchronized to feed)
//                             ↑
//                          tap drill ø
//
// Signature pattern:
//   - Real V-thread helical groove inside the bore wall (built by
//     `prim::helicalSweep`).  Identical geometry to tap_thread; signature
//     kind is the differentiator.
//
// DFM checks:
//   DFM-015 : pilot diameter must match the tap-drill chart for thread_size.
//   DFM-TAP-ENGAGE : thread_depth_mm ≥ 1.5 × pitch_mm.
//   DFM-RIGID-TAP-PITCH : pitch must match a standard metric coarse value
//     for the nominal thread size (M3→0.5, M4→0.7, M5→0.8, M6→1.0, M8→1.25,
//     M10→1.5, …).  Non-standard pitch is allowed but flagged as a warning
//     because rigid-tap heads are typically configured per standard pitch.
//
// Recognize:
//   Same topology as tap_thread (a helical V-groove face inside the bore).
//   Recognition is metadata-driven: we scan workpiece.features() for entries
//   tagged with `skill_id == "rigid_tap"` and replay them at 0.5 confidence
//   (lower than tap_thread's 1.0 because the geometric pattern alone cannot
//   distinguish rigid- from tension-compression-tapping — only the metadata
//   trail can).

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rigid_tap {

constexpr const char* kSkillId = "rigid_tap";

struct Input
{
    FaceDatum   existing_hole_datum;       // pilot bore cylindrical face
    std::string thread_size      = "M3";   // "M2", "M2.5", "M3", "M4", "M5", "M6", "M8", "M10"
    double      pitch_mm         = 0.5;    // metric pitch (mm/rev)
    double      thread_depth_mm  = 0.0;    // engagement depth
};

// Synthesis: real helical sweep + cut from the workpiece, identical to
// tap_thread.  Returns the new workpiece + a rigid_tap signature.
SkillOutput apply(const Workpiece& wp, const Input& in);

// DFM validation (DFM-015, DFM-TAP-ENGAGE, DFM-RIGID-TAP-PITCH).
DFMReport validate(const Workpiece& wp, const Input& in);

// Recognize candidates: metadata-only scan of wp.features().
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Standard metric coarse pitch chart.  Returns the expected pitch for the
// given thread size, or 0.0 for unknown sizes.
double standardCoarsePitch(const std::string& thread_size);

// Tap-drill chart (mirror of tap_thread::tapDrillDiameter — duplicated here
// to keep rigid_tap self-contained).
double tapDrillDiameter(const std::string& thread_size);

}  // namespace rigid_tap
}  // namespace koocadcam::skill
