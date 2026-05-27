#pragma once
// @lat: [[engine/skills#tap_thread]]
//
// tap_thread — an internal thread cut by a tap into an existing pilot hole.
//
//                         ┌─ existing_hole_datum (cylindrical face)
//                         ▼
//                   ──────┬──────────┬──────
//                         │   ▼ axis │
//                         │  ╱│╲     │
//                         │ ╱ │ ╲    │      ↑ thread_depth_mm
//                         │╱  │  ╲   │      │   (helical thread)
//                         │   │   │  │      │
//                         │   │   │  │      ↓
//                         └───┴───┴──┘
//                             ↑
//                          tap drill ø (existing)
//
// Signature pattern (what the skill leaves on the workpiece):
//   - In the FULL implementation: a helical groove face inside the bore wall.
//   - In this APPROXIMATION (slice 2): geometry is UNCHANGED — only metadata
//     is recorded.  The signature carries the thread parameters so process
//     plans can address tap intent against the pre-tap pilot bore.
//
// DFM checks:
//   DFM-015 : pilot diameter must match the tap-drill chart for thread_size
//             (warning if the existing cylindrical face diameter mismatches).
//   thread_depth_mm ≥ 1.5 × pitch_mm (engagement requirement).
//
// Recognize:
//   Without true helical geometry, analysis is metadata-only — scan the
//   workpiece feature history for previously registered tap_thread
//   signatures and replay them.  A STEP round-trip loses these (no real
//   helical face survives import); see TODO in .cpp.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tap_thread {

constexpr const char* kSkillId = "tap_thread";

struct Input
{
    FaceDatum   existing_hole_datum;       // pilot bore cylindrical face
    std::string thread_size      = "M3";   // "M2", "M2.5", "M3", "M4", "M5", "M6", "M8"
    double      pitch_mm         = 0.5;    // metric pitch (mm/rev)
    double      thread_depth_mm  = 0.0;    // engagement depth (≤ pilot depth)
};

// Synthesis: validate, then return the workpiece UNCHANGED + record a
// FeatureSignature describing the thread.  Slice 2 approximation.
SkillOutput apply(const Workpiece& wp, const Input& in);

// DFM validation.
DFMReport validate(const Workpiece& wp, const Input& in);

// Recognize candidates: scans wp.features() for previously registered
// tap_thread signatures.  Empty if metadata was lost (e.g. after STEP RT).
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Look up the recommended tap-drill diameter from the standard chart.
// Returns 0.0 for unknown sizes.
double tapDrillDiameter(const std::string& thread_size);

}  // namespace tap_thread
}  // namespace koocadcam::skill
