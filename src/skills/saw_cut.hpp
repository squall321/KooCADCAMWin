#pragma once
// @lat: [[engine/skills#saw_cut]]
//
// saw_cut — separation cutting with a circular saw blade.
//
// A rotating circular blade traverses a 2D path (linear, circular, or
// arbitrary polyline) through the stock, leaving a wide kerf with a
// rough cut-edge finish.  This is the FAST + CHEAP option for separation
// (sheet metal, plate, bar stock).
//
//                          entry_face (planar, top of stock)
//                                  │
//                  ────────────────┼──────────────────
//                                  │
//                       ┌──────────┴────────┐    ↑  stock thickness
//                       │  ╱╱╱ kerf ╱╱╱     │    │  (saw cuts ALL the way
//                       │ (1.5 – 3.0 mm     │    │   through, leaving rough
//                       │  rough finish)    │    │   oxidised edge)
//                       └──────────╴────────┘    ↓
//                                  ↑
//                            exit face
//
// Process characteristics:
//   - kerf       : 1.5 – 3.0 mm (blade-tooth set)
//   - thickness  : virtually unlimited (depends on blade diameter)
//   - finish     : "rough"
//   - tool       : "circular_saw"
//
// Signature pattern:
//   Through-channel with kerf ≈ 1.5 – 3.0 mm; planar / cylindrical walls
//   connecting the top and bottom faces; NO bottom face.
//
// DFM checks:
//   DFM-002       : kerf within process bounds (1.5 – 3.0 mm)
//   DFM-THICKNESS : workpiece Z extent within process max
//   DFM-SURFACE   : info finding noting rough cut-edge finish
//
// Recognize:
//   Scans for through-channels; for each, returns saw_cut confidence per the
//   kerf-bucket table:
//     kerf 1.2 – 3.0 mm → 0.60 (primary)
//     kerf 3.0 – 6.0 mm → 0.40 (also plausible — overlap with plasma)
//     otherwise → 0 (skipped)

#include "Datum.hpp"
#include "Skill.hpp"
#include "_separation_common.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace saw_cut {

constexpr const char* kSkillId = "saw_cut";

// Input shape is the shared LinearCutInput (variant of cut kind).
using Input = separation_common::LinearCutInput;

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace saw_cut
}  // namespace koocadcam::skill
