#pragma once
// @lat: [[engine/skills#squaring]]
//
// squaring — "square up" a rough stock by trimming all 6 sides to a tight
// axis-aligned rectangular envelope.  This is the universal first operation
// on any rough plate / billet before precision machining: a face mill (or
// surface grinder, or fly cutter) is run on all six sides in turn until
// every adjacent pair is mutually perpendicular and the part fits a known
// rectangular envelope.
//
//       Before:                              After:
//       ┌────────────────┐                   ┌──────────────┐
//      ╱  rough billet   ╲                   │              │
//     ╱   (six skewed     ╲                  │              │
//    ╱     casting faces)  ╲                 │  cleaned     │
//   ╱                       ╲                │  square      │
//   ╲                       ╱                │  envelope    │
//    ╲                     ╱                 │              │
//     ╲                   ╱                  │              │
//      ╲                 ╱                   └──────────────┘
//       └───────────────┘
//
// In this simplified model we collapse the six face-milling passes into
// ONE op: compute the optimal axis-aligned bbox of the input shape,
// optionally inset by `target_envelope_margin_mm` to leave stock to clean
// up, then intersect the input with that envelope using BRepAlgoAPI_Common.
// Any rough material protruding outside the bbox is removed; any internal
// detail is preserved.  Records the per-face material removed so that
// downstream cycle-time estimation can budget the six passes accurately.
//
// Signature pattern:
//   - pattern.kind                    = "squaring"
//   - pattern.target_envelope_margin_mm = <input>
//   - pattern.envelope_size_mm        = [dx, dy, dz]
//   - pattern.material_removed_per_face = { "+x": mm3, "-x": ..., ... }
//   - pattern.total_material_removed_mm3
//
// DFM checks:
//   DFM-INPUT      target_envelope_margin_mm < 0       (error)
//   SQ-MARGIN-DEEP target_envelope_margin_mm > 5 mm    (warning — implies
//                  a single face-mill pass exceeds typical 5 mm depth)
//
// Recognize (limited):
//   Without prior workpiece state we cannot distinguish a "squared rough
//   billet" from "a rectangular block that was always square."  The
//   recognizer therefore emits ONE candidate if and only if the input's
//   shape closely matches its own optimal bbox (volume ratio > 0.95) — i.e.
//   the part is currently rectangular — with confidence 0.40.  Metadata
//   replay (from features()) is always returned at confidence 1.0.

#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace squaring {

constexpr const char* kSkillId = "squaring";

struct Input
{
    // Inward inset applied to the bbox before the common-intersect.  Zero
    // means "clean exactly to bbox"; positive means "leave this much stock
    // on every side for finishing."  Negative is invalid.
    double target_envelope_margin_mm = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace squaring
}  // namespace koocadcam::skill
