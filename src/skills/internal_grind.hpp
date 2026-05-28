#pragma once
// @lat: [[engine/skills#internal_grind]]
//
// internal_grind — INTERNAL precision grind on an existing bore.  Enlarges
// the bore diameter by removal_mm to improve roundness, cylindricity, and
// surface finish.  Process distinct from `ream`: uses a mounted abrasive
// stone on a quill rather than a multi-flute reamer.
//
//      Before:                                  After (radius + removal_mm):
//
//          ┌─┬─┐                                    ┌──┬───┬──┐
//          │ │ │                                    │  │   │  │
//          │ │ │                                    │  │   │  │
//          │ │ │              ─→                    │  │   │  │
//          │ │ │                                    │  │   │  │
//          └─┴─┘                                    └──┴───┴──┘
//          d=R                                     d=R+Δ
//
// Synthesis:
//   Like ream::apply, but with grind-tooling metadata: a cylinder of
//   (oldR + removal_mm) coaxial with the existing bore is cut from the
//   workpiece.
//
// DFM checks:
//   removal_mm ∈ [0.005, 0.1]
//   target bore diameter ≥ 5 mm  — smaller bores cannot accept a grinding
//                                  quill (use ream instead).
//
// Recognize:
//   Topologically identical to a wider drill_hole / ream bore.  Without
//   process metadata internal_grind cannot be distinguished; recognize()
//   returns empty.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace internal_grind {

constexpr const char* kSkillId = "internal_grind";

struct Input
{
    // EXISTING bore on the workpiece.
    FaceDatum     cylindrical_face_datum;
    // Radial enlargement (Δ-radius).  Resulting radius = old + removal_mm.
    double        removal_mm = 0.02;
    std::string   surface_finish = "ra_0.4";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace internal_grind
}  // namespace koocadcam::skill
