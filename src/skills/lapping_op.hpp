#pragma once
// @lat: [[engine/skills#lapping_op]]
//
// lapping_op — FINEST mechanical finish.  Sub-micron material removal
// (typical 0.0001–0.005 mm) using a loose-abrasive compound on a flat lap
// plate.  Produces mirror finishes (Ra ≤ 0.025 µm) and final dimensional
// accuracy (within ±0.5 µm).  Standard for gauge blocks, optical flats,
// precision seal faces.
//
//      Before:                                  After (mirror finish, near-
//                                                      identical geometry):
//          ┌─────────────┐                          ┌─────────────┐
//          │  (Ra 0.2)   │                          │  (Ra 0.02)  │
//          │             │              ─→          │             │
//          │             │                          │             │
//          └─────────────┘                          └─────────────┘
//
// Geometric handling:
//   At our coarse modelling fidelity (typical OCCT modelling tolerance
//   ≈ 1e-4 mm), a sub-micron removal is BELOW the noise floor.  We split
//   behaviour by `removal_mm`:
//     - removal_mm < 0.001 mm  → return the workpiece UNCHANGED (geometric
//                                no-op) but record the lapping signature
//                                in the feature history.
//     - removal_mm ≥ 0.001 mm  → delegate to face_milling::apply for the
//                                cut, restamp signature with lapping
//                                metadata.
//
// DFM checks:
//   removal_mm ∈ [0.0001, 0.005]
//
// Recognize:
//   Impossible without metadata — a lapped face looks the same as any other
//   flat planar face.  recognize() returns empty.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lapping_op {

constexpr const char* kSkillId = "lapping_op";

// Threshold below which we treat lapping as a no-op (geometrically).  This
// is roughly OCCT's modelling tolerance — Boolean ops on sub-tolerance
// cutters can produce degenerate / inconsistent results, so we skip them
// entirely and rely on the signature to carry the intent.
constexpr double kGeometricThresholdMm = 0.001;

struct Input
{
    FaceDatum     entry_face;
    double        removal_mm = 0.001;
    // e.g. "diamond_3um", "diamond_1um", "aluminum_oxide_5um".
    std::string   lapping_compound = "diamond_3um";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lapping_op
}  // namespace koocadcam::skill
