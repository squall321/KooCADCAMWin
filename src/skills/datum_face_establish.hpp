#pragma once
// @lat: [[engine/skills#datum_face_establish]]
//
// datum_face_establish — face-mill one nominated face to high flatness so
// that it can serve as a GD&T datum (A / B / C) for all subsequent
// operations.  Geometrically this is a face_milling-style skim cut at a
// shallow depth, but semantically the face is TAGGED with a datum class
// and a target flatness in μm; the signature records these so downstream
// QA can pull a CMM scan against the same tag.
//
//        ┌────────────────┐                ┌────────────────┐
//        │                │                │ ░░ DATUM A ░░ │   ← new face,
//        │                │ datum_face     │                │     stamped
//        │   stock        │ ────────────▶ │   stock         │     "A"
//        │                │  (depth d)     │                │
//        │                │                │                │
//        └────────────────┘                └────────────────┘
//
// Signature pattern:
//   - pattern.kind                 = "datum_face_establish"
//   - pattern.datum_face_id        = <resolved id on output workpiece>
//   - pattern.datum_face_normal    = [nx, ny, nz]
//   - pattern.datum_class          = "A" | "B" | "C"
//   - pattern.required_flatness_um = <input>
//   - pattern.depth_mm             = <input>
//
// DFM checks:
//   DFM-INPUT       depth_mm <= 0                 (error)
//   DFM-INPUT       datum_class ∉ {A,B,C}         (error)
//   DFM-FLATNESS    required_flatness_um <= 0     (error)
//   DF-FLAT-LOOSE   required_flatness_um > 50     (warning — typical face mill
//                                                  achieves 5–25 µm flatness)
//   DF-DEPTH-DEEP   depth_mm >= 5                 (warning)
//
// Recognize:
//   Geometric recognition is undecidable (any planar face MIGHT be a datum).
//   Metadata replay returns prior datum_face_establish signatures with
//   confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace datum_face_establish {

constexpr const char* kSkillId = "datum_face_establish";

struct Input
{
    FaceDatum    datum_face;                  // which face to clean & tag
    std::string  datum_class           = "A"; // "A" | "B" | "C"
    double       required_flatness_um  = 10.0;
    double       depth_mm              = 0.5;  // light cleanup skim
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace datum_face_establish
}  // namespace koocadcam::skill
