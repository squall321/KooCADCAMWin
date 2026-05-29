#pragma once
// @lat: [[engine/skills#curved_lip_around_face]]
//
// curved_lip_around_face — COMPOUND, SPEC-DRIVEN macro that raises a
// continuous "lip" (rectangular ring frame) up from the perimeter of the
// entry face, offset inward by `inset_mm`.  This is the bezel-around-lens,
// the cover-edge-seal, and the lid-mate geometry used everywhere in
// consumer device enclosures.
//
//                 inner ring          outer ring
//                 (inner edge)        (outer edge)
//          ┌──────┐                                ┌──────┐
//          │      │                                │      │   ↑ lip_height
//          │      │                                │      │   │
//          │ lip  │ ◄── thickness ──►              │ lip  │   │
//          │      │                                │      │   │
//   ───────┴──────┴────────────────────────────────┴──────┴── entry_face
//                                                              (planar)
//          ◄── inset ──►                            ◄── inset ──►
//   ◄────────────────── face bbox length ────────────────────────►
//
// Spec library — `spec_class` selects (inset, thickness, height):
//
//   "bezel_lip"       → inset 1.0 mm, thickness 0.8 mm, height 0.5 mm
//   "cover_lip"       → inset 2.0 mm, thickness 1.5 mm, height 2.0 mm
//   "seal_groove_lip" → inset 3.0 mm, thickness 1.0 mm, height 1.5 mm
//   "flange_lip"      → inset 5.0 mm, thickness 2.5 mm, height 3.0 mm
//
// Implementation:
//   1. Use the entry face's bounding box to derive the outer rectangle.
//   2. Outer ring = bbox shrunk by `inset`.
//   3. Inner ring = outer shrunk by `thickness`.
//   4. Build outer prism (outer×height) and inner prism (inner×height).
//   5. Compute lip = outer_prism − inner_prism (a rectangular ring).
//   6. Fuse lip onto the workpiece.
//
// Auto-context detection:
//   Reads the entry face bbox FROM THE WORKPIECE — caller does NOT specify
//   length/width directly.  This is the "context-aware" part: the skill
//   queries the entry face dimensions and snaps the lip to its perimeter.
//   If the entry face is non-rectangular or zero-size, validate() reports
//   DFM-LIP-CONTEXT.
//
// Signature pattern:
//   - kind = "curved_lip_around_face"
//   - is_compound = true
//   - subfeature_count = 6
//   - spec_class, inset, thickness, height
//   - outer_l, outer_w, inner_l, inner_w
//   - face_id (the resolved entry face)
//
// DFM (mould rule + context-derived):
//   DFM-LIP-UNKNOWN  : spec_class not in library.
//   DFM-WALL-MIN     : thickness ≥ 0.4 mm.
//   DFM-LIP-INSET    : 2·(inset+thickness) < min(face_L, face_W) — else
//                      lip inner would invert.
//   DFM-LIP-ASPECT   : height / thickness ≤ 4 (stability).
//   DFM-LIP-CONTEXT  : entry face is not planar OR not large enough.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace curved_lip_around_face {

constexpr const char* kSkillId = "curved_lip_around_face";

struct Input
{
    FaceDatum   entry_face;
    std::string spec_class;
};

double specInsetFor    (const std::string& spec_class);
double specThicknessFor(const std::string& spec_class);
double specHeightFor   (const std::string& spec_class);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace curved_lip_around_face
}  // namespace koocadcam::skill
