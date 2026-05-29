#pragma once
// @lat: [[engine/skills#draft_wall_with_radius]]
//
// draft_wall_with_radius — COMPOUND, SPEC-DRIVEN macro that raises a single
// rectangular wall from a planar face with an OUTWARD DRAFT (top is wider
// than the base) and a TOP-EDGE FILLET.  The draft is required for mould
// release; the top fillet softens stress concentrations and improves user
// touch on consumer products.
//
//                              ◄── top_thickness (= base + 2H·tanα) ──►
//                              ┌──────────────────────────────────────┐  fillet R (top edges)
//                              │   ╲                                ╱ │
//                              │    ╲   wall body (drafted outward) ╱  │
//                              │     ╲           α°                ╱   │ height_mm
//                              │      ╲                          ╱     │
//                              └───────────────────────────────────────┘
//                                ◄── base_thickness ──►
//          ─────────────────────────────────────────────────────────────  entry_face
//                                                                          (planar)
//
// NOTE on draft direction:
//   We use OUTWARD draft (top WIDER than base) — this is the convention for
//   external wall faces in mould design: when the tool retracts upward,
//   nothing snags.  This is the inverse of cavity draft (where top is
//   narrower).  The sign of `draft_angle_deg` is the magnitude of the
//   outward draft.
//
// Spec library — `spec_class` selects (draft°, top fillet R):
//
//   "low_draft"        →  draft 1.0°, R 0.3 mm   (precision tight-fit walls)
//   "standard_draft"   →  draft 3.0°, R 0.5 mm   (general consumer wall)
//   "high_draft"       →  draft 5.0°, R 1.0 mm   (textured / matte release)
//   "aggressive_draft" →  draft 7.0°, R 1.5 mm   (heavy-section bumpers)
//
// Implementation:
//   1. Build base rectangle (in entry plane, dims = length × base_thickness).
//   2. Build top rectangle (offset along outward normal by height, dims =
//      length_top × top_thickness, where each is widened by 2H·tan(α)).
//   3. Loft (ThruSections) between the two wires to produce a tapered solid.
//   4. Fuse onto workpiece.
//   5. Apply a fillet at the topmost Z (top-rim edges) of the resulting
//      shape using prim::filletEdges with predicate edgesAtZ(topZ).
//
// Signature pattern:
//   - kind = "draft_wall_with_radius"
//   - is_compound = true
//   - subfeature_count = 5 (base wire, top wire, loft, fuse, top fillet)
//   - spec_class
//   - base_thickness, top_thickness, height, length, draft_angle, fillet_r
//
// DFM (mold-rule + spec validation):
//   DFM-DRAFT-UNKNOWN : spec_class not in library.
//   DFM-WALL-MIN      : base_thickness ≥ 0.4 mm.
//   DFM-DRAFT-RANGE   : draft_angle within [0, 15°].
//   DFM-FILLET-LIMIT  : fillet R ≤ 0.4 × base_thickness (else fillet
//                       consumes the wall).
//   DFM-DRAFT-COLLAPSE: top_thickness > 0 (i.e. draft would not collapse
//                       the section to zero).  Currently outward draft
//                       cannot cause collapse — kept for symmetry with
//                       cavity-draft variants.
//
// Recognize:
//   Replay strategy is primary.  Geometric fallback: a solid extension
//   above the entry plane bounded by 4 angled planar walls (non-vertical
//   normals) + 1 top planar face; the angle between any side wall and the
//   entry-plane normal recovers the draft angle.  Confidence ≈ 0.6 on
//   geometry, 0.92 on history replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace draft_wall_with_radius {

constexpr const char* kSkillId = "draft_wall_with_radius";

struct Input
{
    FaceDatum   entry_face;                       // planar base face
    std::string spec_class;                       // selects draft + R from table
    double      center_x_mm     = 0.0;            // wall centre on the face
    double      center_y_mm     = 0.0;
    double      length_mm       = 0.0;            // wall long dimension (base)
    double      base_thickness_mm = 0.0;          // wall short dim at base
    double      height_mm       = 0.0;            // extrusion height above face
    gp_Dir      length_dir      { 1.0, 0.0, 0.0 };// direction of `length` in plane
};

// Spec lookup.  Returns 0.0 if unknown.
double specDraftDegFor(const std::string& spec_class);
double specFilletRMmFor(const std::string& spec_class);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace draft_wall_with_radius
}  // namespace koocadcam::skill
