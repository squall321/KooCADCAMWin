#pragma once
// @lat: [[engine/skills#top_face_recess_with_walls]]
//
// top_face_recess_with_walls — COMPOUND, SPEC-DRIVEN macro that:
//   (1) Cuts a rectangular recess into a planar top face,
//   (2) Raises 4 walls around its perimeter, and
//   (3) Fillets the 4 outer top corners of those walls.
//
// This is the standard PCB tray / gasket-seat / lid-seat geometry used in
// every enclosure design.
//
//          ┌─────────┐                ┌─────────┐               fillet R
//          │         │                │         │              ┊ (corner)
//          │ wall    │  ◄──pocket──►  │  wall   │ ─────► (4)
//          │         │   floor (cut)  │         │
//   ───────┴─────────┴────────────────┴─────────┴──────  entry_face
//                  ▲  recess_depth
//                  ▼  (below entry_face)
//                                                          top
//          ┌─────────────────────────────────────┐         (wall_height
//          │                                     │          above entry)
//          │  wall    pocket floor       wall    │
//   ──────┴──────────────────────────────────────┴──── entry_face
//
// Spec library — `spec_class` selects the wall geometry:
//
//   "small_pcb"   → wall_thickness 1.0 mm, wall_height 2.0 mm, fillet R 0.5 mm
//   "large_pcb"   → wall_thickness 1.5 mm, wall_height 3.0 mm, fillet R 1.0 mm
//   "gasket_seat" → wall_thickness 2.0 mm, wall_height 1.5 mm, fillet R 0.5 mm
//   "lid_seat"    → wall_thickness 1.5 mm, wall_height 4.0 mm, fillet R 1.0 mm
//
// Caller provides: footprint (length × width), recess_depth, position,
// orientation.  The walls sit OUTSIDE the recess footprint — i.e. the
// outer face of each wall is at `length/2 + wall_thickness` from centre.
//
// Compound chain (5 OPS):
//   1. Cut rectangular pocket of (length × width × depth) below entry plane.
//   2. Fuse +X wall (box).
//   3. Fuse +Y wall.
//   4. Fuse −X wall and −Y wall (counted as one op for symmetry; uses
//      fuseMany).
//   5. Fillet 4 outer-top corner edges with R from spec.
//
// Signature pattern:
//   - kind = "top_face_recess_with_walls"
//   - is_compound = true
//   - subfeature_count = 5
//   - spec_class, wall_thickness, wall_height, fillet_r
//   - inner_length, inner_width, recess_depth
//
// DFM:
//   DFM-RECESS-UNKNOWN : spec_class not in library.
//   DFM-WALL-MIN       : wall_thickness ≥ 0.4 mm.
//   DFM-RECESS-INNER   : inner_length, inner_width > 0.
//   DFM-RECESS-DEPTH   : recess_depth ≥ 0 (0 = walls-only, no recess).
//   DFM-RECESS-FILLET  : fillet_r ≤ wall_thickness  (else fillet exceeds
//                        the wall edge).
//
// Recognize:
//   Replay primary.  Geometric: identify a downward planar face below the
//   workpiece top (the pocket floor) + 4 upward-facing wall-top faces at
//   a uniform Z above the workpiece top.  Confidence ≈ 0.7 / 0.92.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace top_face_recess_with_walls {

constexpr const char* kSkillId = "top_face_recess_with_walls";

struct Input
{
    FaceDatum   entry_face;
    std::string spec_class;
    double      center_x_mm    = 0.0;
    double      center_y_mm    = 0.0;
    double      inner_length_mm = 0.0;     // recess X-dimension
    double      inner_width_mm  = 0.0;     // recess Y-dimension
    double      recess_depth_mm = 0.0;     // below entry plane (≥ 0)
};

double specWallThicknessFor(const std::string& spec_class);
double specWallHeightFor   (const std::string& spec_class);
double specFilletRFor      (const std::string& spec_class);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace top_face_recess_with_walls
}  // namespace koocadcam::skill
