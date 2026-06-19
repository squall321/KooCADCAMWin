#pragma once
// @lat: [[engine/cam-3axis-verify#Stage 4]]
//
// CollisionCheck — slice-1 cheap-and-cheerful 3-axis collision detector.
//
// This is NOT the full BRepExtrema_DistShapeShape pipeline described in
// `lat.md/engine/cam-3axis-verify.md` Stage 4 — that requires building a
// tool envelope solid for every sample point, which is too slow for an
// inner-loop check.  Instead we use a coarse bbox-based heuristic that
// is O(N) in path samples and a few floats per sample:
//
//   For each PathSegment, walk in `sample_step_mm` increments from the
//   previous segment's end-point to this segment's end-point.  At each
//   sample, conceptually place a cylinder of radius = tool_dia/2 and
//   height = tool_length_mm at the sample point (tool tip).  Flag a
//   collision if the cylinder body intersects the workpiece bbox ABOVE
//   the tool tip's Z (i.e., the *shank* would dive into stock above the
//   programmed cut depth).
//
// In practice:
//
//   sample_xy ∈ stock XY footprint AND sample.Z + tool_length > stock.zMax
//   → no collision (shank above stock).
//
//   sample.Z >= stock.zMax (tip above stock top) → no collision.
//
//   Otherwise: check whether the SHANK column from sample.Z to
//   sample.Z + tool_length intersects the bbox cylinder around (x, y).
//
// This catches the common slice-1 failure mode — a Rapid that dives
// straight into stock — without doing real solid Booleans.

#include "Toolpath.hpp"

#include "skills/Workpiece.hpp"

#include <string>
#include <vector>

namespace koocadcam::cam {

// One collision event.  `sample_t` is the parametric position along the
// segment ([0, 1]); `description` is a human-readable diagnostic.
struct CollisionEvent
{
    int          segment_index = -1;
    double       sample_t      = 0.0;
    std::string  description;
};

// Walk every segment in `path` at `sample_step_mm` resolution.  Two-phase:
//
//  BROAD phase  — the original cheap single-AABB reject: a sample whose tip is
//    above the stock top (+margin) or outside the XY footprint (+toolR) is
//    DEFINITELY clear and is skipped.  O(1) per sample.
//  NARROW phase — when `exact_narrow_phase` is true, a sample the broad phase
//    could not clear is confirmed against the REAL BRep (wp.shape()): a hit is
//    reported only if the tip is INSIDE the solid (BRepClass3d_SolidClassifier)
//    or the tool column is within `safe_z_margin` of a real face
//    (BRepExtrema_DistShapeShape).  This is what distinguishes a plunge into a
//    concave wall (real gouge) from a clearance move over an OPEN pocket (the
//    bbox treats the pocket interior as solid and false-positives it).
//
// Pass exact_narrow_phase=false for the legacy bbox-only behaviour.
//
// Notes
// - For arc segments we sample by chord length (the end-point), since slice-1
//   paths are kept short.  Future work will sample along the arc proper.
// - The bbox is computed once and NOT updated as the path removes stock.
//
// NB: koo_cam currently has no production caller (it is exercised only by its
// own unit tests), so this accuracy fix ships dark until checkPath /
// generateAllToolpaths are wired into the process Executor or GUI — a tracked
// follow-up.  Bringing the DistShapeShape pipeline forward (planned M6 work)
// nonetheless makes the unit test meaningful and the checker correct-by-design.
std::vector<CollisionEvent> checkPath(
    const Toolpath&         path,
    const skill::Workpiece& wp,
    double                  sample_step_mm     = 0.5,
    double                  safe_z_margin      = 0.1,
    bool                    exact_narrow_phase = true);

}  // namespace koocadcam::cam
