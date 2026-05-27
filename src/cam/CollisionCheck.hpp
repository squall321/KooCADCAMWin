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

// Walk every segment in `path` at `sample_step_mm` resolution, comparing
// against `wp.boundingBox()` plus a `safe_z_margin` (default 0.1 mm) above
// the stock top.  Returns the list of detected collisions (empty = clear).
//
// Notes
// - For arc segments we sample by chord length (i.e., we approximate the arc
//   by its end-point, since slice-1 paths are kept short).  Future work
//   will sample along the arc proper.
// - The workpiece's bbox is computed once; we do NOT update it as the
//   toolpath progresses.  This is acceptable for slice-1 (single-feature
//   check) but a multi-feature plan with stock removal will over-report.
std::vector<CollisionEvent> checkPath(
    const Toolpath&         path,
    const skill::Workpiece& wp,
    double                  sample_step_mm = 0.5,
    double                  safe_z_margin  = 0.1);

}  // namespace koocadcam::cam
