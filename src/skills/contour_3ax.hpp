#pragma once
// @lat: [[engine/skills#contour_3ax]]
//
// contour_3ax — 3-axis surface following.  The tool follows a complex 3D
// polyline (X, Y, Z) along the workpiece surface, removing material along
// the swept stadium of the cutter centerline.  Each segment may have its
// own Z, so the resulting "pocket" has a VARYING-Z bottom (not flat).
//
// At slice-1 fidelity we model this as a CHAIN of slot-stadium segments,
// each at the segment's own bottom Z.  Adjacent segments fuse where they
// touch at the polyline vertex.
//
//                     workpiece top (entry_face)
//                          │
//             ─────────────┼─────────────────
//                          │
//      ┌──────────┐        │       ┌────────┐    each segment has its
//      │ seg 0    └────────┴───────┘ seg N  │    own Z; the floor of the
//      │ z=z0          z=zMid          z=zN │    pocket varies along the
//      └────────────────────────────────────┘    polyline.
//                           ↑
//                    contour_polyline_3d
//
// Signature pattern:
//   - chain of N-1 stadium segments (each is a mill_slot-like shape)
//   - signature.skill_id == "contour_3ax"
//   - signature.pattern.kind == "contour_3ax"
//   - signature.pattern.segments[] : list of (start, end, z) per segment
//
// DFM checks:
//   DFM-002      : tool_dia_mm ≥ 0.8 mm
//   DFM-3AX-PTS  : contour_polyline_3d.size() ≥ 2 (need at least one segment)
//   DFM-3AX-BBOX : all polyline points within the workpiece XY bbox
//
// Recognize:
//   Pattern: sequence of cylindrical end-cap pairs at varying Z values,
//   connected by planar walls.  Topologically each segment is a half-slot;
//   the full feature is a chain.  We use the same dual-path strategy:
//   feature-history first (conf 0.85), and a geometric heuristic that
//   counts vertical-axis cylindrical end-caps and emits a low-confidence
//   marker (conf 0.50) when ≥ 4 are found (2 per segment, ≥ 2 segments).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace contour_3ax {

constexpr const char* kSkillId = "contour_3ax";

struct ContourPoint
{
    double x_mm = 0.0;
    double y_mm = 0.0;
    double z_mm = 0.0;
};

struct Input
{
    FaceDatum                   entry_face;
    std::vector<ContourPoint>   contour_polyline_3d;
    gp_Dir                      axis_dir      { 0.0, 0.0, -1.0 };
    double                      tool_dia_mm   = 6.0;
    // Lateral stepover (used by CAM for multi-pass follow; informational
    // here — the swept stadium width is the full tool diameter).
    double                      stepover_mm   = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace contour_3ax
}  // namespace koocadcam::skill
