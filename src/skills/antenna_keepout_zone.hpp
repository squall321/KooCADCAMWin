#pragma once
// @lat: [[engine/skills#antenna_keepout_zone]]
//
// antenna_keepout_zone — PHONE compound: metal-free clearance trough along a
// flex antenna trace polyline.
//
// Sub-features:
//   For each segment Pi → Pi+1 in the polyline, build an oriented box
//   (length = ‖segment‖, width = keep_out_width_mm, depth = keep_out_depth_mm).
//   Then cut all boxes from the workpiece in one pr::cutMany.
//
// DFM:
//   DFM-INPUT       : keep_out_width_mm > 0; keep_out_depth_mm > 0; polyline ≥ 2 pts
//   DFM-ANT-RANGE   : keep_out_width_mm ∈ [0.5, 5] mm typical
//
// Signature pattern:
//   pattern.kind                       = "antenna_keepout_zone"
//   pattern.is_compound                = true
//   pattern.is_phone_feature           = true
//   pattern.segment_count              = N
//   pattern.derived_volume_removed_mm3 = total volume removed

#include "Datum.hpp"
#include "Skill.hpp"

#include <utility>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace antenna_keepout_zone {

constexpr const char* kSkillId = "antenna_keepout_zone";

struct Input
{
    int                                       face_id            = -1;
    std::vector<std::pair<double, double>>    polyline;          // (x, y) on face
    double                                    keep_out_width_mm  = 1.5;
    double                                    keep_out_depth_mm  = 0.6;
    // ── Slice-13 polygon-pocket mode (optional, backward-compatible) ──
    //
    // If polygon_xy_points is non-empty (≥ 3 points), the skill cuts a
    // POLYGONAL pocket along that boundary instead of a polyline trough.
    // Depth = depth_mm (full-through if it exceeds frame thickness).
    // edge_taper_mm > 0 enables a 3° outward draft for mould-pull clearance.
    std::vector<std::pair<double, double>>    polygon_xy_points;
    double                                    depth_mm           = 0.0;
    double                                    edge_taper_mm      = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace antenna_keepout_zone
}  // namespace koocadcam::skill
