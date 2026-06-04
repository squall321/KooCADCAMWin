#pragma once
// @lat: [[engine/skills#vent_burst_disc_pocket]]
//
// vent_burst_disc_pocket — EV BATTERY compound: pressure-relief burst-disc
// mounting pocket with O-ring face groove + center vent hole through.
//
// Sub-feature chain (3 ops):
//   1. CUT cylindrical pocket bore of disc_dia × pocket_depth
//   2. CUT AS568 O-ring face groove around the pocket bore (uses central
//      _as568_table.hpp depth/width for the dash spec); groove sits on the
//      face plane, centred on the disc bore
//   3. CUT a center vent hole through the housing along the bore axis
//      (vent_dia ≈ disc_dia / 3) — the actual pressure-vent path
//
// Signature:
//   pattern.kind             = "vent_burst_disc_pocket"
//   pattern.is_compound      = true
//   pattern.ev_feature_type  = "vent_burst_disc"
//   pattern.subfeature_count = 3
//   pattern.o_ring_size_key
//   pattern.derived_volume_removed_mm3
//
// DFM gates:
//   - all positive
//   - disc_dia ∈ [4, 25] mm typical
//   - o_ring_size_key must exist in central AS568 table
//   - groove mean_dia derived = disc_dia + 2 mm + groove_width
//   - pocket depth ≥ groove depth + 0.5 (the groove must sit on the FACE,
//     not at the bottom of the pocket — i.e. the pocket is the disc seat,
//     the groove is on the rim)
//   - vent_dia < disc_dia / 2

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace vent_burst_disc_pocket {

constexpr const char* kSkillId = "vent_burst_disc_pocket";

struct Input
{
    int         face_id          = -1;
    double      center_x_mm      = 0.0;
    double      center_y_mm      = 0.0;
    double      disc_dia_mm      = 10.0;     // typ 8-12 mm
    double      pocket_depth_mm  = 2.0;
    std::string o_ring_size_key  = "-111";   // AS568 dash
    double      vent_dia_mm      = 3.0;      // through-vent (≪ disc_dia)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace vent_burst_disc_pocket
}  // namespace koocadcam::skill
