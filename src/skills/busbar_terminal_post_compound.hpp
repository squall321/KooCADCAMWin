#pragma once
// @lat: [[engine/skills#busbar_terminal_post_compound]]
//
// busbar_terminal_post_compound — EV BATTERY compound: copper busbar terminal
// post mounting on a metal housing face with insulator landing.
//
// Sub-feature chain (3 ops):
//   1. FUSE a cylindrical copper-post stud on the face (post_dia × post_height).
//   2. CUT a threaded clearance bore THROUGH the post top (using ISO M-thread
//      tap pilot for the thread_size_key — central _iso_thread_table.hpp).
//   3. CUT an annular insulator recess around the post BASE on the face
//      (insulator_recess_dia × insulator_recess_depth) — landing for the
//      bushing/insulator sleeve that isolates the post from the housing.
//
// Signature:
//   pattern.kind                  = "busbar_terminal_post_compound"
//   pattern.is_compound           = true
//   pattern.ev_feature_type       = "busbar_terminal_post"
//   pattern.subfeature_count      = 3
//   pattern.thread_size_key       = "M6" etc.
//   pattern.derived_volume_added_mm3
//   pattern.derived_volume_removed_mm3
//
// DFM gates:
//   - all positive
//   - thread_size_key must exist in _iso_thread_table.hpp
//   - thread tap pilot < post_dia (must fit in post)
//   - insulator_recess_dia > post_dia + 0.5 (annular gap)
//   - insulator_recess_depth < 0.5 × insulator_recess_dia (geometric sanity)

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace busbar_terminal_post_compound {

constexpr const char* kSkillId = "busbar_terminal_post_compound";

struct Input
{
    int         face_id                   = -1;
    double      center_x_mm               = 0.0;
    double      center_y_mm               = 0.0;
    double      post_dia_mm               = 10.0;
    double      post_height_mm            = 8.0;
    std::string thread_size_key           = "M6";   // ISO metric
    double      insulator_recess_dia_mm   = 14.0;
    double      insulator_recess_depth_mm = 1.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace busbar_terminal_post_compound
}  // namespace koocadcam::skill
