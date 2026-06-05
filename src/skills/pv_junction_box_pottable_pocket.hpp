#pragma once
// @lat: [[engine/skills#pv_junction_box_pottable_pocket]]
//
// pv_junction_box_pottable_pocket — SOLAR/PV compound: photovoltaic junction-box
// potting pocket with N tapped M-thread cable-gland bores along one side wall.
//
// Sub-feature chain (SEQUENTIAL pr::cut, N+1 ops):
//   1. CUT rectangular potting pocket (pocket_w × pocket_h × pocket_depth)
//   2..N+1. CUT gland_count tapped cable-gland bores through pocket -Y wall
//
// Signature:
//   pattern.kind                = "pv_junction_box_pottable_pocket"
//   pattern.is_compound         = true
//   pattern.solar_feature_type  = "pv_junction_box_pocket"
//   pattern.subfeature_count    = 1 + gland_count
//   pattern.gland_thread_size_key
//   pattern.gland_pilot_dia_mm  (derived from central thread table)
//
// DFM gates:
//   - all positive
//   - gland_thread_size_key must exist in _iso_thread_table.hpp
//   - gland_count in {3, 4}
//   - pocket_w in [40, 120] mm and pocket_h in [20, 60] mm
//   - tapped gland bores fit along the wall (count × pilot dia + clearance)

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pv_junction_box_pottable_pocket {

constexpr const char* kSkillId = "pv_junction_box_pottable_pocket";

struct Input
{
    int         face_id              = -1;
    double      center_x_mm          = 0.0;
    double      center_y_mm          = 0.0;
    double      pocket_w_mm          = 132.0;
    double      pocket_h_mm          = 40.0;
    double      pocket_depth_mm      = 12.0;
    int         gland_count          = 4;
    std::string gland_thread_size_key = "M16";  // ISO metric cable gland
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pv_junction_box_pottable_pocket
}  // namespace koocadcam::skill
