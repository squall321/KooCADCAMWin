#pragma once
// @lat: [[engine/skills#drum_lug_seat_compound]]
//
// drum_lug_seat_compound — drum tension lug seat with threaded insert recess.
//
// Slice 16 — audio / musical instrument hardware.
//
// Sub-features (sequential CUTs):
//   1. threaded pilot hole (cut cylinder = UNC tap pilot from central table)
//   2. counterbore for threaded insert (larger dia, shallower depth)
//
// subfeature_count = 2.
//
// lug_size selects the typical UNC thread:
//   "small" → "8-32"   (12 inch toms)
//   "large" → "12-24"  (kick drum / large toms / floor toms)
//
// Custom thread_size_key from UNC central table allowed (overrides lug_size
// default if non-empty).
//
// DFM gates:
//   DFM-INPUT         : positive dims
//   DFM-LUG-SIZE      : lug_size in {"small", "large"}
//   DFM-THREAD-KEY    : thread_size_key (if provided) must exist in UNC table
//   DFM-INSERT-DIA    : insert recess dia must be > pilot dia + 2 mm

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace drum_lug_seat_compound {

constexpr const char* kSkillId = "drum_lug_seat_compound";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm            = 0.0;
    double      center_y_mm            = 0.0;
    gp_Dir      axis_dir               { 0.0, 0.0, -1.0 };
    std::string lug_size               = "small";   // "small" | "large"
    std::string thread_size_key;                    // optional override
    double      tap_depth_mm           = 10.0;
    double      insert_recess_dia_mm   = 9.0;
    double      insert_recess_depth_mm = 4.0;
};

// Default UNC thread for given lug_size.
std::string defaultThreadFor(const std::string& lug_size);

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace drum_lug_seat_compound
}  // namespace koocadcam::skill
