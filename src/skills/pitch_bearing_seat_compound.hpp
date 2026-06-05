#pragma once
// @lat: [[engine/skills#pitch_bearing_seat_compound]]
//
// pitch_bearing_seat_compound — Wind turbine pitch bearing seat.
// The pitch bearing controls blade rotation about its long axis (pitch
// control).  This skill cuts the annular bearing seat pocket (bearing
// inner/outer dia × thickness) plus a bolt circle of mounting holes on the
// PCD around the bearing seat.
//
//   sub-feature 1: annular bearing seat (annular ring pocket)
//   sub-feature 2..N+1: N mounting bolt holes via SEQUENTIAL pr::cut
//
// Bolt-circle holes are placed via gp_Trsf::SetRotation around the bearing
// centerline.  Each bolt hole is cut individually against the running
// workpiece (sequential pr::cut — no compound cutter).
//
// DFM standard cited:
//   IEC 61400-1 — utility blade pitch bearings: inner_dia ~ 2–3 m for
//   utility-scale; bearing thickness / outer_dia ratio 0.05–0.20 typical;
//   bolt count even, ≥ 24 for utility scale.
//
// Signature.pattern:
//   { kind = "pitch_bearing_seat_compound",
//     is_compound = true,
//     wind_feature_type = "pitch_bearing_seat",
//     bolt_count, bearing_inner_dia_mm, bearing_outer_dia_mm,
//     bearing_thickness_mm, bolt_pcd_mm }

#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pitch_bearing_seat_compound {

constexpr const char* kSkillId = "pitch_bearing_seat_compound";

struct Input
{
    int    face_id                  = -1;
    double center_x_mm              = 0.0;
    double center_y_mm              = 0.0;
    double bearing_inner_dia_mm     = 2000.0;  // ~2-3 m for utility blades
    double bearing_outer_dia_mm     = 2200.0;
    double bearing_thickness_mm     = 120.0;
    int    bolt_count               = 60;
    double bolt_pcd_mm              = 2350.0;
    std::string bolt_thread_size_key = "M30";  // M-thread key in _iso_thread_table
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pitch_bearing_seat_compound
}  // namespace koocadcam::skill
