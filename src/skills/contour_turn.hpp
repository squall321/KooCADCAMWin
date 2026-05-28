#pragma once
// @lat: [[engine/skills#contour_turn]]
//
// contour_turn — turning with multiple passes following a smooth contour.
// Similar in spirit to form_turn (revolved profile) but is FORCED to use a
// polyline with > 4 points so it represents a continuously varying contour
// (e.g. a club handle, watch case sweep, ergonomic shaft).
//
// Differences from form_turn:
//   - Min 5 polyline points (≥ 4 line segments).
//   - Records `pass_count` (separate roughing + finishing passes).
//   - Tooling metadata reflects a contour-finishing insert (faster surface
//     speed, smaller feed-per-tooth).
//
// Synthesis: identical to form_turn — close out to the envelope, revolve 360°,
// cut.  Implementation is independent (no shared header) to keep the dependency
// graph flat.
//
// Signature pattern (metadata-prefer recognition):
//   - kind = contour_turn, profile polyline, pass_count.
//
// DFM checks:
//   DFM-INPUT  : ≥ 5 polyline points (this is the defining constraint)
//   DFM-INPUT  : z monotonic, every r ≥ 0.4
//   DFM-LATHE  : every r < stock outer
//   DFM-INPUT  : pass_count ≥ 1

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace contour_turn {

constexpr const char* kSkillId = "contour_turn";

struct ProfilePoint
{
    double z_mm = 0.0;
    double r_mm = 0.0;
};

struct Input
{
    std::vector<ProfilePoint>  profile;
    int                        pass_count = 2;   // ≥ 1
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace contour_turn
}  // namespace koocadcam::skill
