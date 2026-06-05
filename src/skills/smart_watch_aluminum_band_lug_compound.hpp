#pragma once
// @lat: [[engine/skills#smart_watch_aluminum_band_lug_compound]]
//
// smart_watch_aluminum_band_lug_compound — VERY HIGH-ORDER COMPOUND ASSEMBLY.
//
// Smart watch aluminum case with quick-release band attachment.  Thin,
// rectangular form factor like Apple Watch / Galaxy Watch.  Composes:
//
//   1. stock cuboid (case_w × case_h × case_d)
//   2. fillet corners + top/bottom edges (pr::fuse compensation simplified
//      to cut chamfers — we use 4 corner box cuts to round the silhouette)
//   3. band lug attachment fixture at top and bottom — depends on
//      band_lug_type ("spring_bar" | "slide_in" | "magnetic_clip"):
//         - spring_bar: 2 transverse holes per side
//         - slide_in:   slot pocket per side
//         - magnetic_clip: shallow magnetic clip pocket per side
//   4. display window pocket on top face
//   5. digital crown cavity at 3 o'clock (crown_stem_cavity_compound)
//   6. 2 microphone/speaker grilles drilled into side

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace smart_watch_aluminum_band_lug_compound {

constexpr const char* kSkillId = "smart_watch_aluminum_band_lug_compound";

struct Input
{
    double      case_w_mm        = 41.0;
    double      case_h_mm        = 35.0;
    double      case_d_mm        = 10.0;       // typ 8-12 mm
    std::string band_lug_type    = "spring_bar"; // | "slide_in" | "magnetic_clip"
    double      band_width_mm    = 22.0;       // 20 | 22
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace smart_watch_aluminum_band_lug_compound
}  // namespace koocadcam::skill
