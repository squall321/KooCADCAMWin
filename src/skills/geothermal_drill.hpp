#pragma once
// @lat: [[engine/skills#geothermal_drill]]
//
// geothermal_drill — record of a geothermal-well drilling operation.
// Distinct from rock_drill / drill_hole because the relevant scale is
// kilometres, not millimetres, and the completion method (open hole vs
// cased + slotted liner vs cased + perforated) drives downstream
// production-test plans.
//
//   surface ─────────────────────────────────────
//        │ casing
//        │
//        │     conductive geothermal gradient ~30 °C/km
//        │
//        │
//        ▼  well_depth_m  (typ 1500..4500 m)
//      ╲╱╲╱╲╱   reservoir @ bottom_hole_temp_c
//      (open hole | slotted liner | perforated casing)
//
// Signature pattern:
//   - pattern.kind             = "geothermal_drill"
//   - pattern.well_depth_m
//   - pattern.bottom_hole_temp_c
//   - pattern.completion_method  = "open_hole" | "slotted_liner" | "perforated_casing"
//   - pattern.geometry_changed = false   (skill is a metadata record;
//                                         the workpiece B-rep here is the
//                                         CAD of the wellhead, not the bore)
//
// DFM checks:
//   well_depth_m > 0                             (DFM-INPUT error)
//   well_depth_m ∈ [500, 6000]                   (DFM-GEO-DEPTH info)
//   bottom_hole_temp_c > 0                       (DFM-INPUT error)
//   bottom_hole_temp_c ≥ 120                     (DFM-GEO-TEMP info — below this
//                                                economic electricity-grade is hard)
//   completion_method in known set               (DFM-GEO-COMPL info)
//   gradient ~ (BHT - 20) / depth_km roughly
//   between 15 and 90 °C/km                      (DFM-GEO-GRADIENT info)
//
// Synthesis:
//   NO geometric change to workpiece — clone + stamp signature.
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace geothermal_drill {

constexpr const char* kSkillId = "geothermal_drill";

struct Input
{
    double       well_depth_m        = 2500.0;
    double       bottom_hole_temp_c  = 200.0;
    std::string  completion_method   = "slotted_liner";
    // "open_hole" | "slotted_liner" | "perforated_casing"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace geothermal_drill
}  // namespace koocadcam::skill
