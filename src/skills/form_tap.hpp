#pragma once
// @lat: [[engine/skills#form_tap]]
//
// form_tap — cold-forming (chipless) tap.  Instead of cutting V-flutes the
// tool DISPLACES material plastically into the thread shape — there are
// no chips, the thread is stronger (cold-worked surface), and tap life is
// 10-20× a cutting tap.  Restricted to ductile materials.
//
// Geometrically the resulting thread looks the same as a cutting tap
// (V-thread helix on the bore wall), so for slice 1 the synthesis is
// identical to rigid_tap.  The skill distinguishes itself via:
//
//   1. A DIFFERENT pilot-diameter formula:
//        form_tap pilot ≈ nominal_dia − 0.5 × pitch
//        (vs cutting tap pilot ≈ nominal_dia − pitch)
//      → the pilot is BIGGER because no material is removed; only the
//        crest is rolled inward.
//
//   2. A material-restriction DFM info finding ("form_tap requires
//      ductile material"): only aluminum, copper, brass, low-carbon
//      steel, mild stainless are allowed without warning.  Hardened steel
//      / cast iron / titanium emit an info-level note.
//
//   3. Tooling metadata: tool_type = "form_tap", no flutes, no chip
//      evacuation in the NC cycle.
//
// Signature pattern:
//   - Same V-thread groove as tap_thread / rigid_tap.  Distinguished by
//     `pattern.kind == "form_tap"` and the form-tap pilot diameter recorded
//     in pattern.pilot_diameter_mm.
//
// DFM checks:
//   DFM-TAP-ENGAGE   : thread_depth_mm ≥ 1.5 × pitch_mm.
//   DFM-FORM-TAP-DIA : pilot diameter must match the FORM-TAP chart for
//                      thread_size (different from cutting-tap chart).
//   DFM-FORM-TAP-MAT : material must be ductile (info-level finding only
//                      on non-ductile material so caller can override).
//
// Recognize:
//   Same topology as tap_thread; recognition is metadata-driven.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace form_tap {

constexpr const char* kSkillId = "form_tap";

struct Input
{
    FaceDatum   existing_hole_datum;       // pilot bore cylindrical face
    std::string thread_size      = "M3";
    double      pitch_mm         = 0.5;
    double      thread_depth_mm  = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Recommended FORM-TAP pilot diameter.  Distinct from the cutting-tap
// chart: pilot ≈ nominal − 0.5 × pitch.  Returns 0.0 for unknown sizes.
double formTapPilotDiameter(const std::string& thread_size);

// Recommended cutting-tap pilot for reference (used in the DFM info
// finding's narrative).
double standardCoarsePitch(const std::string& thread_size);

// True if `material` is in the form-tap-approved ductile list.
bool isDuctileMaterial(const std::string& material);

}  // namespace form_tap
}  // namespace koocadcam::skill
