#pragma once
// @lat: [[engine/skills#caseback_o_ring_groove]]
//
// caseback_o_ring_groove — COMPOUND FEATURE (2 primitive ops + metadata).
//
// Water-resistant caseback seal feature: a face-seal annular O-ring groove
// (AS568 standard cross-sections) on the INNER face of the caseback, paired
// with a pilot recess for the screw-back thread engagement.  Chains:
//
//   1. annular groove cut: outer annular ring (rectangular cross-section
//      sized to AS568) centred on case axis at mean_dia.  Outer + inner walls
//      are concentric cylinders → FUSED first then single cut.
//   2. screw-back thread pilot recess: a shallower, larger annular cut
//      OUTSIDE the O-ring groove that acts as the thread engagement starter
//      (the actual thread helix is metadata-only — modelled by a future
//      thread_mill_internal skill).
//
// Output: caseback with concentric annular pocket + pilot recess.  Single
// FeatureSignature, pattern.is_compound = true, subfeature_count = 2.
//
// DFM gates (AS568 + ISO 22810 face-seal):
//   - o_ring_size must be in AS568 small-cross-section catalog:
//     -004 / -006 / -008 / -010 / -012 / -014 (CS = 1.78 mm)
//     -016 / -018 / -020 (CS = 1.78 mm, larger ID)
//   - mean_dia must accommodate the O-ring inner-diameter within ±0.1 mm
//   - groove depth = 0.75 × CS (AS568 design rule)
//   - groove width = 1.30 × CS (AS568 design rule)
//   - pilot recess outer Ø ≤ caseback OD − 1 mm (thread material left)
//
// Recognize: metadata replay; concentric annular grooves are not uniquely
// identifiable.  Confidence 1.0 from history.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace caseback_o_ring_groove {

constexpr const char* kSkillId = "caseback_o_ring_groove";

struct Input
{
    FaceDatum    caseback_face;             // inner face of the caseback
    double       center_x_mm        = 0.0;
    double       center_y_mm        = 0.0;
    gp_Dir       axis_dir           { 0.0, 0.0, 1.0 };  // outward face normal (into caseback)
    double       mean_dia_mm        = 0.0;   // O-ring mean diameter
    std::string  o_ring_size        = "-008"; // AS568 dash-number
    double       thread_pilot_od_mm = 0.0;   // outer Ø of thread pilot recess
    double       thread_pilot_depth_mm = 0.4;
};

// AS568 cross-section lookup.  Returns 0.0 if size unknown.
double aS568CrossSection(const std::string& dash_number);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace caseback_o_ring_groove
}  // namespace koocadcam::skill
