#pragma once
// @lat: [[engine/skills#shot_peen_surface]]
//
// shot_peen_surface — cold-work surface treatment.  A controlled stream of
// hard spherical media (steel shot, ceramic beads, glass beads) impacts the
// target face at high velocity, plastically deforming the surface layer
// and introducing a compressive residual stress field 0.1 – 0.5 mm deep.
// Used to suppress fatigue-crack initiation on highly-stressed components
// (springs, gears, shafts, turbine blades).
//
//        ┌─────────────────────┐
//        │ ░░░░░░░░░░░░░░░░░░░ │  ← compressive-stress surface layer
//        │   (unchanged bulk)  │
//        └─────────────────────┘
//          shape macroscopically identical;
//          surface residual σ ≈ −0.4 to −1.0 σ_yield
//
// Signature pattern:
//   - pattern.kind            = "shot_peen_surface"
//   - pattern.target_face_id  = face that was peened
//   - pattern.intensity_almen = Almen-strip intensity (e.g. 0.012 A)
//   - pattern.coverage_pct    = % of surface struck (target ≥ 100 % = 98 % visually)
//   - pattern.shot_size_mm    = shot media diameter
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT                  intensity_almen / coverage_pct / shot_size_mm ≤ 0 → error
//   DFM-PEEN-INTENSITY         intensity_almen ∉ [0.002, 0.030]    → error
//   DFM-PEEN-COVERAGE          coverage_pct < 50 or > 400          → error
//   DFM-PEEN-SHOT-SIZE         shot_size_mm ∉ [0.1, 3.0]           → info
//
// Synthesis:
//   NO geometric change.  Clones the workpiece carrying a new signature.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace shot_peen_surface {

constexpr const char* kSkillId = "shot_peen_surface";

struct Input
{
    FaceDatum    entry_face;
    double       intensity_almen = 0.012;   // Almen A-strip intensity (in)
    double       coverage_pct    = 200.0;   // ≥ 100 % = 98 % visually struck
    double       shot_size_mm    = 0.8;     // shot media diameter
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace shot_peen_surface
}  // namespace koocadcam::skill
