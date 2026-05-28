#pragma once
// @lat: [[engine/skills#lens_grind]]
//
// lens_grind — abrasive shaping of a curved optical surface using bonded
// diamond pellets or loose abrasive slurries.  Establishes the surface
// figure (radius of curvature) and the working aperture before fine
// polishing.  Macroscopic OCCT geometry is unchanged in phase-1; the
// process is recorded as metadata (radius of curvature, aperture, ISO
// 10110 surface quality grade) on the FeatureSignature.
//
//        ┌─────────────┐                       ┌─────────────┐
//        │             │     lens_grind        │   ◠◠◠◠◠     │  ← ground curve
//        │   blank     │  ────────────────▶   │             │
//        │   (flat)    │  (R, aperture, qual)  │   blank     │
//        │             │                       │             │
//        └─────────────┘                       └─────────────┘
//          shape identical in metadata-only phase;  surface_quality recorded
//
// Signature pattern:
//   - pattern.kind                 = "lens_grind"
//   - pattern.target_face_id       = face that was ground
//   - pattern.radius_curvature_mm  = signed (convex > 0, concave < 0, plano = 0)
//   - pattern.aperture_mm          = clear aperture diameter
//   - pattern.surface_quality      = ISO 10110-7 scratch/dig string (e.g. "5/3x0.10")
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   aperture_mm > 0                                 (error)
//   |radius_curvature_mm| ≥ aperture_mm/2 OR plano  (error — sphere otherwise impossible)
//   surface_quality non-empty                       (warning)
//   ISO 10110-7 format hint                         (info)
//
// Recognize: metadata-only.  Returns signatures filtered by skill_id.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lens_grind {

constexpr const char* kSkillId = "lens_grind";

struct Input
{
    FaceDatum   entry_face;
    double      radius_curvature_mm = 100.0;  // 0 = plano, >0 convex, <0 concave
    double      aperture_mm         = 25.0;   // clear aperture (diameter)
    std::string surface_quality     = "5/3x0.10";  // ISO 10110-7 scratch/dig
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lens_grind
}  // namespace koocadcam::skill
