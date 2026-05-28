#pragma once
// @lat: [[engine/skills#peen_form]]
//
// peen_form — shot peening for FORMING (not surface hardening alone).
// A high-velocity stream of small steel/ceramic shot blasts one side of
// a sheet; the impact plastically expands the surface in compression,
// curving the sheet AWAY from the peened face.  Used industrially for
// wing skins and large aerospace panels where conventional press-forming
// would induce springback.
//
//        ▼▼▼▼▼▼▼▼  shot stream
//        ──────────  flat sheet  ──►  peening  ──►   curved sheet (concave up)
//                                                    radius ≈ 1 / curvature
//
// Geometric model (slice-1):
//   For now this skill is a METADATA-ONLY operation.  The macroscopic shape
//   change (small curvature over a large panel) requires either non-trivial
//   sheet deformation or stand-in mesh rebuild — both are beyond slice-1.
//   The skill records the forming intent (intensity + target curvature)
//   so downstream consumers (FEA, CAM, QA) can pick it up.  The shape is
//   passed through unchanged; verification is via metadata replay.
//
// Signature pattern:
//   - kind                          = peen_form
//   - peening_intensity             = Almen-A intensity (0.10 ~ 0.30 mm)
//   - target_curvature_mm_inv       = 1 / radius_of_curvature in mm⁻¹
//   - peen_face                     = (informational) FaceDatum
//   - geometry_changed              = false (slice-1)
//
// DFM checks:
//   DFM-INPUT          : peening_intensity > 0
//   DFM-INPUT          : target_curvature_mm_inv > 0
//   DFM-PEEN-INTENS    : peening_intensity outside [0.05, 0.50] Almen mm
//                        — warning (range covers all industrial practice)
//   DFM-PEEN-CURV      : target_curvature_mm_inv > 0.01 (R < 100 mm) — warning
//                        (very tight curvature exceeds peen-forming capability)
//
// Recognize:
//   Metadata-only.  Geometry cannot reveal a prior peen-form pass; we replay
//   from `workpiece.features()`.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace peen_form {

constexpr const char* kSkillId = "peen_form";

struct Input
{
    FaceDatum  peen_face;                          // face being peened (informational)
    double     peening_intensity        = 0.20;    // Almen-A intensity in mm
    double     target_curvature_mm_inv  = 0.001;   // 1/R in mm⁻¹ (R = 1000 mm → 0.001)
    double     coverage_pct             = 200.0;   // typical 150-300% (Almen scale)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace peen_form
}  // namespace koocadcam::skill
