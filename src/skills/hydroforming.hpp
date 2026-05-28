#pragma once
// @lat: [[engine/skills#hydroforming]]
//
// hydroforming — use high-pressure fluid to form a sheet against a die.
// Geometrically identical to deep_draw (a cup with cylindrical wall +
// rounded bottom), but distinguished by its signature.kind = "hydroforming"
// and by the additional `pressure_mpa` metadata.
//
// Real hydroforming produces thinner, more uniform walls than deep draw at
// the cost of capital equipment (1000-10000-tonne hydraulic presses).  We
// model the geometric result identically to deep_draw — the differentiation
// is metadata-only and downstream layers can compute wall-thinning models
// from pressure if needed.
//
//                           ↑ cup_depth_mm
//                  ┌─────┐  │
//                  │     │  │  ← thinner wall (less than sheet_thickness
//                  │     │  │    under high pressure)
//                  └─────┘  ↓
//                  ──────────────────────────  ← original sheet plane
//
// Synthesis strategy:
//   Same as deep_draw — produce a cup-shaped feature.  Differentiate by
//   tagging the signature with pressure_mpa and skill_id = "hydroforming".
//
// Signature pattern:
//   - same topology as deep_draw
//   - pattern.kind = "hydroforming"
//   - pattern.pressure_mpa recorded
//
// DFM checks:
//   DFM-HF-PRESS    : pressure_mpa ∈ [50, 600] (typical hydroform range)
//   DFM-HF-RATIO    : cup_depth / cup_dia ≤ 1.0 (looser than deep_draw)
//   DFM-HF-CORNER   : corner_r ≥ 2 × sheet_thickness (smaller than deep_draw
//                    because hydraulic pressure conforms to die better)
//   DFM-HF-SHEET    : sheet thickness ≤ 5 mm
//   DFM-INPUT       : cup_dia > 0, cup_depth > 0
//
// Recognize:
//   Identical to deep_draw (cylindrical protrusion).  The hydroforming
//   recognizer returns lower confidence than deep_draw unless the workpiece
//   feature history shows a prior `hydroforming` entry.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hydroforming {

constexpr const char* kSkillId = "hydroforming";

struct Input
{
    FaceDatum   entry_face;                 // sheet top
    double      cup_position_x_mm = 0.0;
    double      cup_position_y_mm = 0.0;
    double      cup_dia_mm        = 0.0;
    double      cup_depth_mm      = 0.0;
    double      corner_r_mm       = 0.0;
    double      sheet_thickness_mm = 0.0;   // 0 → auto-detect from bbox
    double      pressure_mpa      = 200.0;  // typical 50..600
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

double sheetThickness(const Workpiece& wp);

}  // namespace hydroforming
}  // namespace koocadcam::skill
