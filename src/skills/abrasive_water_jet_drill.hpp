#pragma once
// @lat: [[engine/skills#abrasive_water_jet_drill]]
//
// abrasive_water_jet_drill — pierce a single circular hole with an abrasive
// water jet.  Unlike `waterjet_cut` (which traverses a 2D path), this skill
// holds the jet stationary at one (x, y) and lets the abrasive stream cut a
// cylindrical bore.  Used for piercing holes in composites / ceramics /
// thick stainless where conventional drilling would delaminate, microcrack,
// or work-harden.
//
// Process characteristics inherit from `waterjet_cut` (kerf 0.6 – 1.2 mm,
// max 200 mm thick, smooth no-HAZ finish) — but here the kerf IS the hole
// diameter, so we validate dia ∈ [0.5, 25] mm against the practical AWJD
// envelope (below 0.5 mm the abrasive stream destabilises; above 25 mm a
// path-traversed circular cut becomes more efficient).
//
//                    ┌─ entry_face (planar)
//                    ▼
//              ──────┬──────────┬──────
//                    │   ▼ axis │
//                    │  ╱│╲     │
//                    │ ╱ │ ╲    │       ↑ through thickness
//                    │╱  │  ╲   │       │ (AWJD is always through-cut)
//                    └───┴───┴──┘       ↓
//                        ↑
//                       dia_mm
//
// Signature pattern (matches `drill_hole` topology since both produce a
// cylindrical bore):
//   - 1 cylindrical face (bore wall, no HAZ)
//   - 2 circular edges (entry + exit)
//   - through-cut (no bottom face)
//   - tool_type = "waterjet"
//
// DFM checks:
//   DFM-002       : diameter_mm ∈ [0.5, 25] mm
//   DFM-THICKNESS : workpiece Z extent ≤ 200 mm (AWJD upper bound)
//   DFM-SURFACE   : info — "smooth, no HAZ" finish
//
// Recognize: shares topology with drill_hole — recognize() detects vertical
// through cylindrical bores and emits an AWJD candidate when 0.5 ≤ dia ≤ 25.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace abrasive_water_jet_drill {

constexpr const char* kSkillId = "abrasive_water_jet_drill";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    double      diameter_mm   = 0.0;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace abrasive_water_jet_drill
}  // namespace koocadcam::skill
