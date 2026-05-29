#pragma once
// @lat: [[engine/skills#paint_auto]]
//
// paint_auto — automotive paint-shop pass through E-coat / primer / base /
// clear sequence, each followed by a bake in the curing oven.  Records
// final paint color, total coat count (typ 3–5), and oven temperature.
// Macroscopic B-rep is unchanged; the paint layer thickness is sub-mesh
// and is not reflected in the workpiece shape.
//
//        ┌────────────────────┐    coats × N     ┌────────────────────┐
//        │  body in white     │ ───────────────▶ │ painted body       │
//        │  (after BIW)       │   oven @ T °C    │ (color: <name>)    │
//        └────────────────────┘                  └────────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "paint_auto"
//   - pattern.paint_color      = string
//   - pattern.coat_count       = int
//   - pattern.oven_temp_c      = double
//   - pattern.geometry_changed = false
//
// DFM checks:
//   paint_color non-empty                              (DFM-INPUT error)
//   coat_count ≥ 1                                     (DFM-INPUT error if < 1)
//   coat_count > 6                                     (DFM-PAINT-COATS info)
//   oven_temp_c ∈ [120, 220]                           (DFM-PAINT-OVEN info if outside)
//   oven_temp_c ≤ 0                                    (DFM-INPUT error)
//
// Synthesis:
//   NO geometric change.  Clone shape + material, stamp signature.
//
// Recognize:
//   Metadata-only — no geometric fingerprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace paint_auto {

constexpr const char* kSkillId = "paint_auto";

struct Input
{
    std::string  paint_color = "metallic_silver";
    int          coat_count  = 4;          // e-coat + primer + base + clear
    double       oven_temp_c = 160.0;      // bake temperature
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace paint_auto
}  // namespace koocadcam::skill
