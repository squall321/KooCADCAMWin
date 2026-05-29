#pragma once
// @lat: [[engine/skills#sheet_extrude]]
//
// sheet_extrude — flat polymer sheet extrusion: melt is pushed through a
// wide slot die, polished and gauged by a three-roll stack, then trimmed
// to width.  Distinguished from film_extrude by sheet THICKNESS (mm, not
// µm) — sheet is rigid (≥ 0.25 mm), film is flexible (< 0.5 mm).
//
//   melt ──▶ ┌─ slot die ─┐ ──▶ 3-roll stack ──▶ haul-off ──▶ saw
//            │   wide      │      t_mm
//            └─────────────┘
//
// Signature pattern:
//   - pattern.kind                 = "sheet_extrude"
//   - pattern.sheet_thickness_mm   = polished sheet thickness
//   - pattern.width_mm             = trimmed width
//   - pattern.line_speed_m_min     = haul-off speed
//   - pattern.mass_output_kg_h     = derived
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   sheet_thickness_mm ∈ [0.25, 30.0]   DFM-SHEET-THICKNESS
//   width_mm           ∈ (0, 3000]      DFM-SHEET-WIDTH
//   line_speed_m_min   ∈ (0, 100]       DFM-SHEET-SPEED
//
// Synthesis:
//   NO geometric change — clone + stamp.
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

namespace sheet_extrude {

constexpr const char* kSkillId = "sheet_extrude";

struct Input
{
    double sheet_thickness_mm = 3.0;
    double width_mm           = 1500.0;
    double line_speed_m_min   = 10.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sheet_extrude
}  // namespace koocadcam::skill
