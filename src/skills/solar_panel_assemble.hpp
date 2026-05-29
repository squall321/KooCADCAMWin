#pragma once
// @lat: [[engine/skills#solar_panel_assemble]]
//
// solar_panel_assemble — photovoltaic module assembly record.  A solar
// panel laminates N PV cells between front glass, EVA/POE encapsulant,
// and a back-sheet; cells are tabbed & strung in series.  The macroscopic
// panel frame outline is unchanged by this step; cell layout & encapsulant
// chemistry are stamped as metadata.
//
//   ┌──────────────── glass ──────────────┐
//   │  ▓  ▓  ▓  ▓  ▓  ▓  …  ▓  (cells)    │   N cells × cell_size × cell_size
//   │  ─────────────── encapsulant ────── │
//   │  ─────────────── back-sheet ─────── │
//   └────────────────────────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "solar_panel_assemble"
//   - pattern.cell_count       = N
//   - pattern.cell_size_mm     = square cell side (e.g. 158.75, 166, 182, 210)
//   - pattern.encapsulant      = "EVA" | "POE" | "TPO"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   cell_count   > 0              (DFM-INPUT error)
//   cell_size_mm ∈ [125, 230]     (DFM-PV-CELL-SIZE error if outside)
//   encapsulant  ∈ {EVA,POE,TPO}  (DFM-PV-ENCAP info if unknown)
//
// Synthesis: no geometric change — clone + stamp.
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace solar_panel_assemble {

constexpr const char* kSkillId = "solar_panel_assemble";

struct Input
{
    int         cell_count   = 60;
    double      cell_size_mm = 166.0;
    std::string encapsulant  = "EVA";   // "EVA" | "POE" | "TPO"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace solar_panel_assemble
}  // namespace koocadcam::skill
