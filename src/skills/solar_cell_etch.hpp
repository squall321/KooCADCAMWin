#pragma once
// @lat: [[engine/skills#solar_cell_etch]]
//
// solar_cell_etch — wet- or dry-chemical etch step in crystalline-silicon
// solar-cell fabrication.  A KOH / NaOH (alkaline) or HF/HNO3 (acidic) bath
// removes saw damage, textures the wafer surface (random pyramids on c-Si),
// and / or strips phosphosilicate glass after diffusion.  At the macroscopic
// CAD level the wafer outline is UNCHANGED; only a few microns of surface
// material are removed and surface morphology is altered.  This skill
// captures the recipe metadata so process traceability + downstream IV
// curve simulation can reference the etch conditions.
//
//          ┌──────────────────────────┐
//          │   raw  c-Si wafer        │
//          │   (saw damage / oxide)   │
//          └────────────┬─────────────┘
//                       │  etchant @ time_min
//                       ▼
//          ┌──────────────────────────┐
//          │   textured wafer         │
//          │   (random pyramids,      │
//          │    PSG stripped, etc.)   │
//          └──────────────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "solar_cell_etch"
//   - pattern.cell_size_mm
//   - pattern.etchant          = "KOH" | "NaOH" | "HF_HNO3" | "TMAH" | …
//   - pattern.time_min
//   - pattern.geometry_changed = false
//
// DFM checks:
//   cell_size_mm > 0                       (DFM-INPUT error)
//   cell_size_mm ∈ [125, 230]              (DFM-PV-CELL-SIZE info — typical M2/M10/M12)
//   etchant non-empty                      (DFM-INPUT warning)
//   etchant in known set                   (DFM-PV-ETCHANT info)
//   time_min > 0                           (DFM-INPUT error)
//   time_min ≤ 60                          (DFM-PV-ETCH-TIME info — overetch risk)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
//
// Recognize:
//   Metadata-only — sub-micron surface texture is not represented in B-rep.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace solar_cell_etch {

constexpr const char* kSkillId = "solar_cell_etch";

struct Input
{
    double       cell_size_mm = 156.75;   // M2 (156.75) | M6 (166) | M10 (182) | M12 (210)
    std::string  etchant      = "KOH";    // "KOH" | "NaOH" | "HF_HNO3" | "TMAH"
    double       time_min     = 15.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace solar_cell_etch
}  // namespace koocadcam::skill
