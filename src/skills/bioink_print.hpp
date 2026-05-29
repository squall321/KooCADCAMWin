#pragma once
// @lat: [[engine/skills#bioink_print]]
//
// bioink_print — bioink-based 3D bioprinting of a cell-laden scaffold.
//
//                                       ┌──────────────────┐
//   ┌─────────────┐                      │ bioink cartridge │
//   │  scaffold   │   bioink_print       │  (cells + gel)   │
//   │   (CAD ──▶  ──────────────────▶   │     ▼ extrude    │
//   │   target    │                      │  layer-by-layer  │
//   │   shape)    │                      └──────────────────┘
//   └─────────────┘                              │
//                                                ▼
//                                       cell-laden hydrogel
//                                       scaffold (geometry
//                                       identical to target)
//
// This is a *process* skill — the CAD scaffold shape is taken as the FINAL
// printed bioink envelope.  Geometry is preserved (no removal, no boolean);
// only the metadata signature is stamped so downstream slicers / CAPP can
// emit deposition toolpaths and incubation/handling instructions.
//
// Signature pattern:
//   pattern.kind                 = "bioink_print"
//   pattern.bioink_type          = "gelma" | "alginate" | "collagen" | "plga" | …
//   pattern.cell_density_per_ml  = double
//   pattern.layer_height_um      = double  (typical 100–400 μm for hydrogel)
//   pattern.geometric_change     = false
//   pattern.process_family       = "additive_bioprinting"
//
// DFM:
//   DFM-INPUT              bioink_type empty
//   DFM-BIOINK-LAYER       layer_height_um ∈ [50, 1000] μm
//   DFM-BIOINK-DENSITY     cell_density_per_ml ∈ [1e3, 1e8]
//   DFM-BIOINK-VIABILITY   bioink_type unknown (info)
//
// Recognize: metadata-only — analytic B-Rep gives no signal that the part
// is a cell-laden hydrogel scaffold.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bioink_print {

constexpr const char* kSkillId = "bioink_print";

struct Input
{
    std::string  bioink_type         = "gelma";
    double       cell_density_per_ml = 1.0e6;     // cells / mL
    double       layer_height_um     = 200.0;     // μm
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace bioink_print
}  // namespace koocadcam::skill
