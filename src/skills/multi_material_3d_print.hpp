#pragma once
// @lat: [[engine/skills#multi_material_3d_print]]
//
// multi_material_3d_print — like 3d_print, but the slicer is fed a
// per-region MATERIAL map (rigid frame in PLA + soft seal in TPU + clear
// window in resin etc.).  Multi-tool FDM, polyjet, and metal-binder
// dual-feed printers all fit this model.
//
//   ┌───── region A ──┐ ┌── region B ──┐ ┌──── region C ────┐
//   │  material: pla  │ │ material: tpu│ │ material: petg   │
//   │  layer_pct: 40  │ │ layer_pct: 30│ │ layer_pct: 30    │
//   └─────────────────┘ └──────────────┘ └──────────────────┘
//                     Σ layer_pct ≤ 100
//
// METADATA-ONLY (same as 3d_print).  C++ namespace is `mmp3d_print`
// because identifiers cannot start with a digit.  The skill_id string used
// in signatures is "multi_material_3d_print" verbatim.
//
// Signature pattern:
//   pattern.kind                  == "multi_material_3d_print"
//   pattern.process_type
//   pattern.layer_height_mm
//   pattern.region_count
//   pattern.region_material_map   == [{region_id, material, layer_pct}, ...]
//   pattern.geometric_change      == false
//
// DFM:
//   DFM-MMP3D-REGIONS  : region_material_map non-empty + ≥ 2 regions
//                        (single-region case → use plain 3d_print)
//   DFM-MMP3D-PCTSUM   : Σ layer_pct ∈ [50, 100]  (info if < 100, error if > 100)
//   DFM-MMP3D-LAYER    : layer_height_mm ∈ [0.05, 0.5]
//   DFM-MMP3D-MATERIAL : every region has a non-empty material

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mmp3d_print {

constexpr const char* kSkillId = "multi_material_3d_print";

struct RegionMaterial
{
    std::string region_id      = "region_1";
    std::string material       = "pla";
    double      layer_pct      = 0.0;          // % of total volume / layers
};

struct Input
{
    std::string                 process_type    = "fdm";
    double                      layer_height_mm = 0.2;
    std::vector<RegionMaterial> region_material_map;
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace mmp3d_print
}  // namespace koocadcam::skill
