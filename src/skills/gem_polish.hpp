#pragma once
// @lat: [[engine/skills#gem_polish]]
//
// gem_polish — secondary polishing of a faceted gem to its final lustre.
// After gem_cut establishes the facet geometry, the polish step works
// each facet on progressively finer laps until the target polish class
// is reached.  Metadata-only — the host workpiece geometry is unchanged.
//
//        ┌─────────────┐                       ┌─────────────┐
//        │  cut gem    │   gem_polish          │ polished    │
//        │ (matt facet)│ ────────────────────▶│  gem        │
//        │             │ (species,target,grit) │ (mirror lap)│
//        └─────────────┘                       └─────────────┘
//
// Polish class hierarchy (GIA scale):
//   "excellent"  — no visible polish marks @ 10× loupe
//   "very_good"  — faint marks visible @ 10× loupe
//   "good"       — small marks visible @ 10× loupe
//   "fair"       — marks easily visible
//   "poor"       — heavy marks
//
// Signature pattern:
//   pattern.kind                 = "gem_polish"
//   pattern.gem_species          = string
//   pattern.target_polish_class  = string
//   pattern.lap_grit             = int  (mesh size; higher = finer)
//   pattern.geometry_changed     = false
//
// DFM:
//   DFM-INPUT           species or polish class empty (error)
//   DFM-POLISH-CLASS    target not in {excellent, very_good, good, fair, poor} (error)
//   DFM-LAP-GRIT        lap_grit ≤ 0 (error)
//   DFM-LAP-GRIT-LO     lap_grit < 1200 — too coarse for finish polish (info)
//   DFM-LAP-GRIT-HI     lap_grit > 100000 — diminishing returns (info)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gem_polish {

constexpr const char* kSkillId = "gem_polish";

struct Input
{
    std::string  gem_species         = "diamond";
    std::string  target_polish_class = "excellent";  // excellent | very_good | good | fair | poor
    int          lap_grit            = 50000;        // grit / mesh count
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace gem_polish
}  // namespace koocadcam::skill
