#pragma once
// @lat: [[engine/skills#gem_cut]]
//
// gem_cut — primary cutting of a rough gemstone into a finished facet
// arrangement.  The operation is captured as a metadata-only stamp: the
// host workpiece (typically the cradle or dop fixture B-rep) is not
// altered; the cut record carries the species, cut style, and final
// carat weight so downstream polish/grading skills can plan against it.
//
//        ┌─────────────┐                    ┌─────────────┐
//        │  rough      │   gem_cut          │  cut gem    │
//        │  crystal    │ ────────────────▶ │  (faceted)  │
//        │ (host B-rep)│ (species,style,ct) │             │
//        └─────────────┘                    └─────────────┘
//          host shape identical ;  cut record stamped
//
// Cut styles:
//   "round"     — Tolkowsky brilliant, 57/58 facets
//   "oval"      — modified brilliant, elongated outline
//   "princess"  — square modified brilliant, sharp corners
//
// Signature pattern:
//   pattern.kind             = "gem_cut"
//   pattern.gem_species      = "diamond" | "ruby" | "sapphire" | "emerald" | …
//   pattern.cut_style        = "round" | "oval" | "princess"
//   pattern.carat            = double  (1 ct = 0.2 g)
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT       species or style empty (error)
//   DFM-CUT-STYLE   style not in known set (error)
//   DFM-CARAT       carat ≤ 0 (error)
//   DFM-CARAT-LO    carat < 0.005 ct — sub-melee, hand-cut required (info)
//   DFM-CARAT-HI    carat > 100 ct — exceptional stone, custom plan (info)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gem_cut {

constexpr const char* kSkillId = "gem_cut";

struct Input
{
    std::string  gem_species = "diamond";  // diamond | ruby | sapphire | emerald | topaz | …
    std::string  cut_style   = "round";    // round | oval | princess
    double       carat       = 1.0;        // 1 ct = 0.2 g
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace gem_cut
}  // namespace koocadcam::skill
