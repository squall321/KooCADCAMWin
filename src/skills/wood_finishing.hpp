#pragma once
// @lat: [[engine/skills#wood_finishing]]
//
// wood_finishing — surface finishing of a wooden workpiece using varnish,
// stain, oil, lacquer, or wax.  METADATA-ONLY skill: the macroscopic shape
// is unchanged.  Adds a thin film (< 0.05 mm) and changes the SURFACE
// properties (colour, sheen, water resistance, hardness).
//
//         ┌─────────────────┐                ┌─────────────────┐
//         │  bare wood      │   finishing    │ ░░░░░░░░░░░░░░░ │
//         │  (raw grain)    │  ───────────►  │ ▓ stained ▓     │
//         │                 │  (n coats)     │ ░ + sealed      │
//         └─────────────────┘                └─────────────────┘
//
// Signature pattern:
//   pattern.kind            = "wood_finishing"
//   pattern.finish_type     = "varnish" | "stain" | "oil" | "lacquer" | "wax"
//   pattern.coats           = number of coats applied
//   pattern.cure_hours      = total cure time per coat × coats
//   pattern.surface_sheen   = "matte" | "satin" | "semi_gloss" | "gloss"
//   pattern.water_resistance = "low" | "medium" | "high"
//   pattern.geometry_changed = false
//
// DFM checks:
//   DFM-FINISH-TYPE       finish_type not in supported list (error)
//   DFM-FINISH-COATS-MIN  coats < 1                       (error)
//   DFM-FINISH-COATS-MAX  coats > 6                       (warn — diminishing return)
//   DFM-FINISH-CURE       cure_hours_per_coat <= 0         (error)
//
// Recognize:
//   Surface chemistry is below the OCCT B-rep level.  Recognition is
//   metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace wood_finishing {

constexpr const char* kSkillId = "wood_finishing";

struct Input
{
    std::string  finish_type           = "varnish";  // "varnish"|"stain"|"oil"|"lacquer"|"wax"
    int          coats                 = 2;
    double       cure_hours_per_coat   = 8.0;
    std::string  surface_sheen         = "satin";    // "matte"|"satin"|"semi_gloss"|"gloss"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace wood_finishing
}  // namespace koocadcam::skill
