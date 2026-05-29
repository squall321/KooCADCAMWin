#pragma once
// @lat: [[engine/skills#deck_cast]]
//
// deck_cast — cast the bridge deck slab between two piers.  A deck is
// described by its `span_m` (pier-to-pier length), the deck surface
// `area_m2`, and the rebar density `rebar_kg_m3` used to meet bending
// and shrinkage requirements.  KooCADCAM records this as metadata.
//
//          ════════════════════════════════════     ← cast deck slab
//          ║  span_m, area_m2, rebar_kg_m3   ║
//          ════════════════════════════════════
//             ║                            ║
//             ║                            ║         piers
//
// Signature pattern:
//   - pattern.kind             = "deck_cast"
//   - pattern.span_m           = pier-to-pier span (m)
//   - pattern.area_m2          = cast surface area (m2)
//   - pattern.rebar_kg_m3      = rebar density (kg/m3 of concrete)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   span_m       > 0                                (DFM-INPUT error)
//   area_m2      > 0                                (DFM-INPUT error)
//   rebar_kg_m3  > 0                                (DFM-INPUT error)
//   span_m       > 100  ⇒ long-span special review  (DFM-DECK-SPAN info)
//   rebar_kg_m3  < 60   ⇒ very light reinforcement  (DFM-DECK-REBAR info)
//   rebar_kg_m3  > 350  ⇒ very dense reinforcement  (DFM-DECK-REBAR info)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace deck_cast {

constexpr const char* kSkillId = "deck_cast";

struct Input
{
    double  span_m        = 30.0;     // pier-to-pier span (m)
    double  area_m2       = 240.0;    // deck cast surface area (m2)
    double  rebar_kg_m3   = 150.0;    // rebar density (kg/m3 of concrete)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace deck_cast
}  // namespace koocadcam::skill
