#pragma once
// @lat: [[engine/skills#deck_assembly]]
//
// deck_assembly — assembly of a single deck level: deck plate, longitudinal
// girders, transverse beams, stiffeners, and outfitting cut-outs are
// tack-fitted onto the previously laid hull section.  KooCADCAM records the
// deck level (1, 2, … or named: "main", "01"), the deck area, and the count
// of structural members on this deck.  The B-rep envelope is unchanged.
//
//                deck_level
//          ────────────────────────
//         ┃                        ┃
//         ┃ ████████████████████ ┃   ← area_m2
//         ┃                        ┃     member_count members
//          ────────────────────────
//                 hull section
//
// Signature pattern:
//   - pattern.kind             = "deck_assembly"
//   - pattern.deck_level       = level name / number (string)
//   - pattern.area_m2          = deck area
//   - pattern.member_count     = count of structural members on this deck
//   - pattern.geometry_changed = false
//
// DFM checks:
//   deck_level    non-empty                     (DFM-INPUT error)
//   area_m2       > 0                           (DFM-INPUT error)
//   member_count  > 0                           (DFM-INPUT error)
//   area_m2       > 5000 ⇒ super-deck, segment & track separately
//                                               (DFM-DECK-AREA info)
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

namespace deck_assembly {

constexpr const char* kSkillId = "deck_assembly";

struct Input
{
    std::string  deck_level   = "main";   // "main" | "01" | "02" | "upper" …
    double       area_m2      = 200.0;    // deck area
    int          member_count = 24;       // count of structural members
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace deck_assembly
}  // namespace koocadcam::skill
