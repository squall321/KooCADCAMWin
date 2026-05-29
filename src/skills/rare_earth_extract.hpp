#pragma once
// @lat: [[engine/skills#rare_earth_extract]]
//
// rare_earth_extract — solvent-extraction / ion-exchange recovery of a
// targeted rare-earth element (Nd, Dy, Pr typical) from end-of-life
// permanent magnets, fluorescent phosphors, or mine tailings.  Pure
// metadata operation: workpiece B-rep is unchanged; a recovery record
// stamps the extracted element mass for supply-chain traceability.
//
//   ┌─────────────────┐  rare_earth_extract   ┌──────────────────┐
//   │ EOL magnet feed │ ────────────────────▶ │ stamp record     │
//   │ feed_kg, target │ (element, recovery%)  │ recovered_kg     │
//   └─────────────────┘                       └──────────────────┘
//
// Signature pattern:
//   - pattern.kind                 = "rare_earth_extract"
//   - pattern.feed_kg
//   - pattern.element_target       = "Nd" | "Dy" | "Pr"
//   - pattern.recovery_pct
//   - pattern.recovered_kg         = feed_kg * recovery_pct / 100
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   element_target  non-empty                    (DFM-INPUT error)
//   element_target  ∈ {Nd, Dy, Pr}               (DFM-REE-TARGET info if unknown)
//   feed_kg         > 0                          (DFM-INPUT error)
//   recovery_pct    ∈ (0, 100]                   (DFM-INPUT error)
//   recovery_pct    ≤ 90 typical pilot ceiling   (DFM-REE-RECOVERY info if exceeded)
//
// Synthesis: NO geometric change.  Clone + stamp signature.
// Recognize: Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rare_earth_extract {

constexpr const char* kSkillId = "rare_earth_extract";

struct Input
{
    double       feed_kg         = 20.0;     // EOL magnet / phosphor feed mass
    std::string  element_target  = "Nd";     // "Nd" | "Dy" | "Pr"
    double       recovery_pct    = 80.0;     // recovered target-element fraction
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rare_earth_extract
}  // namespace koocadcam::skill
