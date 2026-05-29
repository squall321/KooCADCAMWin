#pragma once
// @lat: [[engine/skills#ballast_install]]
//
// ballast_install — installation of permanent or fixed ballast (solid pig
// iron, concrete blocks, or steel punching) into ballast tanks / void
// spaces to trim the vessel.  KooCADCAM records the ballast type, total
// installed mass (tonnes), and the count of discrete ballast locations.
// The hull B-rep is unchanged.
//
//      ┃   ───────────────────┒
//      ┃   ▓▓▓▓ ▓▓▓▓ ▓▓▓▓     ┃   ← locations_count blocks
//      ┃   total_mass_t kg     ┃      of ballast_type
//      ┃   ───────────────────┒
//
// Signature pattern:
//   - pattern.kind              = "ballast_install"
//   - pattern.ballast_type      = "pig_iron" | "concrete" | "steel_punching" …
//   - pattern.total_mass_t      = total installed mass (tonnes)
//   - pattern.locations_count   = number of discrete install locations
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   ballast_type      non-empty                 (DFM-INPUT error)
//   total_mass_t      > 0                       (DFM-INPUT error)
//   locations_count   > 0                       (DFM-INPUT error)
//   total_mass_t      > 500   ⇒ heavy lift planning advisory
//                                               (DFM-BALLAST-MASS info)
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

namespace ballast_install {

constexpr const char* kSkillId = "ballast_install";

struct Input
{
    std::string  ballast_type    = "pig_iron";  // "pig_iron" | "concrete" | "steel_punching"
    double       total_mass_t    = 50.0;        // tonnes
    int          locations_count = 4;           // discrete install bays
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ballast_install
}  // namespace koocadcam::skill
