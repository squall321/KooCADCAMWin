#pragma once
// @lat: [[engine/skills#propulsion_install]]
//
// propulsion_install — installation of the prime mover (main engine,
// electric pod, water-jet, azimuth thruster) coupled to its drive shaft.
// KooCADCAM records the propulsion type, installed power (kW) and the main
// shaft diameter.  Shaft alignment, foundation, and stern-tube bearings are
// downstream skills; this skill only stamps the propulsion record.
//
//        ┏━━━━━━━━━━━━━━━┓        ┏━━━━━━━━━┓
//        ┃               ┃═══════ ┃         ┃ →
//        ┃   prime mover ┃ shaft  ┃ propeller┃
//        ┃   power_kw    ┃ Ø dia  ┃         ┃
//        ┗━━━━━━━━━━━━━━━┛        ┗━━━━━━━━━┛
//
// Signature pattern:
//   - pattern.kind              = "propulsion_install"
//   - pattern.propulsion_type   = "diesel" | "pod" | "waterjet" | "azimuth" …
//   - pattern.power_kw          = installed mechanical / electrical power
//   - pattern.shaft_dia_mm      = main shaft Ø
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   propulsion_type   non-empty                 (DFM-INPUT error)
//   power_kw          > 0                       (DFM-INPUT error)
//   shaft_dia_mm      ∈ [50, 1500]              (DFM-PROP-SHAFT error if outside)
//   power_kw          > 50000 ⇒ class-society
//                                  review advisory
//                                               (DFM-PROP-POWER info)
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

namespace propulsion_install {

constexpr const char* kSkillId = "propulsion_install";

struct Input
{
    std::string  propulsion_type = "diesel";   // "diesel" | "pod" | "waterjet" | "azimuth"
    double       power_kw        = 5000.0;     // installed kW
    double       shaft_dia_mm    = 250.0;      // main shaft Ø
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace propulsion_install
}  // namespace koocadcam::skill
