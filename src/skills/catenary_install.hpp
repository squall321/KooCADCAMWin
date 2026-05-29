#pragma once
// @lat: [[engine/skills#catenary_install]]
//
// catenary_install — installation of overhead contact-line equipment for
// electrified railways: contact wire, messenger wire, droppers, masts,
// brackets and registration arms.  KooCADCAM records the system voltage
// class (DC 1.5 kV, AC 25 kV, etc.), the design span between supports,
// and the contact-wire height above rail.  Macroscopic B-rep unchanged.
//
//   ╳───────────●───────────●───────────╳   ← masts every span_m
//      ╲     droppers     ╱                  contact wire at height_m
//       ╲────────────────╱                   energised at line_voltage_kv
//
// Signature pattern:
//   - pattern.kind             = "catenary_install"
//   - pattern.line_voltage_kv  = system voltage (kV)
//   - pattern.span_m           = support-to-support span
//   - pattern.height_m         = contact-wire height ATR
//   - pattern.geometry_changed = false
//
// DFM checks:
//   line_voltage_kv > 0                          (DFM-INPUT error)
//   line_voltage_kv ∈ [0.6, 50]                  (DFM-CT-VOLT error if outside)
//   span_m          ∈ [30, 80]                   (DFM-CT-SPAN error if outside)
//   height_m        ∈ [4.7, 6.5]                 (DFM-CT-HEIGHT error if outside)
//
// Synthesis:
//   NO geometric change. Clone + stamp.
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

namespace catenary_install {

constexpr const char* kSkillId = "catenary_install";

struct Input
{
    double  line_voltage_kv = 25.0;   // 1.5 / 3 / 15 / 25 kV common
    double  span_m          = 60.0;   // support-to-support span
    double  height_m        = 5.08;   // contact-wire height above rail
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace catenary_install
}  // namespace koocadcam::skill
