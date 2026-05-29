#pragma once
// @lat: [[engine/skills#separator_install]]
//
// separator_install — surface facility installation of a 2- or 3-phase
// production separator vessel.  Captures separator type, design capacity,
// and rated working pressure for a vessel placed on a production pad.
// This is a facility-installation record; the workpiece model (separator
// shell or skid) is NOT geometrically modified.
//
//          ┌──────────────────────────────────────┐
//          │   gas  ──▶                           │
//          │   oil  ──▶  ←  separator_type        │
//          │   water ─▶  ←  capacity_bpd          │
//          │             ←  working_pressure_kpa  │
//          └──────────────────────────────────────┘
//
// Signature pattern:
//   - pattern.kind                 = "separator_install"
//   - pattern.separator_type       = "2-phase" | "3-phase"
//   - pattern.capacity_bpd
//   - pattern.working_pressure_kpa
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   separator_type ∈ {"2-phase","3-phase"}       (DFM-SI-TYPE info if unknown)
//   capacity_bpd > 0                             (DFM-INPUT error)
//   capacity_bpd > 100000                        (DFM-SI-CAP info — very large vessel)
//   working_pressure_kpa > 0                     (DFM-INPUT error)
//   working_pressure_kpa > 20000                 (DFM-SI-PRESS info — HP class)
//   working_pressure_kpa < 100                   (DFM-SI-PRESS info — atmospheric)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
//
// Recognize:
//   Metadata-only — installed vessel ID is a facility record, not B-rep.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace separator_install {

constexpr const char* kSkillId = "separator_install";

struct Input
{
    std::string  separator_type        = "3-phase";   // "2-phase" | "3-phase"
    double       capacity_bpd          = 10000.0;     // barrels per day
    double       working_pressure_kpa  = 5000.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace separator_install
}  // namespace koocadcam::skill
