#pragma once
// @lat: [[engine/skills#profile_extrude]]
//
// profile_extrude — continuous extrusion of complex polymer cross-sections
// (window frames, weatherstrip, channels).  Polymer melt is pushed through
// a profile die, cooled in a calibrator, and pulled by a caterpillar.
// This skill is a RECORD: profile_id refers to a die catalogue entry; the
// run metadata captures cross-section area + take-off speed.
//
//   melt ──▶ ┌─ profile die ─┐ ──▶ calibrator ──▶ haul-off ──▶ saw
//            │   shape: ID   │
//            └───────────────┘
//
// Signature pattern:
//   - pattern.kind                  = "profile_extrude"
//   - pattern.profile_id            = die catalogue identifier
//   - pattern.cross_section_mm2     = profile A
//   - pattern.take_off_speed_m_min  = haul-off speed
//   - pattern.mass_output_kg_h      = derived (ρ × A × v)
//   - pattern.geometry_changed      = false
//
// DFM checks:
//   profile_id           non-empty        DFM-INPUT
//   cross_section_mm2    ∈ (0, 50000]     DFM-PROFILE-AREA
//   take_off_speed_m_min ∈ (0, 60]        DFM-PROFILE-SPEED
//
// Synthesis:
//   NO geometric change — clone + stamp.
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace profile_extrude {

constexpr const char* kSkillId = "profile_extrude";

struct Input
{
    std::string profile_id            = "PRF-0001";
    double      cross_section_mm2     = 250.0;
    double      take_off_speed_m_min  = 5.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace profile_extrude
}  // namespace koocadcam::skill
