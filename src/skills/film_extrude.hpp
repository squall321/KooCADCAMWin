#pragma once
// @lat: [[engine/skills#film_extrude]]
//
// film_extrude — cast film extrusion: polymer melt is pushed through a
// slot die onto a chilled roll, producing a thin continuous film that is
// trimmed and wound.  This skill is a process RECORD: the workpiece is
// the source stock; macroscopic B-rep does not change.  The downstream
// film roll is captured as metadata (thickness, width, line speed).
//
//   melt ──▶ ┌─── slot die ───┐ ──▶ chill roll ──▶ winder
//            │                │      film, t_µm
//            └────────────────┘      width_mm
//
// Signature pattern:
//   - pattern.kind                 = "film_extrude"
//   - pattern.film_thickness_um    = target film thickness (µm)
//   - pattern.width_mm             = trimmed film width
//   - pattern.line_speed_m_min     = take-off line speed
//   - pattern.melt_output_kg_h     = derived throughput estimate
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   film_thickness_um  ∈ [5, 500]      DFM-FILM-THICKNESS
//   width_mm           ∈ (0, 4000]     DFM-FILM-WIDTH
//   line_speed_m_min   ∈ (0, 800]      DFM-FILM-SPEED
//
// Synthesis:
//   NO geometric change — clone shape + material, stamp signature.
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

namespace film_extrude {

constexpr const char* kSkillId = "film_extrude";

struct Input
{
    double film_thickness_um = 50.0;
    double width_mm          = 1200.0;
    double line_speed_m_min  = 60.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace film_extrude
}  // namespace koocadcam::skill
