#pragma once
// @lat: [[engine/skills#blow_film]]
//
// blow_film — blown-film extrusion: polymer melt is pushed through an
// annular die, air is injected to inflate the tube into a bubble, which
// is cooled by an air ring, collapsed into layflat, and wound.  The Blow
// Up Ratio (BUR) = bubble_dia / die_dia drives transverse orientation;
// take-off / extrusion rate ratio drives longitudinal orientation.
//
//                ┌─┐ ◀ air
//   melt ──▶ annular die ──▶ bubble ──▶ collapsing frame ──▶ winder
//                              BUR
//
// Signature pattern:
//   - pattern.kind             = "blow_film"
//   - pattern.bubble_dia_mm    = inflated bubble diameter
//   - pattern.layflat_width_mm = collapsed flat width (π × dia / 2)
//   - pattern.bur              = blow up ratio
//   - pattern.consistency      = layflat vs π·dia/2 cross-check
//   - pattern.geometry_changed = false
//
// DFM checks:
//   bubble_dia_mm    ∈ (0, 3000]            DFM-BLOW-DIA
//   layflat_width_mm ∈ (0, 5000]            DFM-BLOW-LAYFLAT
//   bur              ∈ [1.5, 4.0]           DFM-BLOW-BUR
//   layflat ≈ π × dia / 2 (±10%)            DFM-BLOW-CONSISTENCY (info)
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

namespace blow_film {

constexpr const char* kSkillId = "blow_film";

struct Input
{
    double bubble_dia_mm    = 400.0;
    double layflat_width_mm = 628.0;   // ~ π × 400 / 2
    double bur              = 2.5;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace blow_film
}  // namespace koocadcam::skill
