#pragma once
// @lat: [[engine/skills#cylindrical_grind]]
//
// cylindrical_grind — EXTERNAL precision grind on an outer cylindrical
// surface (a shaft / OD).  Reduces the outer diameter by removal_mm,
// improving roundness and surface finish.  Typical removal: 0.005–0.1 mm
// radial, Ra ≈ 0.2 µm.
//
//      Before:                                  After (radius - removal_mm):
//
//          ┌─────┐                                  ┌───┐
//          │     │                                  │   │
//          │     │                                  │   │
//          │     │              ─→                  │   │
//          │     │                                  │   │
//          │     │                                  │   │
//          └─────┘                                  └───┘
//          R                                       R − Δ
//
// Synthesis:
//   The keeper is the smaller cylinder (radius − removal_mm) — but we model
//   it as a subtraction of the OUTER ANNULAR RING between the old and new
//   radii over the cylinder's axial extent.  An ε margin is added to the
//   annulus outer radius to guarantee a clean Boolean intersection with the
//   existing outer surface.
//
// DFM checks:
//   removal_mm ∈ [0.005, 0.1]
//   resulting radius (old − removal_mm) must be > 0
//
// Recognize:
//   Same geometry as a plain cylindrical face.  Without process metadata we
//   cannot tell "turned + ground" from "turned" alone.  The convention is
//   that ground OD radii tend to land on "round" precision values (e.g.
//   ending in .500 / .250 / .000); recognize() emits LOW-confidence
//   candidates only for cylindrical faces whose radius is near such a
//   value.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cylindrical_grind {

constexpr const char* kSkillId = "cylindrical_grind";

struct Input
{
    // EXISTING outer cylindrical face on the workpiece.
    FaceDatum     cylindrical_face_datum;
    // Radial removal (Δ-radius).  Resulting radius = old − removal_mm.
    double        removal_mm = 0.02;
    std::string   surface_finish = "ra_0.4";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cylindrical_grind
}  // namespace koocadcam::skill
