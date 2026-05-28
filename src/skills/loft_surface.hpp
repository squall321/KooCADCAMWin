#pragma once
// @lat: [[engine/skills#loft_surface]]
//
// loft_surface — skin a smooth solid through a sequence of ≥ 2 closed
// cross-section wires, using BRepOffsetAPI_ThruSections.  The sections
// can have arbitrary shape (any closed planar polygon) and need not be
// parallel; ThruSections interpolates between them.
//
//          section 0
//          ┌────┐
//          │    │
//          └────┘
//            │   ◄── lofted surface
//          ┌────────┐
//          │        │
//          └────────┘
//              section 1
//
// Signature pattern:
//   pattern.kind             = "loft_surface"
//   pattern.section_count
//   pattern.section_vertex_count : array, count of vertices per section
//
// DFM:
//   DFM-INPUT          : ≥ 2 sections required.
//   DFM-LOFT-CLOSED    : every section must be a closed polygon
//                        (first == last vertex, or ≥ 3 distinct vertices).
//   DFM-LOFT-COINCIDE  : consecutive sections must not be coincident.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace loft_surface {

constexpr const char* kSkillId = "loft_surface";

struct Input
{
    // sections[k] = closed polyline (last == first OR understood to wrap).
    // Each section must have ≥ 3 unique vertices.
    std::vector<std::vector<gp_Pnt>>  sections;
    bool                              ruled = false;   // true = linear blend
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace loft_surface
}  // namespace koocadcam::skill
