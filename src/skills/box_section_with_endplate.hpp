#pragma once
// @lat: [[engine/skills#box_section_with_endplate]]
//
// box_section_with_endplate — COMPOUND structural skill that builds a
// hollow rectangular tube (RHS / box section) plus two welded endplates
// drilled with a regular bolt-hole pattern.
//
//   sub-feature 1: outer box (length × width × height)
//   sub-feature 2: inner cavity cut (length × (width-2·wall_t) × (height-2·wall_t))
//   sub-feature 3: front endplate (endplate_t × width × height) at x = -endplate_t
//   sub-feature 4: rear  endplate (endplate_t × width × height) at x = length
//   sub-feature 5..N: bolt-hole pattern through BOTH endplates
//                    (bolt_grid.first × bolt_grid.second array)
//
// subfeature_count = 4 + 2 × (n_bolts).
//
// Tube length runs along +X.  Endplates lie in the YZ plane.
//
// DFM (EN 10219 + AWS D1.1):
//   DFM-BOX-INPUT       : any non-positive dimension
//   DFM-BOX-WALL        : wall_t < 1.6 mm or wall_t > min(width, height)/2
//   DFM-BOX-ENDPLATE-T  : endplate_t < wall_t × 1.5 (weld leg constraint)
//   DFM-BOX-BOLT-PITCH  : bolt-hole minimum edge distance < 2 × bolt_dia
//                         (AISC J3.4)

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <utility>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace box_section_with_endplate {

constexpr const char* kSkillId = "box_section_with_endplate";

struct Input
{
    double length_mm     = 600.0;
    double width_mm      = 100.0;
    double height_mm     = 80.0;
    double wall_t_mm     = 5.0;
    double endplate_t_mm = 10.0;
    // bolt_grid = (#cols, #rows) on each endplate (nx, ny).  nx*ny holes per plate.
    std::pair<int, int> bolt_grid     = { 2, 2 };
    double              bolt_dia_mm   = 12.0;
    double              edge_dist_mm  = 25.0;   // distance from plate edge to first hole
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace box_section_with_endplate
}  // namespace koocadcam::skill
