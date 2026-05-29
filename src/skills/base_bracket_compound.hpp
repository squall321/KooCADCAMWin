#pragma once
// @lat: [[engine/skills#base_bracket_compound]]
//
// base_bracket_compound — COMPOUND structural skill that builds an L-shape
// mounting bracket: two perpendicular plates fused at 90°, 4 mounting holes
// on the base leg, 1 alignment dowel hole on the standing leg, and a
// triangular gusset rib connecting them for stiffness.
//
//   sub-feature 1: base plate   (leg1 × plate_t × bracket_width)
//   sub-feature 2: standing plate(plate_t × leg2 × bracket_width)
//   sub-feature 3..6: 4 mounting holes (bolt_pattern grid on base plate)
//   sub-feature 7: 1 alignment dowel hole on standing leg
//   sub-feature 8: triangular gusset rib (fuse between the two plates)
//
// subfeature_count = 2 + 4 + 1 + 1 = 8.
//
// Geometry:
//   Base plate sits in XY plane:  x ∈ [0, leg1], y ∈ [0, bracket_width], z ∈ [0, plate_t]
//   Standing plate is vertical:   x ∈ [0, plate_t], y ∈ [0, bracket_width], z ∈ [0, leg2]
//
// DFM (DIN 18800 / AISC J6 bracket connections):
//   DFM-BRK-INPUT       : any non-positive input
//   DFM-BRK-PLATE-T     : plate_t < 4 mm (sub-structural)
//   DFM-BRK-BOLT-EDGE   : mounting hole edge distance < 1.5 × bolt_dia
//   DFM-BRK-DOWEL       : dowel hole edge distance < 2 × dowel_dia

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <utility>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace base_bracket_compound {

constexpr const char* kSkillId = "base_bracket_compound";

struct Input
{
    double leg1_mm          = 200.0;   // base-plate horizontal leg
    double leg2_mm          = 150.0;   // standing-plate vertical leg
    double plate_t_mm       = 10.0;
    double bracket_width_mm = 100.0;   // depth in Y for both plates
    // bolt_pattern = (#cols, #rows) on the base plate.
    std::pair<int, int> bolt_pattern   = { 2, 2 };
    double              bolt_dia_mm    = 10.0;
    double              dowel_dia_mm   = 8.0;
    double              gusset_size_mm = 50.0; // gusset rib leg size (= rib triangle leg)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace base_bracket_compound
}  // namespace koocadcam::skill
