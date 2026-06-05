#pragma once
// @lat: [[engine/skills#diver_case_full_compound]]
//
// diver_case_full_compound — VERY HIGH-ORDER COMPOUND ASSEMBLY (5+ sub-skills).
//
// Complete ISO 6425 dive watch case synthesised from a bare cylindrical stock
// in a SINGLE apply() call by COMPOSING existing slice-12/13 watch skills.
//
// Sub-skill composition chain (sequential pr::cut / pr::fuse via sub-skills):
//   1. stock cylinder (case_dia × case_height)
//   2. lug_integrated_curved (4 lugs at 12+6 o'clock pairs)
//   3. diving_bezel_unidirectional (120-click ISO 6425 bezel seat)
//   4. sapphire_glass_seat (stepped glass bore + O-ring side groove)
//   5. crown_guard_shoulder_compound (boss + clearance at crown position)
//   6. crown_stem_cavity_compound (stem cavity + AS568 -001 O-ring groove)
//   7. case_back_screw_down (screw-down caseback + AS568 -116 O-ring groove)
//
// Output: complete watch case body with FULL ISO 6425 feature set.
// Single FeatureSignature, pattern.is_compound = true, pattern.is_watch_assembly
// = true, pattern.watch_style = "diver", subfeature_count = 6.
//
// DFM gates:
//   - case_dia_mm ∈ [30, 60] mm typical wristwatch
//   - case_height_mm ∈ [8, 18] mm typical dive watch
//   - bezel_outer_dia_mm > case_dia − 2 mm (bezel sits on top)
//   - lug_width_mm ∈ {20, 22, 24}
//   - crown_position ∈ {"3", "4"}
//   - all internal sub-skill DFM passes propagate up
//
// Recognize: metadata replay (very-high-order compound — geometric detection
// impractical).  Confidence 0.5.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace diver_case_full_compound {

constexpr const char* kSkillId = "diver_case_full_compound";

struct Input
{
    double      case_dia_mm        = 42.0;    // typ 38-46 mm
    double      case_height_mm     = 13.0;    // typ 12-15 mm
    double      bezel_outer_dia_mm = 41.0;    // typ < case_dia
    double      lug_width_mm       = 22.0;    // 20 | 22 | 24
    std::string crown_position     = "3";     // "3" | "4"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace diver_case_full_compound
}  // namespace koocadcam::skill
