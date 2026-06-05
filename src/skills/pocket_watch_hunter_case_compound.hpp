#pragma once
// @lat: [[engine/skills#pocket_watch_hunter_case_compound]]
//
// pocket_watch_hunter_case_compound — VERY HIGH-ORDER COMPOUND ASSEMBLY.
//
// Hunter-style pocket watch with hinged front cover.  Larger diameter than
// wristwatches (45-55 mm), classic chain-bow attachment at top, hinge
// groove + pin at 12 o'clock, 3 movement mounting holes, threaded caseback
// retainer.
//
// Sub-skill composition chain:
//   1. stock cylinder (case_dia × case_thickness)
//   2. hinge groove cut at 12 o'clock + drill hinge pin hole
//   3. bow ring attachment at top (annular fuse + through-bore)
//   4. sapphire_glass_seat (crystal pocket)
//   5. 3 movement mounting holes (drilled through caseback up into case)
//   6. case_back_screw_down (AS568 -116 O-ring) as threaded retainer

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pocket_watch_hunter_case_compound {

constexpr const char* kSkillId = "pocket_watch_hunter_case_compound";

struct Input
{
    double      case_dia_mm           = 50.0;   // 45 | 50 | 55 typical
    double      case_thickness_mm     = 14.0;
    std::string hinge_position        = "12_oclock";
    double      bow_attachment_dia_mm = 8.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pocket_watch_hunter_case_compound
}  // namespace koocadcam::skill
