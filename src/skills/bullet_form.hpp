#pragma once
// @lat: [[engine/skills#bullet_form]]
//
// bullet_form — record installation/seating of a finished projectile (bullet)
// into a cartridge case for authorized civilian/commercial small-arms
// manufacturing.  The macroscopic workpiece B-rep represents the assembled
// cartridge: the bullet is a sub-OCCT body inserted into the case neck.
// This skill stamps the installed-projectile metadata so process planning
// and inspection can downstream verify seating depth and crimp.
//
// Signature pattern:
//   - pattern.kind                = "bullet_form"
//   - pattern.caliber_mm          = bullet body diameter
//   - pattern.bullet_weight_grain = projectile mass (grain)
//   - pattern.jacket_material     = "copper" | "gilding_metal" | "lead" | ...
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   caliber_mm           ∈ [3.0, 20.0]    (DFM-AMMO-CAL error if outside)
//   bullet_weight_grain  ∈ [15, 750]      (DFM-AMMO-WEIGHT error if outside)
//   jacket_material      non-empty        (DFM-INPUT warning)
//
// Synthesis: metadata-only.  Clone shape + material, stamp signature.
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bullet_form {

constexpr const char* kSkillId = "bullet_form";

struct Input
{
    double      caliber_mm          = 9.0;
    double      bullet_weight_grain = 115.0;
    std::string jacket_material     = "copper";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bullet_form
}  // namespace koocadcam::skill
