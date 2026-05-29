#pragma once
// @lat: [[engine/skills#insulation_install]]
//
// insulation_install — installation record for building thermal
// insulation (mineral wool, foam, PIR, rockwool, fibreglass).  The
// insulation is a sub-OCCT laminar covering that does NOT alter the
// host workpiece B-rep; it is captured as metadata for U-value /
// thermal-bridge / fire-rating downstream checks.
//
// Signature pattern:
//   - pattern.kind              = "insulation_install"
//   - pattern.insulation_type   = "mineral_wool" | "rigid_foam" | "pir" | "fibreglass" | …
//   - pattern.thickness_mm      = installed layer thickness
//   - pattern.area_m2           = installed area
//   - pattern.r_value           = declared thermal resistance (m^2 K / W)
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   thickness_mm ∈ (0, 500]                 (DFM-INSUL-THK error)
//   area_m2      > 0                        (DFM-INPUT error)
//   r_value      > 0                        (DFM-INPUT error)
//   r_value < 1.0                           (DFM-INSUL-PERF info, sub-code thermal)
//   insulation_type ∈ known set             (DFM-INSUL-MATERIAL info)

#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace insulation_install {

constexpr const char* kSkillId = "insulation_install";

struct Input
{
    std::string insulation_type = "mineral_wool";
    double      thickness_mm    = 100.0;
    double      area_m2         = 50.0;
    double      r_value         = 2.5;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace insulation_install
}  // namespace koocadcam::skill
