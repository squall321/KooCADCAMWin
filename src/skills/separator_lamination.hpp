#pragma once
// @lat: [[engine/skills#separator_lamination]]
//
// separator_lamination — bond a microporous polymer separator film (PE,
// PP, ceramic-coated) onto the electrode surface to form a stable
// "electrode/separator" sandwich before stacking or winding.  The film
// is fed through a heated roll pair against the coated electrode; the
// adhesive (binder in the separator coating) tacks under heat + pressure.
//
//        ─────────► foil + active coating
//          ━━━━━━━━━━━━━━━━━━━━━━━━━━━━  ← separator (laminated)
//          ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
//          ════════════════════════════
//                  hot roll, ~5 - 10 bar
//
// Signature pattern:
//   - pattern.kind                = "separator_lamination"
//   - pattern.film_thickness_um   = separator film thickness
//   - pattern.lamination_temp_c   = roll surface temperature
//   - pattern.film_material       = "PE" | "PP" | "PE_PP_PE" | "ceramic_coated_PE"
//   - pattern.roll_pressure_bar   = lamination nip pressure
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   DFM-INPUT          film_thickness_um ≤ 0 or lamination_temp_c ≤ 0 (error)
//   DFM-SEP-THK        film_thickness_um < 5 or > 40 (error — outside
//                      industrial separator window)
//   DFM-SEP-TEMP-LOW   lamination_temp_c < 60 °C (error — adhesion too weak)
//   DFM-SEP-TEMP-HIGH  lamination_temp_c > 130 °C (error — risk of
//                      PE pore collapse / thermal shutdown trigger)
//   DFM-SEP-MATERIAL   film_material unrecognised (info)
//   DFM-SEP-PRESSURE   roll_pressure_bar outside [1, 20] bar (info)
//
// Synthesis:
//   NO geometric change — clone the workpiece and stamp the signature.
//
// Recognize:
//   Metadata-only replay, confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace separator_lamination {

constexpr const char* kSkillId = "separator_lamination";

struct Input
{
    double       film_thickness_um  = 20.0;        // typical PE separator
    double       lamination_temp_c  = 90.0;        // mid-range PE softening
    std::string  film_material      = "PE";        // "PE"|"PP"|"PE_PP_PE"|"ceramic_coated_PE"
    double       roll_pressure_bar  = 5.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace separator_lamination
}  // namespace koocadcam::skill
