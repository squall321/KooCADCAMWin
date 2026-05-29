#pragma once
// @lat: [[engine/skills#rotor_balancing]]
//
// rotor_balancing — dynamic balancing of a rotating mass (turbine wheel,
// compressor rotor, propeller hub).  Mass imbalance is measured on a
// balancing machine and corrected by either adding heavy-metal weights or
// removing material at the heavy plane.  The CAD geometry typically does
// NOT capture the correction (it is recorded as a process-step instead),
// so this skill is metadata-only — it stamps the achieved balance grade
// into the workpiece signature for QA traceability.
//
//                     ↻ ω
//                  ╭───────╮
//                  │   ●   │     ← residual imbalance (mass × radius)
//                  │   │   │       expressed as g·mm
//                  ╰───┬───╯
//                      ↑
//                axis of rotation
//
// Signature pattern:
//   - pattern.kind                   = "rotor_balancing"
//   - pattern.mass_imbalance_g_mm
//   - pattern.balance_grade_iso_1940 = "G0.4" | "G1" | "G2.5" | "G6.3" | "G16" | "G40"
//   - pattern.rotation_speed_rpm
//   - pattern.correction_plane_count = 1 | 2
//   - geometry_changed               = false
//
// DFM checks:
//   mass_imbalance_g_mm    > 0                  (DFM-INPUT error if not)
//   balance_grade          ∈ ISO 1940 set       (DFM-BAL-GRADE warning if unknown)
//   rotation_speed_rpm     > 0                  (DFM-INPUT error if not)
//   correction_plane_count ∈ {1, 2}             (DFM-BAL-PLANE warning if outside)
//   imbalance vs grade consistency check        (DFM-BAL-GRADE-MISMATCH info)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
//
// Recognize:
//   Metadata replay only — balancing leaves no B-rep trace.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rotor_balancing {

constexpr const char* kSkillId = "rotor_balancing";

struct Input
{
    double      mass_imbalance_g_mm     = 5.0;       // residual after balance (g·mm)
    std::string balance_grade_iso_1940  = "G2.5";    // ISO 1940/1 quality grade
    double      rotation_speed_rpm      = 10000.0;   // operating speed
    int         correction_plane_count  = 2;         // 1 = static, 2 = dynamic
    double      rotor_mass_kg           = 5.0;       // rotor mass (for grade check)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rotor_balancing
}  // namespace koocadcam::skill
