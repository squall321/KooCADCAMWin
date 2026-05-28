#pragma once
// @lat: [[engine/skills#bolted_joint]]
//
// bolted_joint — assembly-level joint: a clearance through-hole pair (one
// hole drilled in this workpiece) sized for a bolt of the given thread spec,
// with torque & preload metadata describing how the mating part will be
// clamped.  Subtractive on the current workpiece (the through-hole); the
// fastener body and the matching hole on the second part are recorded only
// as metadata for the assembly planner / BOM generator.
//
//                       ┌─ entry_face (planar)
//                       ▼
//                 ──────┬──────────┬──────
//                       │   ▼ axis │
//                       │  ╱│╲     │
//                       │ ╱ │ ╲    │       (clearance through-hole — bolt
//                       │╱  │  ╲   │        passes through this part)
//                       │   │   │  │
//                 ──────┴───┴───┴──┴──────
//                            ↑
//                       hole_dia_mm
//                            ↑
//                     bolt_thread (e.g. "M5")
//
// Signature pattern (what the skill leaves on the workpiece):
//   - 1 cylindrical face (the through-hole bore)
//   - 2 circular edges (entry & exit) — no bottom face (always through)
//   - kind = "bolted_joint", carrying bolt_thread / torque_Nm / preload_N
//
// DFM checks:
//   DFM-BOLT-DIA   : hole_dia_mm derived from bolt_thread must be ≥ 1.6 mm
//   DFM-BOLT-FIT   : clearance_mm ∈ [0.05, 1.0] mm  (industry medium-fit range)
//   DFM-BOLT-TORQUE: torque_Nm > 0
//   DFM-BOLT-PRELD : preload_N  > 0
//
// Recognize:
//   History replay (confidence 1.0).  Geometric fallback iterates cylindrical
//   through-holes whose diameter matches a standard bolt-clearance (M2..M12)
//   plus a typical clearance offset.  Confidence ~0.55 because plain
//   drill_hole at the same diameter is indistinguishable without metadata.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bolted_joint {

constexpr const char* kSkillId = "bolted_joint";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm   = 0.0;
    double      position_y_mm   = 0.0;
    gp_Dir      axis_dir        { 0.0, 0.0, -1.0 };
    std::string bolt_thread     = "M5";   // "M2" .. "M12"
    double      clearance_mm    = 0.5;    // hole_dia = bolt_nom + clearance
    double      torque_Nm       = 0.0;    // assembly tightening torque
    double      preload_N       = 0.0;    // resulting clamp preload
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Helpers exposed for tests.  Returns 0.0 if the thread code is unknown.
double      boltNominalDia(const std::string& bolt_thread);
double      holeDiameterFor(const std::string& bolt_thread, double clearance_mm);

}  // namespace bolted_joint
}  // namespace koocadcam::skill
