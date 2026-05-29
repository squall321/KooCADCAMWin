#pragma once
// @lat: [[engine/skills#adjuster_screw_pocket_compound]]
//
// adjuster_screw_pocket_compound — COMPOUND MACRO for a finely-adjustable
// screw seat with a captive lock-nut and an indicator scribe groove.
//
// Sub-feature chain (3 concentric / co-axial geometries):
//   1. Fine-pitch tap pilot hole       — pilot_dia from fine_thread (M3×0.35,
//                                        M4×0.5, M5×0.5, M6×0.75).  Depth =
//                                        tap_depth_mm (≥ 3 × pitch).
//   2. Lock-nut counterbore seat       — pilot.dia(lock_nut_M) +
//                                        across-flats clearance (≈ 0.4 mm).
//                                        Depth = nut_thk + 0.2 mm clearance.
//   3. Indicator scribe groove on rim  — annular V-groove around the seat
//                                        opening (width 0.3 mm, depth 0.15 mm).
//
//                          ┌─ entry_face ──────────────────────┐
//                          ▼                                   ▼
//                      ────┬──────────────┬──── ◄── scribe groove (rim)
//                          │   ▲ lock-nut │
//                          │     pocket   │
//                          │              │
//                          ├──────────────┤      ── seat → pilot step
//                          │  ▲ pilot     │
//                          │              │
//                          └──────────────┘
//
// Standard:
//   - Fine-pitch metric (ISO 261): "M4×0.5", "M5×0.5", "M6×0.75", "M8×1".
//   - Lock-nut across-flats (DIN 934): M3=5.5, M4=7, M5=8, M6=10, M8=13 mm.
//
// DFM gates:
//   - Fine-thread size must be in the supported table.
//   - Tap depth ≥ 3 × pitch (else thread strips).
//   - Lock-nut seat dia > fine-pitch pilot dia (else not a counterbore).
//   - Scribe groove width ≥ 0.2 mm (sub-CNC engraver limit).
//
// Signature pattern:
//   - is_compound = true
//   - subfeature_count = 3
//   - kind = "adjuster_screw_pocket_compound"
//   - All input + derived (pilot_dia_mm, lock_nut_af_mm, pitch_mm) recorded.
//
// Recognize:
//   Metadata replay — compound semantics cannot be safely reconstructed
//   from B-rep alone (pilot + counterbore + tiny groove looks like many
//   plain step-bores).  Returns confidence 1.0 when feature history matches.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace adjuster_screw_pocket_compound {

constexpr const char* kSkillId = "adjuster_screw_pocket_compound";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm  = 0.0;
    double      position_y_mm  = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    std::string fine_thread;                // "M4x0.5", "M5x0.5", "M6x0.75", "M8x1"
    std::string lock_nut_M;                 // "M3", "M4", "M5", "M6", "M8"
    double      tap_depth_mm   = 0.0;
    double      nut_thickness_mm = 0.0;     // DIN 934 thickness
    double      scribe_groove_width_mm = 0.3;
    double      scribe_groove_depth_mm = 0.15;
};

// Resolve helpers (public for testability).
double finePilotDiameterFor (const std::string& fine_thread);
double finePitchFor         (const std::string& fine_thread);
double lockNutAcrossFlatsFor(const std::string& lock_nut_M);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace adjuster_screw_pocket_compound
}  // namespace koocadcam::skill
