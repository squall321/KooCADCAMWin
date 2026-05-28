#pragma once
// @lat: [[engine/skills#spring_form]]
//
// spring_form — coil a length of wire into a helical compression spring on
// a CNC coiling machine.  The wire is fed against a pin that bends it into
// a helix; the pitch and diameter are programmed.
//
//                  ╲   ╲   ╲   ╲   ╲      ← coils with pitch p
//                 ──────────────────────  free length L
//                   wire dia d
//                                                   coil_pitch
//   geometric quantities:
//     n_turns                  number of active coils
//     coil_pitch (mm/turn)     axial advance per turn
//     wire_dia (mm)            wire diameter
//     free_length (mm)         unloaded length (≈ n_turns × coil_pitch + d)
//
// Slice-1 representation:
//   The detailed coil geometry (a true helix sweep) is non-trivial to
//   construct robustly and would inflate the workpiece face count
//   dramatically.  Slice-1 keeps the workpiece geometry UNCHANGED and
//   stores all dimensional intent in the signature.  Downstream layers
//   (rendering, FEA) can rebuild the helix from the metadata.
//
// Signature pattern:
//   - kind            = spring_form
//   - coil_pitch_mm   = axial pitch per turn
//   - n_turns         = total active coils
//   - wire_dia_mm     = wire diameter
//   - free_length_mm  = expected free length
//   - mean_dia_mm     = mean coil diameter (informational; derived from input)
//   - spring_index    = mean_dia_mm / wire_dia_mm  (sanity: 4 ≤ C ≤ 12)
//
// DFM checks:
//   DFM-INPUT          : wire_dia_mm > 0, n_turns > 0, coil_pitch_mm > 0
//   DFM-SPR-INDEX      : spring_index outside [4, 12] — warning
//                        (C < 4: stress concentration; C > 12: buckling risk)
//   DFM-SPR-SOLID      : coil_pitch_mm ≤ wire_dia_mm — error
//                        (coils would touch in unloaded state — invalid spring)
//   DFM-SPR-LEN        : free_length / mean_dia > 4 — warning
//                        (high slenderness ratio — buckling risk)
//
// Recognize:
//   Metadata-only.  A coiled spring sitting in B-rep storage cannot be
//   distinguished from any other helical workpiece without dedicated
//   helix-extraction code; we replay from features().

#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace spring_form {

constexpr const char* kSkillId = "spring_form";

struct Input
{
    double  coil_pitch_mm   = 0.0;   // axial advance per turn
    double  n_turns         = 0.0;   // number of active coils (may be fractional)
    double  wire_dia_mm     = 0.0;   // wire diameter
    double  free_length_mm  = 0.0;   // expected unloaded length
    double  mean_dia_mm     = 0.0;   // mean coil diameter (centre-to-centre)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace spring_form
}  // namespace koocadcam::skill
