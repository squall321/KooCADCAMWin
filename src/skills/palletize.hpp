#pragma once
// @lat: [[engine/skills#palletize]]
//
// palletize — stack finished parts onto a pallet for shipping or for
// intra-plant transfer.  Pure material-handling step; the single
// workpiece's geometry is unchanged.  The signature records the
// rows × cols × layers stacking pattern and the pallet footprint so a
// downstream cell-controller can drive a robot to deposit each part at
// its assigned slot.
//
//                  ┌─────────────────┐
//      layer 2 →   │ ◼ ◼ ◼ ◼ ◼       │     rows × cols at each layer
//      layer 1 →   │ ◼ ◼ ◼ ◼ ◼       │
//                  │ ◼ ◼ ◼ ◼ ◼       │  ← layer N
//                  └────┬───────┬────┘
//                       │       │
//                ╔══════╧═══════╧══════╗
//                ║       PALLET        ║   pallet_size = [L, W] mm
//                ╚═════════════════════╝
//
// Signature pattern:
//   - pattern.kind             = "palletize"
//   - pattern.pallet_pattern   = { "rows": R, "cols": C, "layers": L }
//   - pattern.parts_per_pallet = R * C * L
//   - pattern.pallet_size      = [length_mm, width_mm]
//   - pattern.geometry_changed = false
//
// DFM checks:
//   rows ≥ 1, cols ≥ 1, layers ≥ 1               (DFM-INPUT error)
//   pallet_length_mm > 0, pallet_width_mm > 0     (DFM-INPUT error)
//   parts_per_pallet ≤ 5000                       (DFM-PALLET-COUNT info)
//   pallet area ≤ standard ISO (1200×1000)        (DFM-PALLET-SIZE info)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
//
// Recognize:
//   Metadata-only — no geometric fingerprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace palletize {

constexpr const char* kSkillId = "palletize";

struct Input
{
    int     rows            = 4;
    int     cols            = 4;
    int     layers          = 3;
    double  pallet_length_mm = 1200.0;       // ISO standard
    double  pallet_width_mm  = 1000.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace palletize
}  // namespace koocadcam::skill
