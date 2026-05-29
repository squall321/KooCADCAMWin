#pragma once
// @lat: [[engine/skills#drop_forge]]
//
// drop_forge — drop-hammer (impact-die) forging.  A heavy hammer is dropped
// (gravity or steam-augmented) onto stock seated in a die, with shape
// produced by repeated blows rather than continuous press load.  Compared
// to closed-die forging on a press, drop forging:
//   - imparts energy per blow (kJ) rather than tonnage
//   - requires multiple blows for full die fill
//   - is preferred for thin or web-bearing geometries
//
//                  ┌──── hammer ────┐
//                  │   E = m·g·h    │  blow_count × hammer_energy_kj
//                  ▼                ▼
//        ┌─────────────────────────────┐
//        │           stock             │  ──►   shaped part (per die)
//        └─────────────────────────────┘
//                lower die anvil
//
// Synthesis strategy:
//   Metadata-only — the input workpiece is taken as the post-forge shape.
//   We clone and stamp.  Geometric edits belong in `closed_die_forge`
//   (single-press) or upstream CAD die-cavity work.
//
// Signature pattern:
//   - kind                = "drop_forge"
//   - hammer_energy_kj
//   - blow_count
//   - total_energy_kj     (energy × blows)
//   - geometry_changed    = false
//
// DFM checks:
//   DFM-INPUT       : hammer_energy_kj > 0; blow_count >= 1
//   DFM-DF-ENERGY   : hammer_energy_kj > 200 kJ — warning (very heavy hammer)
//   DFM-DF-BLOWS    : blow_count > 20 — info (rework / die wear concern)
//
// Recognize:
//   Pure metadata replay — geometry alone cannot identify impact forging.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace drop_forge {

constexpr const char* kSkillId = "drop_forge";

struct Input
{
    double  hammer_energy_kj = 20.0;   // energy per blow
    int     blow_count       = 3;      // number of blows per part
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace drop_forge
}  // namespace koocadcam::skill
