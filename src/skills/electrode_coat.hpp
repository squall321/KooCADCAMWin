#pragma once
// @lat: [[engine/skills#electrode_coat]]
//
// electrode_coat — slurry-coat anode or cathode active material onto a
// metallic current-collector foil (typ. Al for cathode, Cu for anode).  A
// slot-die or comma-bar applicator deposits a wet slurry of active material
// + binder + solvent; the foil then passes through a drying tunnel.  The
// final coating is a porous composite layer 30 – 250 μm thick on each side.
//
//        ┌──────────────────────────────────┐
//        │ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │  ← active material layer
//        │                                  │     (NMC811, LFP, graphite, …)
//        │  (collector foil — Al or Cu,     │
//        │   8 – 20 μm thick)               │
//        │                                  │
//        │ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │  ← opposite-side coating
//        └──────────────────────────────────┘
//
// Geometry is NOT modelled — the coating is a sub-OCCT thin film.  All
// process parameters live in the FeatureSignature for downstream CAPP /
// QA / cell-balancing tools.
//
// Signature pattern:
//   - pattern.kind                       = "electrode_coat"
//   - pattern.active_material            = "NMC811" | "LFP" | "graphite" | "LTO" | …
//   - pattern.electrode_side             = "anode" | "cathode"
//   - pattern.thickness_um               = wet+dried coat thickness per side
//   - pattern.areal_capacity_mAh_cm2     = areal capacity (key design knob)
//   - pattern.collector_foil_material    = "aluminum" | "copper"
//   - pattern.coating_method             = "slot_die" | "comma_bar" | "doctor_blade"
//   - pattern.geometry_changed           = false
//
// DFM checks:
//   DFM-INPUT              thickness ≤ 0 or areal_capacity ≤ 0 (error)
//   DFM-COAT-THK           thickness < 30 μm or > 250 μm (error — outside
//                          industrial slot-die window)
//   DFM-COAT-LOAD          areal_capacity < 0.5 or > 6.0 mAh/cm² (info —
//                          outside common cell-design window)
//   DFM-COAT-MATERIAL      active_material unrecognised (info)
//   DFM-COAT-SIDE          electrode_side neither anode nor cathode (error)
//   DFM-COAT-FOIL-MISMATCH cathode with Cu foil / anode with Al foil (info —
//                          unusual chemistry; LTO anode legitimately uses Al)
//
// Synthesis:
//   NO geometric change.  Clones the workpiece and stamps the signature.
//
// Recognize:
//   Impossible from B-rep (porous composite film is sub-OCCT).  Metadata-only
//   replay from `workpiece.features()`, confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace electrode_coat {

constexpr const char* kSkillId = "electrode_coat";

struct Input
{
    std::string  active_material         = "NMC811";        // cathode default
    std::string  electrode_side          = "cathode";       // "anode" | "cathode"
    double       thickness_um            = 80.0;            // dried coat, per side
    double       areal_capacity_mAh_cm2  = 3.5;             // typical EV cathode
    std::string  collector_foil_material = "aluminum";      // "aluminum" | "copper"
    std::string  coating_method          = "slot_die";      // "slot_die"|"comma_bar"|"doctor_blade"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace electrode_coat
}  // namespace koocadcam::skill
