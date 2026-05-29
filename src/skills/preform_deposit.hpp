#pragma once
// @lat: [[engine/skills#preform_deposit]]
//
// preform_deposit — MCVD (Modified Chemical Vapor Deposition) or OVD
// (Outside Vapor Deposition) preform deposition.  Produces a doped silica
// glass preform rod with a graded refractive-index profile (core + clad).
//
// MCVD: SiCl4 + GeCl4 + O2 → SiO2 / GeO2 deposit inside a fused-silica
// substrate tube; tube is collapsed into a solid preform.
// OVD: same chemistry, but deposit on outside of a rotating bait rod; bait
// is removed and tube is consolidated.
//
//        ┌──────────────────────────────────┐
//        │  cladding (SiO2, undoped)        │  ← outer ring
//        │   ┌──────────────────────────┐   │
//        │   │  core (SiO2 + GeO2 ~5%) │   │  ← higher n, doped
//        │   └──────────────────────────┘   │
//        └──────────────────────────────────┘
//          preform → fiber_draw downstream
//
// Signature pattern:
//   - pattern.kind                     = "preform_deposit"
//   - pattern.method                   = "MCVD" | "OVD" | "VAD"
//   - pattern.core_dia_mm              = <input>
//   - pattern.clad_dia_mm              = <input>
//   - pattern.dopant_concentration_pct = <input>
//   - pattern.dopant_species           = "GeO2" | "F" | "P2O5" | …
//   - pattern.geometry_changed         = false
//
// DFM checks:
//   DFM-INPUT             core_dia_mm or clad_dia_mm ≤ 0  (error)
//   DFM-PREFORM-GEOMETRY  core_dia_mm ≥ clad_dia_mm       (error)
//   DFM-PREFORM-DOPANT    dopant_concentration_pct ∉ [0.1, 30] (error)
//   DFM-PREFORM-METHOD    method ∉ {MCVD, OVD, VAD}       (info)
//   DFM-PREFORM-NA        clad/core ratio < 5             (info; multi-mode)
//
// Synthesis:
//   Pure preform-recording — geometry of the workpiece is UNCHANGED.  The
//   preform is conceptually a separate downstream stage; this skill stamps
//   the metadata so fiber_draw can find it.
//
// Recognize:
//   Impossible from B-rep alone (dopant profile is sub-OCCT).  Returns
//   signatures filtered from `workpiece.features()`, confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace preform_deposit {

constexpr const char* kSkillId = "preform_deposit";

struct Input
{
    std::string  method                   = "MCVD";        // "MCVD" | "OVD" | "VAD"
    double       core_dia_mm              = 8.0;           // doped core
    double       clad_dia_mm              = 125.0;         // outer cladding
    double       dopant_concentration_pct = 5.0;           // GeO2 mol %
    std::string  dopant_species           = "GeO2";        // GeO2 | F | P2O5
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace preform_deposit
}  // namespace koocadcam::skill
