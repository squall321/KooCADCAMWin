#pragma once
// @lat: [[engine/skills#gas_purify]]
//
// gas_purify — record a process step that purifies a process gas before
// it enters a UHV / cryogenic system.  Common upstream of ALD, MBE,
// cryostat backfills, and superconducting qubit dilution refrigerators.
// Common methods: getter bed (SAES), cold trap (LN2), Pd-membrane
// (hydrogen), molecular sieve.  The macroscopic workpiece is unchanged;
// this is a metadata record of supply-gas quality used downstream.
//
//     process gas ──▶ [ purifier ] ──▶ UHV / cryo system
//                       ▲
//                       │ method = getter | cold_trap | Pd_membrane | mol_sieve
//                       │ output purity → final_purity_ppb
//
// Signature pattern:
//   - pattern.kind                = "gas_purify"
//   - pattern.gas_species         = <input>
//   - pattern.purification_method = <input>
//   - pattern.final_purity_ppb    = <input>
//   - geometry_changed            = false
//
// DFM checks:
//   gas_species          non-empty                  (DFM-INPUT)
//   purification_method  in known set               (DFM-GAS-METHOD info)
//   final_purity_ppb     > 0                        (DFM-INPUT)
//   final_purity_ppb     > 1000 ⇒ DFM-GAS-PURITY warning (not UHV-grade)
//
// Synthesis: clone + stamp.  Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gas_purify {

constexpr const char* kSkillId = "gas_purify";

struct Input
{
    std::string gas_species         = "argon";
    std::string purification_method = "getter";   // "getter" | "cold_trap" | "Pd_membrane" | "mol_sieve"
    double      final_purity_ppb    = 10.0;       // impurity, ppb (parts per billion)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gas_purify
}  // namespace koocadcam::skill
