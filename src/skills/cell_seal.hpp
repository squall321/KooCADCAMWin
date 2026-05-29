#pragma once
// @lat: [[engine/skills#cell_seal]]
//
// cell_seal — final hermetic closure of a Li-ion cell.  Three industrial
// methods cover virtually all formats:
//
//   - "weld"   laser or ultrasonic seam closes the cap of cylindrical /
//              prismatic metal cans (e.g. 18650, 21700, 4680).
//   - "crimp"  mechanical fold-over closure used on legacy cylindrical
//              cells with a gasket-sealed cap.
//   - "heat"   thermal heat-seal of an aluminium-laminate pouch cell along
//              its three open edges.
//
// Leakage is reported in pascal-cubic-metres per second (Pa·m³/s) by a
// helium fine-leak tester.  A "good" Li-ion seal typically registers
// ≤ 1 × 10⁻⁹ Pa·m³/s.
//
//        ┌─────── weld seam ───────┐
//   ╔════╪═════════════════════════╪════╗   ← cap welded to can
//   ║    └─────── cell can ────────┘    ║
//   ╚═══════════════════════════════════╝
//
// Signature pattern:
//   - pattern.kind          = "cell_seal"
//   - pattern.seal_method   = "weld" | "crimp" | "heat"
//   - pattern.leak_pa_m3_s  = post-seal helium leak rate
//   - pattern.weld_energy_j = energy per weld (for "weld"; 0 otherwise)
//   - pattern.seal_temp_c   = heat-seal temperature (for "heat"; 0 otherwise)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT          leak_pa_m3_s < 0 (error)
//   DFM-SEAL-METHOD    seal_method not in {weld, crimp, heat} (error)
//   DFM-SEAL-LEAK      leak_pa_m3_s > 1e-8 (error — fails hermeticity spec)
//   DFM-SEAL-LEAK-WARN leak_pa_m3_s > 1e-9 (info — marginal, monitor)
//   DFM-SEAL-WELD-ENERGY  weld method with energy < 1 J (info — likely under-fused)
//   DFM-SEAL-HEAT-TEMP    heat-seal temperature outside [120, 200] °C (info)
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

namespace cell_seal {

constexpr const char* kSkillId = "cell_seal";

struct Input
{
    std::string  seal_method     = "weld";      // "weld" | "crimp" | "heat"
    double       leak_pa_m3_s    = 1.0e-10;     // good hermetic seal
    double       weld_energy_j   = 25.0;        // laser weld pulse energy
    double       seal_temp_c     = 165.0;       // heat-seal nominal
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cell_seal
}  // namespace koocadcam::skill
