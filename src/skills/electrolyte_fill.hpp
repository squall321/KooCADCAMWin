#pragma once
// @lat: [[engine/skills#electrolyte_fill]]
//
// electrolyte_fill — inject liquid electrolyte (LiPF6 / LiFSI in carbonate
// solvents) into the dry-assembled cell can or pouch.  Cells are placed in
// a vacuum chamber so trapped air evacuates before the electrolyte wets the
// porous separator + electrode stack.  Insufficient vacuum traps air pockets
// that block ion transport.  Excessive electrolyte raises cost + gas volume.
//
//          ┌─── cell can ───┐
//          │                │
//   nozzle │ ::::::::::::: │  ← electrolyte volume (mL)
//   ─►──── │ ::::::::::::: │
//          │ ::::::::::::: │
//          └────────────────┘
//
// Geometry is NOT modelled.  All wetting / impregnation lives in the
// FeatureSignature.
//
// Signature pattern:
//   - pattern.kind                  = "electrolyte_fill"
//   - pattern.volume_ml             = injected volume
//   - pattern.electrolyte_type      = "LiPF6_EC_DMC" | "LiFSI_EMC" | "LiTFSI_DOL_DME" | …
//   - pattern.vacuum_pressure_mbar  = chamber vacuum at the moment of dose
//   - pattern.fill_temperature_c    = bath / can temp (typically 20 – 30 °C)
//   - pattern.dwell_time_min        = post-fill rest time for wetting
//   - pattern.geometry_changed      = false
//
// DFM checks:
//   DFM-INPUT              volume_ml ≤ 0  (error)
//   DFM-FILL-VOL-SMALL     volume_ml < 0.5 mL (error — below pouch-cell typical)
//   DFM-FILL-VOL-LARGE     volume_ml > 200 mL (error — bigger than 100-Ah prismatic)
//   DFM-FILL-VAC-HIGH      vacuum_pressure_mbar > 200 (info — risk of air entrapment)
//   DFM-FILL-VAC-LOW       vacuum_pressure_mbar <= 0  (error — invalid)
//   DFM-FILL-ELECTROLYTE   electrolyte_type unrecognised (info)
//   DFM-FILL-TEMP          fill_temperature_c outside [10, 40] °C (info)
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

namespace electrolyte_fill {

constexpr const char* kSkillId = "electrolyte_fill";

struct Input
{
    double       volume_ml             = 4.0;              // pouch-cell mid-range
    std::string  electrolyte_type      = "LiPF6_EC_DMC";   // 1 M LiPF6 in EC/DMC
    double       vacuum_pressure_mbar  = 50.0;             // strong vacuum default
    double       fill_temperature_c    = 25.0;
    double       dwell_time_min        = 60.0;             // overnight rest is common
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace electrolyte_fill
}  // namespace koocadcam::skill
