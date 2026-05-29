#pragma once
// @lat: [[engine/skills#tube_to_tubesheet_weld]]
//
// tube_to_tubesheet_weld — TIG roll-and-weld of a tube end into a tubesheet
// hole, repeated for every tube in a heat-exchanger bundle.  The operation
// fuses each tube's outside diameter to the tubesheet bore wall around the
// full circumference; the tube end is flush with (or slightly proud of) the
// tubesheet face.  This is the canonical joint of a shell-and-tube heat
// exchanger — there can be hundreds to thousands of these joints per vessel.
//
//                          ┌── tubesheet ──┐
//                          │   ░░░░░░░░░   │   (TIG fillet around bore)
//                          │ ╔══════════╗  │
//      tube interior  ─────╫──────────────╫───── tube interior
//                          │ ╚══════════╝  │
//                          │   ░░░░░░░░░   │   (TIG fillet around bore)
//                          └───────────────┘
//
// Signature pattern:
//   - pattern.kind             = "tube_to_tubesheet_weld"
//   - pattern.tube_dia_mm
//   - pattern.sheet_thick_mm
//   - pattern.joint_count          (number of tubes welded to this sheet)
//   - pattern.weld_process         = "tig_roll"
//   - pattern.fillet_leg_mm        (≈ tube wall thickness)
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   DFM-INPUT           tube_dia_mm ≤ 0, sheet_thick_mm ≤ 0, joint_count ≤ 0
//   DFM-HE-TUBE-DIA     tube_dia_mm < 6 mm or > 50 mm (typical shell-and-tube
//                       range; outside is an info-level hint only)
//   DFM-HE-SHEET-THK    sheet_thick_mm < tube_dia_mm × 0.2 → joint may not
//                       develop full strength (info)
//   DFM-HE-COUNT        joint_count > 10 000 → batch / fixture concerns (info)
//
// Synthesis:
//   Geometric no-op (the tube/tubesheet B-rep is not modelled at this slice).
//   Stamps the signature so downstream CAPP / inspection knows how many
//   tube-to-tubesheet welds are in the bundle and what NDT regime applies.
//
// Recognize:
//   Pure metadata replay from `workpiece.features()`.  Geometry alone cannot
//   distinguish a roll-and-weld joint from a press-expanded joint — the
//   process-plan annotation is what disambiguates.

#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tube_to_tubesheet_weld {

constexpr const char* kSkillId = "tube_to_tubesheet_weld";

struct Input
{
    double  tube_dia_mm    = 19.05;   // 3/4" tubes (most common shell-and-tube)
    double  sheet_thick_mm = 25.0;    // tubesheet thickness
    int     joint_count    = 1;       // tubes welded to this sheet
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tube_to_tubesheet_weld
}  // namespace koocadcam::skill
