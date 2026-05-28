#pragma once
// @lat: [[engine/skills#wave_solder]]
//
// wave_solder — Through-hole soldering by passing a populated PCB across a
// standing wave of molten solder.  Leads protruding from the underside touch
// the wave and form fillets in one pass.  The macroscopic PCB B-rep is
// UNCHANGED — solder fillet volumes are sub-millimetre and not modelled in
// OCCT here.  What is recorded is the wave-solder run: alloy chemistry,
// pot temperature, and conveyor speed (which together govern dwell time).
//
//          ┌─────────────── PCB (conveyor) ───────────────┐
//          │  ▌▌▌▌▌▌▌▌▌▌  ◯  ◯  ◯  ◯  ◯  ◯  ◯  ▌▌▌▌▌▌▌▌▌▌  │  →  conveyor_speed
//          └──────────────────┃▓▓▓▓▓▓▓▓▓▓▓┃───────────────┘
//                             ┃           ┃
//                             ┃  WAVE     ┃   ← molten solder pot
//                             ┃ (260 °C)  ┃      temp_c
//                             └───────────┘
//
// Signature pattern:
//   - pattern.kind             = "wave_solder"
//   - pattern.solder_alloy     = "SAC305" | "Sn63Pb37" | "SN100C" | …
//   - pattern.temp_c           = <input>   pot temperature (°C)
//   - pattern.conveyor_speed   = <input>   m/min
//   - pattern.dwell_time_s     = derived = effective wave width / conveyor_speed
//   - pattern.geometry_changed = false
//
// DFM process gates (error vs info):
//   DFM-INPUT              temp_c ≤ 0                (error)
//   DFM-INPUT              conveyor_speed ≤ 0        (error)
//   DFM-INPUT              solder_alloy empty        (error)
//   DFM-WAVE-TEMP-LOW      temp_c < 230 °C           (error — Sn-Pb solidus already)
//   DFM-WAVE-TEMP-HIGH     temp_c > 290 °C           (error — board delamination risk)
//   DFM-WAVE-SPEED-LOW     conveyor_speed < 0.4 m/min (info — long dwell, bridging risk)
//   DFM-WAVE-SPEED-HIGH    conveyor_speed > 2.0 m/min (info — short dwell, cold-joint risk)
//   DFM-WAVE-ALLOY         solder_alloy not in known catalog (info)
//
// Recognize:
//   Impossible from B-rep (fillets sub-OCCT).  Returns ONLY signatures
//   replayed from `workpiece.features()` filtered by skill_id — confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace wave_solder {

constexpr const char* kSkillId = "wave_solder";

struct Input
{
    std::string  solder_alloy   = "SAC305";    // "SAC305" | "Sn63Pb37" | "SN100C"
    double       temp_c         = 260.0;       // wave pot temperature
    double       conveyor_speed = 1.0;         // m/min
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace wave_solder
}  // namespace koocadcam::skill
