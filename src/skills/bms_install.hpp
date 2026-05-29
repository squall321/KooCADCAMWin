#pragma once
// @lat: [[engine/skills#bms_install]]
//
// bms_install — record the installation of a Battery Management System (BMS)
// onto a pack.  The BMS is an external PCB / harness that monitors per-cell
// voltage and temperature; nothing about the pack B-rep changes.  The skill
// captures bms_id, channel count, and the upstream communication protocol so
// downstream firmware / commissioning steps can be planned.
//
//          ┌───────── BMS PCB ──────────┐
//          │ ch0 ch1 ch2 ... chN        │       ← channel_count
//          │     CAN / LIN bus          │       ← communication_protocol
//          └────────────┬───────────────┘
//                       │ harness
//                  ─────┴─────  pack interface
//
// Signature pattern:
//   - pattern.kind                   = "bms_install"
//   - pattern.bms_id                 = <input>
//   - pattern.channel_count          = <input>
//   - pattern.communication_protocol = "CAN" | "LIN"
//   - pattern.geometry_changed       = false
//
// DFM checks:
//   bms_id non-empty                            (DFM-INPUT error)
//   channel_count ∈ [4, 192]                    (DFM-BMS-CH error if out)
//   communication_protocol ∈ {"CAN", "LIN"}     (DFM-BMS-COMM error)
//
// Synthesis: NO geometric change. Clone + stamp.
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bms_install {

constexpr const char* kSkillId = "bms_install";

struct Input
{
    std::string  bms_id                 = "BMS-EV-001";
    int          channel_count          = 96;     // # cell-monitor channels
    std::string  communication_protocol = "CAN";  // "CAN" | "LIN"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bms_install
}  // namespace koocadcam::skill
