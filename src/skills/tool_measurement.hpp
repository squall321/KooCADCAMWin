#pragma once
// @lat: [[engine/skills#tool_measurement]]
//
// tool_measurement — pre-setting / measurement of a cutting tool on an
// optical or contact pre-setter (Zoller, Speroni, Parlec).  The measured
// diameter + length offsets are pushed to the CAM tool-library and to the
// machine's H/D table.  Recording the measurement step on the workpiece
// signature gives end-to-end traceability: which physical tool, on which
// pre-setter, with what offsets, made this part.
//
//       ┌────────────────────┐
//       │   pre-setter       │   meas_dia, meas_length → CAM library
//       │   (Zoller P-410)   │  ──────────────────────▶  + H/D table
//       │                    │
//       └────────────────────┘
//                ↑ tool
//
// Signature pattern:
//   - pattern.kind             = "tool_measurement"
//   - pattern.tool_id          = <input>
//   - pattern.meas_machine_id  = <input>
//   - pattern.dia_meas_mm      = <input>
//   - pattern.length_meas_mm   = <input>
//   - pattern.geometry_changed = false
//
// DFM checks:
//   tool_id non-empty                            (DFM-INPUT error if not)
//   meas_machine_id non-empty                    (DFM-INPUT error if not)
//   dia_meas_mm > 0                              (DFM-INPUT error if not)
//   length_meas_mm > 0                           (DFM-INPUT error if not)
//   dia_meas_mm in [0.1, 200]                    (DFM-TM-DIA info if outside)
//   length_meas_mm in [5, 500]                   (DFM-TM-LEN info if outside)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tool_measurement {

constexpr const char* kSkillId = "tool_measurement";

struct Input
{
    std::string tool_id          = "EM-6MM-4F-001";
    std::string meas_machine_id  = "ZOLLER-P410-01";
    double      dia_meas_mm      = 6.000;
    double      length_meas_mm   = 75.000;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tool_measurement
}  // namespace koocadcam::skill
