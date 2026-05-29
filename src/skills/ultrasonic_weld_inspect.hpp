#pragma once
// @lat: [[engine/skills#ultrasonic_weld_inspect]]
//
// ultrasonic_weld_inspect — pulse-echo / phased-array UT of a named weld,
// per AWS D1.x acceptance.  Metadata-only: no geometric change.
//
//   piezo probe ▼
//          ▼ ─ ─ ─ ─ ─ ─ ─ ─ ─ weld bead ─ ─ ─
//          ▼          defect ◢◣              back wall
//
// Pattern:
//   pattern.kind             = "ultrasonic_weld_inspect"
//   pattern.weld_id          = <input>
//   pattern.freq_mhz         = <input>
//   pattern.defect_class     = "D1" | "D2" | "D3"
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT          weld_id empty
//   DFM-INPUT          freq_mhz outside (0, 25]
//   DFM-NDT-CLASS      defect_class not in {D1,D2,D3}

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ultrasonic_weld_inspect {

constexpr const char* kSkillId = "ultrasonic_weld_inspect";

struct Input
{
    std::string  weld_id;
    double       freq_mhz       = 5.0;
    std::string  defect_class   = "D1";   // "D1" | "D2" | "D3"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ultrasonic_weld_inspect
}  // namespace koocadcam::skill
