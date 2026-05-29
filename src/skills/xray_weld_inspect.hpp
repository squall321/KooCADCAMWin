#pragma once
// @lat: [[engine/skills#xray_weld_inspect]]
//
// xray_weld_inspect — radiographic (X-ray) NDT of a named weld, per AWS
// D1.x acceptance.  Metadata-only: no geometric change.
//
//   X-ray source ──▶  weld bead  ──▶  film / DDA panel
//                       │
//                       └─ defect map: cracks / porosity / slag / LoF / LoP
//
// AWS D1.1 §6 / D1.5 acceptance classes:
//   "D1"  cyclically-loaded structures (most stringent)
//   "D2"  statically-loaded structures
//   "D3"  tubular structures
//
// Pattern:
//   pattern.kind             = "xray_weld_inspect"
//   pattern.weld_id          = <input>
//   pattern.kv               = <input>
//   pattern.exposure_min     = <input>
//   pattern.defect_class     = "D1" | "D2" | "D3"
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT          weld_id empty
//   DFM-INPUT          kv outside (20, 600] keV
//   DFM-INPUT          exposure_min <= 0
//   DFM-NDT-CLASS      defect_class not in {D1,D2,D3}

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace xray_weld_inspect {

constexpr const char* kSkillId = "xray_weld_inspect";

struct Input
{
    std::string  weld_id;
    double       kv             = 200.0;
    double       exposure_min   = 3.0;
    std::string  defect_class   = "D1";   // "D1" | "D2" | "D3"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace xray_weld_inspect
}  // namespace koocadcam::skill
