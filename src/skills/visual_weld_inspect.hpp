#pragma once
// @lat: [[engine/skills#visual_weld_inspect]]
//
// visual_weld_inspect — visual inspection (VT) of a weld, per AWS D1.x
// acceptance.  Records weld size measurement + accept/reject.
// Metadata-only.
//
// Per AWS D1.1 §6.9: a fillet weld leg size shall be within +∞ / -1.5 mm
// of the specified size; we encode the absolute weld_size_mm and the
// pass/fail verdict.
//
// Pattern:
//   pattern.kind             = "visual_weld_inspect"
//   pattern.weld_id          = <input>
//   pattern.weld_size_mm     = <input>
//   pattern.accept           = <input bool>
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT          weld_id empty
//   DFM-INPUT          weld_size_mm <= 0
//   DFM-VT-OVERSIZE    weld_size_mm > 40 (info — unusually large fillet)

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace visual_weld_inspect {

constexpr const char* kSkillId = "visual_weld_inspect";

struct Input
{
    std::string  weld_id;
    double       weld_size_mm = 6.0;
    bool         accept       = true;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace visual_weld_inspect
}  // namespace koocadcam::skill
