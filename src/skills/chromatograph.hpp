#pragma once
// @lat: [[engine/skills#chromatograph]]
//
// chromatograph — record an analytical chromatography run.  A sample is
// injected onto a column and resolved into component peaks using the chosen
// method (HPLC, GC, LC-MS).  The workpiece (the sample vial or autosampler
// cartridge) is geometrically unchanged; the signature records method, column
// identifier and runtime.
//
//        ┌─────────────┐                       ┌─────────────┐
//        │  injected   │   chromatograph       │  peaks      │
//        │   sample    │ ─────────────────────▶│  (resolved) │
//        │             │  (method, column,     │             │
//        │             │   runtime_min)        │             │
//        └─────────────┘                       └─────────────┘
//          shape identical ; chromatogram metadata stamped
//
// Signature pattern:
//   pattern.kind             = "chromatograph"
//   pattern.method           = "HPLC" | "GC" | "LC-MS"
//   pattern.column_id        = string
//   pattern.runtime_min      = double
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT         method unknown / empty (error)
//   DFM-INPUT         column_id empty (error)
//   DFM-CHRO-TIME     runtime_min <= 0 (error)
//   DFM-CHRO-TIME     runtime_min > 240 (info — exceptionally long run)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace chromatograph {

constexpr const char* kSkillId = "chromatograph";

struct Input
{
    std::string  method       = "HPLC";     // "HPLC" | "GC" | "LC-MS"
    std::string  column_id    = "C18-150x4.6";
    double       runtime_min  = 15.0;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace chromatograph
}  // namespace koocadcam::skill
