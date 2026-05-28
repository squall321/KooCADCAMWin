#pragma once
// @lat: [[engine/skills#thin_film_deposit]]
//
// thin_film_deposit — additive semiconductor process that grows a thin
// (nm-scale) film on the wafer using CVD, PVD, or ALD chemistry.  At the
// CAD/CAM planning level the change is sub-micron and is treated as a
// METADATA-ONLY operation: the shape is preserved and the deposition
// recipe is recorded on the signature.
//
//        ┌──────────────┐                       ┌──────────────┐
//        │   substrate  │   thin_film_deposit   │ substrate +  │
//        │              │  ──────────────────▶  │  thin film   │
//        └──────────────┘                       └──────────────┘
//             (no OCCT shape delta — film thickness < 1 μm)
//
// Signature pattern:
//   - pattern.kind                  = "thin_film_deposit"
//   - pattern.process_type          = "CVD" | "PVD" | "ALD"
//   - pattern.thickness_nm          = <input>
//   - pattern.deposit_rate_nm_min   = <input>
//   - pattern.cycle_time_min        = thickness / rate
//   - pattern.is_additive           = true
//
// DFM checks:
//   process_type ∈ {CVD, PVD, ALD}
//   thickness_nm        > 0
//   deposit_rate_nm_min > 0
//   ALD warning if thickness > 100 nm (ALD is slow, info-level)
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace thin_film_deposit {

constexpr const char* kSkillId = "thin_film_deposit";

struct Input
{
    std::string  process_type        = "CVD";   // CVD | PVD | ALD
    double       thickness_nm        = 50.0;
    double       deposit_rate_nm_min = 10.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace thin_film_deposit
}  // namespace koocadcam::skill
