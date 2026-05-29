#pragma once
// @lat: [[engine/skills#anchor_bolt_install]]
//
// anchor_bolt_install — installation record for a cast-in-place anchor bolt
// that fastens a steel base plate to a concrete foundation.  The bolt is
// embedded into wet concrete, cured, and torqued through the base plate
// hole.  The macroscopic workpiece is unchanged; this skill captures the
// installed-fastener metadata for inspection and torque verification.
//
//             ───── base plate ─────
//             │  ░░░░░  │           ← torque_nm on nut
//          ───┴─────────┴───
//             │  bolt  │
//          ░░░│        │░░░         ← concrete
//          ░░░│        │░░░             embedment_mm
//          ░░░└────────┘░░░         ← anchor head / bent foot
//          ░░░░░░░░░░░░░░░░
//
// Signature pattern:
//   - pattern.kind             = "anchor_bolt_install"
//   - pattern.bolt_dia_mm
//   - pattern.embedment_mm
//   - pattern.torque_nm
//   - pattern.embed_ratio (= embedment_mm / bolt_dia_mm)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   bolt_dia_mm    ∈ [12, 64]            (DFM-AB-DIA   error if outside)
//   embedment_mm   ≥ 8 × bolt_dia_mm     (DFM-AB-EMBED error if shallow)
//   embedment_mm   ≤ 30 × bolt_dia_mm    (DFM-AB-EMBED info  if very deep)
//   torque_nm      > 0                   (DFM-INPUT    error if non-positive)

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace anchor_bolt_install {

constexpr const char* kSkillId = "anchor_bolt_install";

struct Input
{
    double bolt_dia_mm  = 24.0;
    double embedment_mm = 240.0;     // 10 × dia default
    double torque_nm    = 300.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace anchor_bolt_install
}  // namespace koocadcam::skill
