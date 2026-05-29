#pragma once
// @lat: [[engine/skills#vsat_assemble]]
//
// vsat_assemble — Very-Small-Aperture-Terminal satellite dish assembly:
// records the dish diameter, the assigned satellite transponder, and the
// feed polarization (linear / circular).  The workpiece B-rep is unchanged
// — only the assembled-terminal record is stamped.
//
//                     ╭──── ─ ─ ─ ─
//                    ╱ ╲           ╲   ← parabolic reflector (dish_dia_m)
//                   ╱   ╲           ╲
//                   ╲   ╱  ●─── feed (transponder, polarization)
//                    ╲ ╱           ╱
//                     ╰──── ─ ─ ─ ─
//                      │
//                      │ pedestal / mount
//                     ─┴─
//
// Signature pattern:
//   - pattern.kind             = "vsat_assemble"
//   - pattern.dish_dia_m
//   - pattern.transponder      = string ("Ku-A1" | "Ka-B3" | "C-7" | ...)
//   - pattern.polarization     = "linear_h" | "linear_v" | "rhcp" | "lhcp"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   dish_dia_m > 0                                 (DFM-INPUT error)
//   dish_dia_m ∉ [0.6, 3.7]                        (DFM-VSAT-DIA warning — typical VSAT range)
//   transponder non-empty                          (DFM-INPUT error)
//   polarization ∉ {linear_h, linear_v, rhcp,lhcp} (DFM-VSAT-POL info)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
//
// Recognize:
//   Metadata-only — no geometric fingerprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace vsat_assemble {

constexpr const char* kSkillId = "vsat_assemble";

struct Input
{
    double       dish_dia_m    = 1.2;
    std::string  transponder   = "Ku-A1";
    std::string  polarization  = "linear_v";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace vsat_assemble
}  // namespace koocadcam::skill
