#pragma once
// @lat: [[engine/skills#waveguide_pattern]]
//
// waveguide_pattern — photolithography + dry-etch step that patterns a
// waveguide trace on a photonic integrated circuit (PIC).  Mask defines
// the trace topology; etch_depth_nm + line_width_nm define the cross-
// section that determines the guided-mode profile.
//
//        ┌────────────────────────────────────────┐
//        │  cladding (SiO2)                       │
//        │     ┌──┐         ┌──┐    ┌────────────┐│
//        │     │  │═════════│  │════│            ││ ← Si core ribs
//        │     └──┘         └──┘    └────────────┘│   etch_depth_nm down,
//        │  substrate (Si)                        │   line_width_nm wide
//        └────────────────────────────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "waveguide_pattern"
//   - pattern.mask_id          = <input>
//   - pattern.etch_depth_nm    = <input>
//   - pattern.line_width_nm    = <input>
//   - pattern.process          = "RIE" | "DRIE" | "ICP"
//   - pattern.aspect_ratio     = etch_depth_nm / line_width_nm
//   - pattern.geometry_changed = false  (sub-OCCT scale)
//
// DFM checks:
//   DFM-INPUT             mask_id empty                             (error)
//   DFM-WG-DEPTH          etch_depth_nm ∉ [50, 5000]                (error)
//   DFM-WG-WIDTH          line_width_nm ∉ [100, 10000]              (error)
//   DFM-WG-ASPECT         aspect > 10 (DRIE limit)                  (info)
//
// Synthesis:
//   Records waveguide pattern metadata.  Feature scale (≈ 100 nm) is below
//   any reasonable OCCT tolerance, so this skill is metadata-only.  PICs
//   are modeled at the layout level, not the B-rep level.
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

namespace waveguide_pattern {

constexpr const char* kSkillId = "waveguide_pattern";

struct Input
{
    std::string  mask_id        = "WG_BASIC_v1";
    double       etch_depth_nm  = 220.0;        // standard SOI rib height
    double       line_width_nm  = 500.0;        // 0.5 µm rib width
    std::string  process        = "ICP";        // "RIE" | "DRIE" | "ICP"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace waveguide_pattern
}  // namespace koocadcam::skill
