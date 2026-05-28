#pragma once
// @lat: [[engine/skills#3d_print]]
//
// 3d_print — additive manufacturing.  Builds the workpiece layer by layer.
//
//                          process_type:
//                          ─────────────────────────────────────────────
//                          "fdm"          → fused deposition modeling
//                                           (thermoplastic filament)
//                          "sla"          → stereolithography (UV resin)
//                          "sls"          → selective laser sintering (powder)
//                          "metal_laser"  → metal powder-bed laser fusion
//
//        layer N+1  ────────────────────  ▲
//        layer N    ────────────────────  │ layer_height_mm
//        layer N−1  ────────────────────  ▼
//        ...                              built up to full part height
//
// Model: the starting workpiece is the FINAL printed part.  3d_print does
// not modify geometry — it records the process metadata that lets a
// downstream slicer / executor emit G-code, infill, support, and
// layer-by-layer toolpaths.
//
// Note: the C++ namespace is `p3d_print` (file/skill-id is "3d_print" but
// C++ identifiers cannot start with a digit).  The skill_id string used
// across signatures is "3d_print" verbatim.
//
// DFM:
//   DFM-PRINT-LAYER  : layer_height_mm ∈ [0.05, 0.5] mm
//   DFM-PRINT-INFILL : (FDM only) infill_pct ∈ [10, 100]
//   DFM-PRINT-COMPAT : process_type vs material compatibility (info-only)
//
// Signature pattern:
//   pattern.kind == "3d_print"
//   pattern.process_type, pattern.layer_height_mm, pattern.material,
//   pattern.infill_pct  (FDM only — 0 for non-FDM)
//   pattern.geometric_change == false
//
// Recognize: metadata-only.  Layer-step "stair" artifacts COULD be detected
// geometrically on a high-res mesh, but on the analytic B-Rep workpiece the
// part is treated as the ideal final shape (slicer artifacts not modeled).

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace p3d_print {

constexpr const char* kSkillId = "3d_print";

struct Input
{
    std::string   process_type    = "fdm";    // "fdm" | "sla" | "sls" | "metal_laser"
    double        layer_height_mm = 0.2;
    std::string   material        = "pla";
    double        infill_pct      = 20.0;     // FDM only; 0 for non-FDM
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace p3d_print
}  // namespace koocadcam::skill
