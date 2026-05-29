#pragma once
// @lat: [[engine/skills#friction_stir_weld_aero]]
//
// friction_stir_weld_aero — long solid-state seam joining two aerospace
// extrusions (typically Al-Li wing-skin to fuselage stringer, or 2xxx/7xxx
// pairings).  A rotating shoulder + pin tool plunges into the joint line
// and traverses, plasticising and forging the material WITHOUT melting.
// Because the surfaces remain quasi-flush, this is treated as a metadata-
// only skill: the visible seam line is a sub-OCCT texture rather than a
// raised bead.
//
//        traverse direction →
//          ┌──────────┬──────────┐
//          │ alloy A  │ alloy B  │     ← two aerospace extrusions
//          │ ─────────┼───────── │     ← seam = tool path (FSW signature)
//          │          │          │
//          └──────────┴──────────┘
//                 ▲
//             tool shoulder (Ø tool_dia_mm) + pin
//
// Signature pattern:
//   - pattern.kind                 = "friction_stir_weld_aero"
//   - pattern.tool_dia_mm
//   - pattern.traverse_speed_mm_min
//   - pattern.alloy_pair           = e.g. "AA2024-AA7075"
//   - pattern.seam_length_mm
//   - geometry_changed             = false
//
// DFM checks:
//   tool_dia_mm           ∈ [8, 25]   (DFM-FSW-TOOL error if outside)
//   traverse_speed_mm_min ∈ [50, 1500] (DFM-FSW-SPEED error if outside)
//   seam_length_mm        ≥ 50.0      (DFM-FSW-LEN error — short seams use TIG/laser)
//   alloy_pair            non-empty   (DFM-INPUT warning)
//
// Synthesis:
//   NO geometric change.  Clone the workpiece + stamp signature.
//
// Recognize:
//   Metadata replay only (FSW seam is a sub-OCCT micro-texture).

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace friction_stir_weld_aero {

constexpr const char* kSkillId = "friction_stir_weld_aero";

struct Input
{
    FaceDatum   entry_face;                              // face containing the seam line
    double      tool_dia_mm           = 16.0;            // shoulder Ø (typ. 10 – 20 mm)
    double      traverse_speed_mm_min = 400.0;           // typ. 100 – 800 mm/min
    double      seam_length_mm        = 300.0;           // length of the seam
    std::string alloy_pair            = "AA2024-AA7075"; // e.g. "AA2024-AA7075"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace friction_stir_weld_aero
}  // namespace koocadcam::skill
