#pragma once
// @lat: [[engine/skills#panel_attach]]
//
// panel_attach — installation record for an exterior or interior cladding /
// sheathing panel fastened to a steel-framing sub-structure.  Examples:
// metal siding, gypsum sheathing, sandwich insulation panel, glass curtain
// wall.  The panel is mechanically fastened (screws / rivets / clips) around
// its perimeter and joints are sealed.  The workpiece B-rep is unchanged;
// panel + fasteners + sealant are sub-OCCT cladding metadata.
//
//        ───────────────────────────────
//        │                             │  ← panel_type (metal_siding /
//        │ ▓ ▓ ▓ ▓ ▓ ▓ ▓ ▓ ▓ ▓ ▓ ▓ ▓ ▓ │     gypsum_board / sandwich / glass)
//        │                             │
//        │ ▓ ▓ ▓ ▓ ▓ ▓ ▓ ▓ ▓ ▓ ▓ ▓ ▓ ▓ │  ← fastener_count screws / clips
//        │                             │
//        ───────────────────────────────
//          〜〜 sealant_type bead 〜〜
//
// Signature pattern:
//   - pattern.kind             = "panel_attach"
//   - pattern.panel_type
//   - pattern.fastener_count
//   - pattern.sealant_type
//   - pattern.geometry_changed = false
//
// DFM checks:
//   fastener_count  ∈ [4, 200]           (DFM-PA-COUNT error if outside)
//   panel_type      non-empty            (DFM-INPUT    error if empty)
//   sealant_type    non-empty            (DFM-INPUT    warning if empty)
//   panel_type      ∈ {metal_siding, gypsum_board, sandwich, glass}
//                                        (DFM-PA-TYPE info if unknown)

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace panel_attach {

constexpr const char* kSkillId = "panel_attach";

struct Input
{
    std::string panel_type     = "metal_siding";
    int         fastener_count = 24;
    std::string sealant_type   = "silicone";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace panel_attach
}  // namespace koocadcam::skill
