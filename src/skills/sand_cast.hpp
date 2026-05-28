#pragma once
// @lat: [[engine/skills#sand_cast]]
//
// sand_cast — sand casting process: pour molten metal into a sand mold to
// produce a part with the shape of the mold cavity.
//
//      ┌─────────────────┐         ┌─────────────────┐
//      │                 │   pour  │░░░░░░░░░░░░░░░░░│
//      │   sand mold     │   ──►   │░░░░░░░░░░░░░░░░░│  ← molten metal
//      │   cavity        │         │░░░  cast part ░░│    fills cavity
//      │   (inverse of   │         │░░░░░░░░░░░░░░░░░│
//      │    final part)  │         │░░░░░░░░░░░░░░░░░│
//      └─────────────────┘         └─────────────────┘
//
// Model: the starting workpiece IS already the cast part — sand_cast is a
// PROCESS applied to an existing cavity-inverse shape, not a geometry
// modifier.  Its job is to:
//   - validate the part against sand-casting DFM (wall thickness, draft),
//   - record the casting process metadata (material, parting line, …)
//     into the FeatureSignature so process planners downstream can emit a
//     casting work order rather than a CNC tool path.
//
// DFM:
//   DFM-CAST-WALL    : wall_thickness ≥ 3.0 mm (sand cast minimum — finer
//                      walls cannot fill or risk hot tears).
//   DFM-CAST-DRAFT   : draft_angle ≥ 1° (pattern release from sand).
//   DFM-CAST-MAT     : material in known set (warning if unknown — used by
//                      process planner for shrink/feed calculation).
//
// Signature pattern:
//   pattern.kind == "sand_cast"
//   pattern.material, pattern.wall_thickness_mm, pattern.draft_angle_deg,
//   pattern.parting_line_z_mm  (optional)
//   pattern.geometric_change == false
//
// Recognize:
//   Metadata-only — sand casting leaves no recoverable geometric trace
//   beyond the final part shape itself.  recognize() replays the feature
//   history if present, otherwise returns empty.

#include "Datum.hpp"
#include "Skill.hpp"

#include <optional>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sand_cast {

constexpr const char* kSkillId = "sand_cast";

struct Input
{
    std::string             material            = "gray_iron";  // e.g. "gray_iron", "aluminum_a356"
    double                  wall_thickness_mm   = 0.0;
    double                  draft_angle_deg     = 2.0;
    std::optional<double>   parting_line_z_mm   = std::nullopt;
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace sand_cast
}  // namespace koocadcam::skill
