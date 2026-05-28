#pragma once
// @lat: [[engine/skills#ultra_precision_diamond_turn]]
//
// ultra_precision_diamond_turn — single-point diamond turning (SPDT) of
// non-ferrous optics-grade material (aluminum, copper, OFE copper, brass,
// nickel-plated steel, polymers).  Produces optical-grade surfaces with
// roughness Ra ≤ 5 nm (10 nm typical) and form errors < 0.1 µm.
//
// Geometrically identical to a `turn_external` operation; this skill is
// METADATA-ONLY — synthesis delegates and re-stamps the signature.
//
// Special tooling:
//   - single-crystal natural diamond tool, nose radius 0.1 – 5 mm
//   - ferrous materials (steel, cast iron) are FORBIDDEN: carbon dissolves
//     graphitically into the diamond → catastrophic tool wear in seconds
//   - aerostatic spindle, ambient-controlled enclosure
//
// Signature pattern:
//   - turn_external topology + pattern.kind = "ultra_precision_diamond_turn"
//   - tooling.tool_material  = "single_crystal_diamond"
//   - tooling.cutting_speed_sfm ≈ 1500 (very high)
//   - tooling.feed_per_tooth_mm ≈ 0.005 (sub-um chip)
//   - tooling.extra.target_ra_nm    (caller-supplied)
//   - tooling.extra.form_error_um   (caller-supplied)
//
// DFM checks:
//   DFM-INPUT          : final_dia_mm > 0; end_z > start_z
//   DFM-SPDT-FERROUS   : material ∈ {steel, iron, ferrous_*} → error
//                        (graphitization destroys diamond tool)
//   DFM-SPDT-RA        : target_ra_nm < 1 nm → warning (below practical limit)
//   DFM-SPDT-RA        : target_ra_nm > 100 nm → warning (use conv. turning)
//   DFM-SPDT-NOSE-R    : tool_nose_r_mm ∉ [0.1, 5.0] → warning

#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ultra_precision_diamond_turn {

constexpr const char* kSkillId = "ultra_precision_diamond_turn";

struct Input
{
    double       start_z_mm        = 0.0;
    double       end_z_mm          = 0.0;
    double       final_dia_mm      = 0.0;
    int          roughing_passes   = 1;
    double       tool_nose_r_mm    = 0.5;     // diamond nose, 0.1 – 5 mm
    double       target_ra_nm      = 5.0;     // optical grade target
    std::string  workpiece_material = "aluminum_6061";  // ferrous → error
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ultra_precision_diamond_turn
}  // namespace koocadcam::skill
