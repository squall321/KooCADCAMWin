#pragma once
// @lat: [[engine/skills#hard_turning]]
//
// hard_turning — outer-diameter turning of HARDENED steel (HRC > 50).
//
// Hard turning is a finishing operation that replaces conventional grind
// processes on hardened workpieces.  Geometrically the result is identical
// to `turn_external`: the stock's outer radius is reduced from R_initial
// to R_final over a Z range, with two annular shoulders at the endpoints.
// The DIFFERENCE is in TOOLING and DFM — hard turning uses a CBN insert,
// runs at much lower feed/SFM, and requires a rigid lathe.
//
// This skill is METADATA-ONLY: synthesis delegates to `turn_external` and
// re-stamps the FeatureSignature so the strategy + CBN-tooling info wins.
//
//                ┌─────────────────┐  R_initial
//                │   HRC > 50      │
//        ────────┘   ┌─────────┐   └──────── ← end_z
//                    │ CBN cut │              R_final
//                    │         │
//        ────────────┘         └──────────── ← start_z
//
// Signature pattern:
//   - 1 cylindrical face (axis = +Z, radius = R_final)
//   - 2 annular planar shoulder faces
//   - pattern.kind     = "hard_turning"
//   - pattern.strategy = "hard_turning"
//   - tooling.tool_material = "cbn"
//   - tooling.cutting_speed_sfm low (~180 sfm)
//   - tooling.feed_per_tooth_mm low (~0.05 mm/rev)
//
// DFM checks:
//   DFM-INPUT      : final_dia_mm > 0; end_z > start_z; roughing_passes ≥ 1
//   DFM-HARDTURN-HRC : workpiece_hrc < 50 → error (use turn_external instead)
//   DFM-HARDTURN-HRC : workpiece_hrc > 65 → error (above CBN practical limit)
//   DFM-LATHE      : final_dia_mm ≥ current outer dia (nothing to remove)
//   DFM-HARDTURN-FEED  : feed_override_mm > 0.1 → warning (CBN chips at high feed)
//
// Recognize:
//   Topology overlaps with turn_external — we surface a feature only when
//   the workpiece carries a previously-stamped hard_turning signature.
//   Otherwise turn_external dominates (confidence 0.92 vs our 0.40 marker).

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hard_turning {

constexpr const char* kSkillId = "hard_turning";

struct Input
{
    double  start_z_mm           = 0.0;
    double  end_z_mm             = 0.0;
    double  final_dia_mm         = 0.0;
    int     roughing_passes      = 1;
    double  workpiece_hrc        = 55.0;   // Rockwell C of the (hardened) stock
    double  feed_override_mm     = 0.0;    // 0 = use CBN default 0.05 mm/rev
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hard_turning
}  // namespace koocadcam::skill
