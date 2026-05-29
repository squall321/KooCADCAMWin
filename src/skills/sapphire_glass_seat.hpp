#pragma once
// @lat: [[engine/skills#sapphire_glass_seat]]
//
// sapphire_glass_seat — COMPOUND FEATURE (3 primitive cuts).
//
// Watch-case sapphire crystal seat: a tiered bezel-inner bore that supports
// the underside of the sapphire glass plus a small O-ring groove on the
// vertical shoulder wall to seal the case top.  Chains:
//
//   1. outer bore cut: large cylinder (Ø = glass_dia + 0.1 mm slip-fit) cut
//      from the case top down to ledge_depth.
//   2. inner bore cut: smaller cylinder (Ø = glass_dia − 1.0 mm) cut deeper
//      to form the ledge shoulder for the glass to rest on.  These two
//      cylinders are CONCENTRIC and OVERLAPPING → FUSED then single cut
//      (counterbore-style).
//   3. O-ring groove: thin annular groove cut into the VERTICAL shoulder
//      wall of the outer bore (so the O-ring side-seals against the glass
//      edge).  AS568 -004 family (CS ≈ 1.0 mm).
//
// Output: case top with stepped bore + side groove.  Single FeatureSignature,
// pattern.is_compound = true, subfeature_count = 3.
//
// DFM gates:
//   - glass_dia ∈ [10, 60] mm (typical wristwatch)
//   - ledge_depth ≥ 1.0 mm and ≤ case_thickness − 1.0 mm
//   - inner_bore_dia ≤ glass_dia − 0.8 mm (enough ledge support)
//   - O-ring groove cross-section follows AS568 -004 family (CS = 1.78 mm
//     × 0.75 depth → 1.33 mm; we use 1.0 mm with margin)
//   - shoulder height ≥ groove width + 0.4 mm (groove fits in shoulder)
//
// Recognize: metadata replay (compound concentric bores with side groove are
// ambiguous geometrically).  Confidence 1.0 from history.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sapphire_glass_seat {

constexpr const char* kSkillId = "sapphire_glass_seat";

struct Input
{
    FaceDatum   case_face;            // top face of case (where bore enters)
    double      center_x_mm     = 0.0;
    double      center_y_mm     = 0.0;
    gp_Dir      axis_dir        { 0.0, 0.0, -1.0 };
    double      glass_dia_mm    = 0.0;   // sapphire OD
    double      ledge_depth_mm  = 1.5;   // outer-bore depth = sapphire seat
    double      inner_bore_dia_mm = 0.0; // 0 → auto = glass_dia − 1.0
    double      inner_bore_depth_mm = 0.0; // 0 → auto = ledge_depth + 1.5
    double      o_ring_cs_mm    = 1.0;   // AS568 -004 family
    double      o_ring_depth_mm = 0.4;   // groove depth into shoulder wall
    double      o_ring_z_offset_mm = 0.5; // axial offset from top
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sapphire_glass_seat
}  // namespace koocadcam::skill
