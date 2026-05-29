#pragma once
// @lat: [[engine/skills#full_penetration_butt_prep]]
//
// full_penetration_butt_prep — COMPOUND MACRO that prepares one plate edge
// for a full-penetration butt weld per AWS A2.4 BUC1 (single-V groove).
//
// Geometric sub-features (≥ 3):
//   1. Bevel A — angled prism cut on the +Y side of the edge.
//   2. Bevel B — angled prism cut on the -Y side of the edge (mirror).
//   3. Root radius — small cylinder fillet at the bottom of the V, removing
//      the sharp internal corner so the welder can lay a clean root pass.
//
// The root_face_mm parameter is the LAND at the bottom of the V that is NOT
// removed (full plate_t = root_face + (groove vertical depth)).
//
//                        ┌───────────────┐
//                        │  \         /  │  ← bevel A / bevel B (angle/2)
//                        │   \       /   │
//                        │    \     /    │     (groove_angle is the FULL
//                        │     \   /     │      included angle of the V)
//                        │      \ /      │
//                        │  ───  v  ───  │  ↑ root_face_mm (unmachined land)
//                        │               │  ↓
//                        └───────────────┘
//                                ↕
//                              edge_dir (the welded edge runs along this dir)
//
// Implementation strategy (slice-1):
//   1. Resolve the entry face (top face of the plate).
//   2. Build two box cutters tilted by groove_angle/2 — together they carve
//      the V-groove footprint.
//   3. Add a small root-radius cylinder along the bottom of the V (fused
//      with the two bevel boxes into a single connected cutter) so the
//      Boolean op produces one clean compound cut.
//
// DFM checks (AWS A2.4 / AWS D1.1):
//   DFM-WELD-ANGLE  : groove_angle must be in [30°, 90°] (single-V range).
//   DFM-WELD-LAND   : root_face_mm in [0.5, plate_t / 3] (DIN EN ISO 9692-1).
//   DFM-WELD-PLATE  : plate_t >= 3 mm (below: no prep needed, AWS BTC-P4).
//
// Recognize (compound metadata replay):
//   We can't fingerprint a V-groove uniquely from B-rep alone (planar
//   inclined faces look like chamfer faces).  recognize() falls back to
//   metadata REPLAY from the workpiece's feature history: it scans
//   wp.features() for any signature whose pattern.kind == kSkillId and
//   replays the params verbatim with confidence = 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace full_penetration_butt_prep {

constexpr const char* kSkillId = "full_penetration_butt_prep";

struct Input
{
    FaceDatum   entry_face;                          // the top (entry) face of the plate
    gp_Dir      edge_dir         { 1.0, 0.0,  0.0 }; // direction the welded edge runs along
    double      plate_t          = 0.0;              // plate thickness (Z)
    double      groove_angle     = 60.0;             // FULL included V angle in degrees
    double      root_face_mm     = 1.5;              // un-machined land at bottom of V
    double      edge_center_x_mm = 0.0;              // XY of the edge center on the entry face
    double      edge_center_y_mm = 0.0;
    double      edge_length_mm   = 50.0;             // groove runs this length along edge_dir
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace full_penetration_butt_prep
}  // namespace koocadcam::skill
