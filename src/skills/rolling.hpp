#pragma once
// @lat: [[engine/skills#rolling]]
//
// rolling — reduce thickness of a sheet by passing through rollers.
// Also implicitly extrudes the part length (volume preservation), but for
// slice-1 we model the result as a thickness reduction only and report
// pre/post dimensions in the signature.
//
//        ┌──┐                                ┌──┐
//   ────►│  │─── thick sheet ─────────────►  │  │  ── thinner sheet ──►
//        └──┘                                └──┘
//        roller                              roller
//
// Synthesis strategy (slice-1, robust):
//   Rather than non-uniform OCCT scaling (which is not a strict gp_Trsf
//   transformation), we rebuild the workpiece as a NEW cuboid with the
//   same XY extents but a target thickness.  This is the cleanest
//   topological representation and avoids degenerate scale operations on
//   complex shapes.  For arbitrary stock, the bbox is used as the
//   reference.
//
// Signature pattern:
//   - 2 large parallel planar faces (sheet top + bottom)
//   - separation between them = target_thickness
//   - metadata recording pre/post thickness and reduction percentage
//
// DFM checks:
//   DFM-ROLL-PASS   : reduction_pct_per_pass ≤ 50% (industrial typical)
//   DFM-ROLL-MIN    : target_thickness ≥ 0.1 × original (max cold-roll reduction
//                     of 90% across multiple passes)
//   DFM-INPUT       : target_thickness > 0
//
// Recognize:
//   Pattern is metadata-driven (a flat sheet alone is indistinguishable
//   from a cuboid stock).  Recognize returns the smallest-extent direction
//   as the rolled axis; the recovered params include the observed thickness
//   but the original (pre-roll) thickness is unrecoverable from geometry
//   alone — set to 0.0 in the recovered params unless metadata is present.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rolling {

constexpr const char* kSkillId = "rolling";

struct Input
{
    FaceDatum   entry_face;                      // sheet top (used for DFM context)
    double      target_thickness_mm = 0.0;       // post-rolling thickness
    double      reduction_pct_per_pass = 30.0;   // 0…50 typical
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Helper — return the smallest bbox extent (= sheet thickness for cuboid stock).
double currentThickness(const Workpiece& wp);

}  // namespace rolling
}  // namespace koocadcam::skill
