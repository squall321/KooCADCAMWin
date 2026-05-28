#pragma once
// @lat: [[engine/skills#fillet_edge]]
//
// fillet_edge — round one or more edges with a constant radius.
//
//   Before:                       After (radius_mm = r):
//
//        ┌─────────┐                   ╭─────────╮
//        │         │                  ╱           ╲
//        │         │                 │             │
//        │         │                 │             │
//        └─────────┘                  ╲           ╱
//        (sharp top                    ╰─────────╯
//         and bottom                  (smooth, blended rim)
//         rim edges)
//
// The skill accepts a hybrid edge selector — either a single EdgeDatum, or
// a Z-band (`edges_at_z_mm` + `tolerance_mm`) that selects every edge whose
// midpoint Z lies in that band.  This dual form mirrors how WatchFront's
// `applyCornerRadius` filleted the entire top rim of a case in one shot.
//
// Signature pattern (what the skill leaves on the workpiece):
//   - For each edge filleted: one new cylindrical/toroidal "blend" face
//     replacing the sharp corner.  Topology: # of new cyl/toroidal faces =
//     # of edges filleted.
//
// DFM checks:
//   DFM-004 : radius_mm ≥ 0.2 mm   (standard ball-endmill minimum)
//   DFM-011 : post-fillet adjacent-face angle ≥ 5°  — deferred (needs
//             geometry analysis on the result shape); we emit an info-
//             level note acknowledging that this is not yet enforced.
//
// Recognize:
//   For each cylindrical or toroidal face whose two trim edges parallel a
//   former straight or circular edge of the workpiece, the underlying surface
//   radius is the fillet radius.  Confidence is high (≥ 0.85) when multiple
//   such faces share the same radius (consistent rim) and low (~ 0.6) for
//   isolated blends — the latter could also be intentional design surface.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace fillet_edge {

constexpr const char* kSkillId = "fillet_edge";

// Z-band edge selector — every edge whose midpoint Z is within `tolerance_mm`
// of `z_mm`.  Equivalent to `prim::edgesAtZ(z_mm, tolerance_mm)`.
struct EdgesAtZBand
{
    double z_mm         = 0.0;
    double tolerance_mm = 1e-3;
};

// Either a single edge datum or a Z-band selecting many edges.
using EdgeSelector = std::variant<EdgeDatum, EdgesAtZBand>;

struct Input
{
    EdgeSelector edge_selector;   // single edge OR Z-band
    double       radius_mm = 0.0;

    // ── Optional coarser fallback selector ─────────────────────────────────
    //
    // When `edge_selector` fails to match any surviving edge — typically
    // because preceding Boolean ops in a multi-step process plan
    // re-fingerprint the topology (TopoDS_Edge identities and face_ids
    // change) — the skill re-tries with a Z-band built from these fields.
    // The retry ignores face IDs entirely.  If still no edges match, the
    // skill throws SkillError as before.
    //
    //   z_min_mm / z_max_mm : explicit Z range (mm).  When both are unset
    //                         and the primary selector is an EdgesAtZBand,
    //                         the retry widens its tolerance ×100.
    //   edge_filter_dir     : "" | "all" | "horizontal" | "vertical" — hint
    //                         to drop edges that don't run in the requested
    //                         direction.  "horizontal" keeps edges whose
    //                         curve direction is approximately (1,0,0) or
    //                         (0,1,0) (i.e., Z-component small).  "vertical"
    //                         keeps Z-running edges.  Empty = no filter.
    std::optional<double> z_min_mm;
    std::optional<double> z_max_mm;
    std::string           edge_filter_dir;
};

// Synthesis: apply this skill to a workpiece.
SkillOutput apply(const Workpiece& wp, const Input& in);

// DFM validation.
DFMReport validate(const Workpiece& wp, const Input& in);

// Recognize candidates of this skill in the given workpiece.
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace fillet_edge
}  // namespace koocadcam::skill
