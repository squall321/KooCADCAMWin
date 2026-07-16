#pragma once
// @lat: [[engine/skills#Layer 5 LLM adapter]]
//
// FeatureTransfer — slice-1 CROSS-PRODUCT feature transfer (watch → phone).
//
// Given a step recovered from one product (e.g. the watch bezel ring found
// by re::inferProcessPlan), re-express it so it executes correctly on a
// DIFFERENT product.  The transfer policy:
//
//   • POSITIONS are SCALED into the destination frame.  A feature centred at
//     40% of the watch's half-extent lands at 40% of the phone's half-extent
//     — placement is relative to the product, so it must re-express.
//   • INTRINSIC dimensions are PRESERVED.  A Ø44 ring stays Ø44 — a machined
//     feature's size is its identity, and the phone is not a scaled watch.
//   • FIT is CLAMPED, ratio-preserving, only when the preserved size cannot
//     physically sit inside the destination (with a safety margin).
//
// This is the OPPOSITE of parts::reframePlanForMoves' resize policy
// (parts/DatumGraph.cpp, scaleFeatureDimensions): there the SAME part was
// resized, so its features' dimensions must grow with it — Ø3 holes on a
// grown PCB must follow the larger bolt circle.  Here the destination is a
// DIFFERENT product; the feature's dimensional identity is exactly the thing
// being transferred, so dimensions stay fixed and only the placement (and,
// at the physical limit, fit) adapts.
//
// VALIDATED ENVELOPE — everything outside it REFUSES loudly
// (transferred=false) instead of emitting a silently-wrong step:
//   • skill whitelist — two placement families:
//       FACE-LOCAL  { "annular_groove" }: center_x/y are offsets from the
//                   resolved entry face's centre (apply()'s in-plane basis);
//       WORLD-XY    { "bolt_circle_pattern", "counterbore_ring_pattern",
//                   "countersink_ring_pattern", "linear_hole_array",
//                   "rectangular_hole_grid" }:
//                   positions (center_x/y, start_x/y, origin_x/y) are WORLD
//                   coordinates
//                   (the member atoms pierce the entry plane at world
//                   (px, py) — see skills/Workpiece.hpp
//                   entryPointOnFacePlane), so they re-express RELATIVE to
//                   the frame centres: v' = dstC + (v − srcC)·(dstHalf/srcHalf).
//                   The per-member hole_centers breadcrumb is stripped
//                   (Diagnostic) — apply() RECOMPUTES every member from
//                   centre + PCD + count + phase (pitch circles) or
//                   start + direction + pitch (linear arrays), which is the
//                   whole re-anchoring story for patterns.  A linear array
//                   fits by PER-MEMBER circles and clamps its PITCH about
//                   the array centre (hole dia and count preserved; refuse
//                   when the clamped pitch would merge adjacent holes).
//       USE-WORLD   { "box_pocket" } (use_world form only, FACE-ANCHORED
//                   only): world_center is the PLACEMENT (the mouth centre
//                   apply() cuts at), not a breadcrumb — its XY re-expresses
//                   frame-relative and its Z re-anchors to the MEASURED
//                   destination entry plane.  Frame-only refuses: this is
//                   the one family whose transferred Z executes VERBATIM
//                   (no execute-time face resolution), so a bbox-approximate
//                   mouth would cut air/short in the executed geometry.
//                   face_xaxis (orientation) is intrinsic.  Fit probes the
//                   rectangle's margin-expanded perimeter AND requires the
//                   mouth OPEN at every sample (material just below the
//                   plane, air just above): a re-expressed centre buried
//                   under a HIGHER deck of a stepped destination refuses —
//                   an internal cavity is not a pocket.  One-factor
//                   ratio-preserving clamp; sub-0.8 mm slivers refuse.
//   • FRONT/BACK (±Z) entry only — a non-±Z face_normal (face-local family)
//     or a non-±Z axis_dir (pattern family; mirrors the skills' own
//     validate error) refuses;
//   • placement: face-anchored EXACT when opts.dst_shape is supplied;
//     otherwise the conservative near-CONCENTRIC envelope (a centre past
//     half the destination half-extent refuses);
//   • destination footprint: face-anchored mode MEASURES it — the feature's
//     outermost circle (footprint radius + margin) is point-classified
//     against the actual solid at the entry plane, and the clamp bisects the
//     largest feasible radius (assumes a footprint star-shaped about the
//     executed centre — true of every shipped product).  A round watch case
//     therefore clamps against its rim where an AABB would over-allow up to
//     √2× on diagonal offsets, bbox-inflating lugs/crown do not inflate the
//     footprint, and a circle over an existing cut's void also clamps.
//     Frame-only mode keeps the per-axis AABB approximation, so phone→watch
//     (round footprint) is only sound face-anchored;
//   • destination thick enough below the entry plane for the cut + 1 mm
//     floor, else clamp — and REFUSE when the clamp would have to eat a
//     fastener-critical intrinsic (a counterbore seat, a countersink cone):
//     those dimensions are the mating identity being transferred, so a
//     destination that cannot hold them cannot receive the feature.
//     Through-drilled bolt circles skip the depth clamp (through is through).
//
// PATTERN FIT: the outermost cut radius is PCD/2 + widest-member-dia/2.
// When it does not fit, the PITCH CIRCLE alone shrinks — the member
// diameters are fastener sizes and stay preserved.  A post-clamp PCD at or
// below the widest member dia (the skill's own validate error) or a
// member-to-member chord below the widest dia (adjacent cuts would merge —
// the result would not be a pattern) refuses.
//
// EXPLICIT REFUSAL BEATS SILENT WRONGNESS: a step whose param family has not
// been audited for the positional/intrinsic split could be silently misplaced
// or mis-sized on the destination; refusing loudly keeps transferred=true
// trustworthy.
//
// Contract: `TransferResult.transferred == false` means "this step was NOT
// made safe for the destination; do not execute it".  The refused step is
// additionally NEUTERED (skill_id = "refused_transfer") so a caller that
// ignores the flag fails loudly at the Executor instead of cutting
// source-frame geometry on the destination.
//
// Known fidelity caveat (measured, follow-up): annular_groove::recognize
// reads the groove WALL extent, which a rim chamfer legitimately shortens
// (the default watch bezel measures 0.7 mm for a nominal 1.0).  A same-part
// replay restores the shape because the chamfer step replays too; a TRANSFER
// carries only the ring, so the destination groove inherits the wall-read
// depth.  Fixing this cleanly means floor-anchored depth semantics in the
// recognizer — tracked as follow-up, out of slice-1 scope.

// Layer-3 dependency.  Same real-header-or-stub fallback as EditOp.hpp.
#if __has_include("process/StepInvocation.hpp")
#  include "process/StepInvocation.hpp"
#else
#  include "_ProcessPlanStub.hpp"
#endif

#include <string>
#include <vector>

class TopoDS_Shape;

namespace koocadcam::adapt {

// ── AnchorFrame ──────────────────────────────────────────────────────────
//
// The product-level coordinate frame a transferred position is expressed
// against: the optimal bounding box's centre + half-extents.  Positions
// scale by the ratio of half-extents (source → destination) per axis.
struct AnchorFrame
{
    double cx = 0.0, cy = 0.0, cz = 0.0;   // optimal-bbox centre
    double hx = 0.0, hy = 0.0, hz = 0.0;   // optimal-bbox half-extents

    // Measure a shape's frame via BRepBndLib::AddOptimal (the repo-wide
    // bbox convention; see engine/primitives/Bbox.hpp).  A null or void
    // shape yields the default (all-zero) frame.
    static AnchorFrame fromShape(const TopoDS_Shape& shape);
};

// ── ParamRole ────────────────────────────────────────────────────────────
//
// The transfer taxonomy over step-param keys (mirrors the key families
// parts/DatumGraph.cpp's extractAxisCoord scans):
//
//   Positional — locations in the product frame; SCALED on transfer.
//   Datum      — face/axis references selecting WHERE on the body to act;
//                re-established on the destination, never scaled.
//   Intrinsic  — the feature's own dimensions/counts; PRESERVED.
//   Diagnostic — recognizer-side breadcrumbs (face ids, world coords);
//                meaningless off the source product, STRIPPED.
enum class ParamRole { Positional, Datum, Intrinsic, Diagnostic };

// Classify one param key.  Unrecognised keys default to Intrinsic (an
// unknown key is safer preserved untouched than guessed at).
ParamRole classifyParam(const std::string& key);

// ── transferFeature ──────────────────────────────────────────────────────

struct TransferOptions
{
    // Minimum clearance between the feature's outer extent and the
    // destination footprint edge before the fit clamp engages.
    double fit_margin_mm = 2.0;

    // FACE-ANCHORED placement (optional): when the destination SHAPE is
    // supplied, transferFeature resolves the entry face at TRANSFER time with
    // the same rule the Executor will use (FaceByNormal ±Z, "largest") and
    // anchors every fit decision to the REAL face:
    //   - the executed ring centre = face centre + (cx, cy) — the radial
    //     clamp tests that WORLD position against the frame boundary, so an
    //     off-centre entry face (a recessed deck, a stepped plate) is handled
    //     exactly instead of assumed centred;
    //   - the depth limit becomes (face Z − frame bottom − 1 mm floor), the
    //     stock actually below the entry plane, not the whole-bbox thickness;
    //   - the near-concentric envelope refusal is replaced by the exact
    //     placement test (the approximation it guarded against is gone);
    //   - an unresolvable entry face refuses.
    // Null (default) keeps the frame-only behaviour and its conservative
    // concentric envelope.  The pointer is borrowed for the call only.
    const TopoDS_Shape* dst_shape = nullptr;
};

struct TransferResult
{
    process::StepInvocation step;               // the adapted copy
    bool transferred = false;                   // false => do NOT execute
    bool fit_clamped = false;                   // dimensions shrunk to fit
    std::vector<std::string> notes;             // human-readable audit trail
};

// Transfer `src` (recovered on `srcProduct`) so it can execute on
// `dstProduct`: strip product-bound keys, inject a portable datum, scale
// positions by the frame half-extent ratios, preserve intrinsics, and clamp
// (ratio-preserving) only when the feature physically cannot fit.
TransferResult transferFeature(const process::StepInvocation& src,
                               const std::string&             srcProduct,
                               const std::string&             dstProduct,
                               const AnchorFrame&             srcFrame,
                               const AnchorFrame&             dstFrame,
                               const TransferOptions&         opts = {});

}  // namespace koocadcam::adapt
