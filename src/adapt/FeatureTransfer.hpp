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
// Slice-1 VALIDATED ENVELOPE — everything outside it REFUSES loudly
// (transferred=false) instead of emitting a silently-wrong step:
//   • skill whitelist { "annular_groove" };
//   • FRONT/BACK (±Z) entry only — a surviving non-±Z face_normal refuses
//     (the fit clamp maps depth→Z, footprint→X/Y);
//   • near-CONCENTRIC placement — the executed position is FACE-LOCAL
//     (annular_groove::apply offsets from the resolved face's centre), which
//     coincides with the frame-ratio math only while the entry face is centred
//     on the product frame (true of both shipped products) and the offset is
//     small; a scaled centre past half the destination half-extent refuses.
//     Full face-anchored placement (resolve the destination face at transfer
//     time) is documented follow-up;
//   • rectangular-footprint destination — the AABB clamp does not model a
//     round boundary (phone→watch stays out of scope);
//   • destination thick enough for the groove + 1 mm floor, else refuse.
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
