#pragma once
// @lat: [[engine/skills#form_turn]]
//
// form_turn — turning operation with a profiled (form) cutter that produces
// an arbitrary radial profile along the workpiece Z axis.
//
//                ┌─────────────────┐  R_initial
//                │                 │
//                │  ___            │
//        ────────┘ /   \    ___    └──────── ← profile envelope (max r at each z)
//                 /     \  /   \
//                /       \/     \             radii sampled from the input
//                                            polyline of (z, radius) waypoints
//                                            (monotonic in z, each r > 0)
//
// Synthesis: the input polyline `(z_i, r_i)` defines an axisymmetric profile.
// We build a closed wire that traces the profile out to a constant outer
// envelope `R_envelope = stock_outer + overhang` (so the revolved face is the
// region to REMOVE), then revolve it 360° about the Z axis with
// `BRepPrimAPI_MakeRevol(profileFace, axis, 2π, copy=true)` to obtain a solid
// tool, then `BRepAlgoAPI_Cut` from the workpiece.
//
// Signature pattern (metadata-only because arbitrary profiles produce
// arbitrary face counts — geometric recognition is sketched but loose):
//   - kind = form_turn, axis = Z, plus the original (z, r) polyline so
//     recognize() can perform metadata replay.
//
// DFM checks:
//   DFM-INPUT   : ≥ 2 polyline points
//   DFM-INPUT   : z monotonic (strictly increasing)
//   DFM-INPUT   : every radius > 0.4 (min cutter contact)
//   DFM-LATHE   : every profile radius < current stock outer radius
//   DFM-LATHE   : Z range overlaps the stock bbox
//
// Recognize:
//   1. Metadata replay (preferred — exact recovery of the polyline).
//   2. Geometric fallback (lower confidence): scan for any rotational-axis
//      surface coaxial with Z whose radius is less than the stock outer.
//      Returns a sampled approximation of the profile.

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace form_turn {

constexpr const char* kSkillId = "form_turn";

struct ProfilePoint
{
    double z_mm = 0.0;
    double r_mm = 0.0;
};

struct Input
{
    // Polyline of (z, r) waypoints, sorted by z ascending.  ≥ 2 points.
    std::vector<ProfilePoint>  profile;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace form_turn
}  // namespace koocadcam::skill
