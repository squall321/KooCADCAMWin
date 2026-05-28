#pragma once
// @lat: [[engine/skills#probe_point]]
//
// probe_point — INSPECTION skill that records a single-point touch-probe
// measurement against the workpiece.  Geometrically a no-op: the
// workpiece shape is unchanged.  The skill records the probed location
// and the tolerance in the FeatureSignature so downstream process plans
// can emit the corresponding inspection block in the CMM/probe macro.
//
//                              probe_point_3d
//                                    ▼
//                                    ●
//                                  ╱   ╲
//                                 ╱     ╲     ← workpiece face
//                          ──────┴───────┴──────
//                                  (no cut)
//
// Used as an INSPECTION operation: between cutting passes, or after a
// finish cut, the machine probes specific reference points to verify
// that the part is within tolerance — drives the closed-loop machining
// workflow.
//
// Input:
//   probe_point_3d  — gp_Pnt expected location of the touch in world coords
//   tolerance_mm    — acceptance band (e.g. 0.05 mm)
//
// DFM checks:
//   tolerance_mm > 0
//   probe_point should be on or near a workpiece face.  For slice 1 we
//   verify the point is within the workpiece bbox + tolerance.
//
// Recognize:
//   Metadata-only operation — leaves no geometry on the workpiece, so
//   it CANNOT be recovered from final geometry alone.  recognize()
//   always returns an empty vector.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace probe_point {

constexpr const char* kSkillId = "probe_point";

struct Input
{
    gp_Pnt    probe_point_3d { 0.0, 0.0, 0.0 };
    double    tolerance_mm   = 0.05;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace probe_point
}  // namespace koocadcam::skill
