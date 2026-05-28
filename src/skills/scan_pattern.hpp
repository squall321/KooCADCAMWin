#pragma once
// @lat: [[engine/skills#scan_pattern]]
//
// scan_pattern — INSPECTION skill that probes a 2-D pattern of points
// across a planar face (flatness / form check, e.g. for a sealing
// surface, optical mount face, or precision datum).  Geometrically a
// no-op: the workpiece shape is unchanged.  The skill computes the
// actual probe XYZ locations from the chosen pattern type and the face
// geometry, and stores all probe points in the FeatureSignature so the
// downstream process plan can emit the CMM/probe macro.
//
//    Patterns (in the entry-face local UV plane):
//
//       "grid"           "spiral"           "diagonal"
//      ●  ●  ●  ●  ●     ●─●─●               ●
//      ●  ●  ●  ●  ●    ╱     ╲                ●
//      ●  ●  ●  ●  ●   ●   ●   ●                 ●
//      ●  ●  ●  ●  ●    ╲     ╱                    ●
//      ●  ●  ●  ●  ●     ●─●─●                       ●
//
// Input:
//   entry_face_datum — the planar face to scan
//   pattern          — "grid" | "spiral" | "diagonal"
//   point_count      — target count (default 25)
//   tolerance_mm     — acceptance band per probed point
//
// DFM checks:
//   pattern is one of the supported strings
//   point_count >= 4
//   face area >= 100 mm² (otherwise pattern doesn't fit)
//   tolerance_mm > 0
//
// Recognize:
//   Metadata-only operation — leaves no geometry on the workpiece, so
//   it cannot be recovered from final geometry alone.  recognize()
//   always returns an empty vector.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace scan_pattern {

constexpr const char* kSkillId = "scan_pattern";

struct Input
{
    FaceDatum     entry_face_datum;
    std::string   pattern      = "grid";    // "grid" | "spiral" | "diagonal"
    int           point_count  = 25;
    double        tolerance_mm = 0.05;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace scan_pattern
}  // namespace koocadcam::skill
