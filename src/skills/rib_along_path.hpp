#pragma once
// @lat: [[engine/skills#rib_along_path]]
//
// rib_along_path — Add a vertical rib that follows a polyline path across a
// host face.  The path is given as a sequence of (u,v) points in face-local
// UV space; we map them to world XYZ by treating the host face as a XY plane
// at the face's average Z (slice-1 simplification — good enough for the
// flat caseback / phone bottom plate where ribs are most useful).
//
// SolidWorks "Rib" along a sketched path.
//
//        host face (XY)                ┌──┐
//        ────────────              ┌──┘  └──┐
//        ── path: A→B→C  ───►      │ rib seg│   stack of thin boxes
//        ────────────              └────────┘   fused onto workpiece
//
// Algorithm:
//   1. Resolve host face → compute average Z & host face outward normal.
//   2. For each segment (Pi, Pi+1) in the polyline:
//        - length = ‖Pi+1 − Pi‖
//        - build a thin box centred on the segment midpoint, oriented along
//          the segment, dims (length × thickness × height) extruded along
//          +Z (i.e. the host face outward direction).
//   3. Fuse all segment boxes onto the workpiece.
//
// Signature pattern:
//   pattern.kind             = "rib_along_path"
//   pattern.is_compound      = true
//   pattern.parameters       = { host_face_id, rib_thickness_mm, rib_height_mm,
//                                segment_count }
//   pattern.derived_volume   = sum(box volumes)
//
// DFM:
//   DFM-INPUT          : rib_thickness_mm > 0; rib_height_mm > 0; path
//                        polyline must have ≥ 2 points.

#include "Datum.hpp"
#include "Skill.hpp"

#include <utility>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rib_along_path {

constexpr const char* kSkillId = "rib_along_path";

struct Input
{
    int                                       host_face_id     = -1;
    std::vector<std::pair<double, double>>    path_polyline;       // (u,v) ≈ (x,y)
    double                                    rib_thickness_mm = 1.0;
    double                                    rib_height_mm    = 2.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rib_along_path
}  // namespace koocadcam::skill
