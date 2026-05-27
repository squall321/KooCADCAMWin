#pragma once
// @lat: [[engine/skills#wire_edm]]
//
// wire_edm — wire electrical-discharge machining (a.k.a. "wire EDM").
//
// A thin wire (typ. 0.25 mm brass) passes vertically through the
// workpiece while a spark discharge erodes a kerf along its path.  The
// wire follows an arbitrary closed 2D contour, producing a THROUGH cut
// (no bottom face).  Used for:
//   - tight-tolerance internal openings (square holes, gear teeth)
//   - hard-material parts (tool steels, carbide, titanium)
//   - tall narrow cuts that an end-mill can't deflect-stably reach
//
//                          entry_face (planar, perpendicular to wire)
//                                  │
//                  ────────────────┼──────────────────
//                                  │
//                       ┌──────────┴────────┐
//                       │   (closed 2D      │   ↑  workpiece thickness
//                       │    contour swept  │   │  (wire passes all the
//                       │    vertically)    │   │   way through)
//                       │                   │   ↓
//                       └──────────╴────────┘
//                                  ↑
//                              exit face (parallel to entry)
//
// Signature pattern:
//   - N planar wall faces (one per polygon segment) running through the
//     full thickness of the workpiece
//   - NO bottom face (it's a through cut)
//   - Top + bottom of the cut leave open polygonal edges on the entry and
//     exit faces
//
// DFM checks:
//   DFM-INPUT : contour_waypoints.size() ≥ 3 (closed polygon needs 3 sides)
//   DFM-WIRE-KERF : wire kerf width (0.25 mm) is baked in; informational
//
// Recognize:
//   Identify through-features whose horizontal cross-section is a non-
//   circular polygon — i.e. a set of planar walls all parallel to the
//   wire axis that close around an enclosed area without a bottom face.
//   Confidence ~ 0.7.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace wire_edm {

constexpr const char* kSkillId = "wire_edm";

struct Input
{
    FaceDatum                              entry_face;
    // Closed 2D contour — list of (x_mm, y_mm) waypoints.  The wire
    // path connects them in order; the LAST point is auto-closed to the
    // FIRST.  Must have ≥ 3 unique points.
    std::vector<std::array<double, 2>>     contour_waypoints;
    // Wire axis (perpendicular to entry_face; cuts STRAIGHT through).
    // Phase-1: only ±Z supported.
    gp_Dir                                 axis_dir { 0.0, 0.0, -1.0 };
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace wire_edm
}  // namespace koocadcam::skill
