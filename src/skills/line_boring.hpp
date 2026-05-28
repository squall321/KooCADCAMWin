#pragma once
// @lat: [[engine/skills#line_boring]]
//
// line_boring — bore multiple coaxial holes in line through a series of
// bosses / supports.  The classic engine-block crankshaft application:
// every web has a bearing seat that must be perfectly coaxial with the others.
// A single rigid boring bar passes through all the bosses and machines them
// in one setup, guaranteeing collinearity.
//
//            ┌──── boring bar axis ────┐
//            │                         │
//        ┌───┴───┐  ┌───────┐  ┌───┬───┐
//        │   ●───┼──┼───●───┼──┼───●   │   ●  = bore Z positions
//        │       │  │       │  │       │   │  = boss outer surface
//        └───────┘  └───────┘  └───────┘
//          boss1     boss2      boss3
//
// Slice-1 simplification:
//   The synthesis cuts a SINGLE through-cylinder spanning from min(bore_z) -
//   segment_overhang to max(bore_z) + segment_overhang at radius dia/2.
//   This produces ONE merged cylindrical face (not 3 separated bore segments)
//   because in slice-1 we don't model the inter-boss air gaps as real
//   geometry — the stock is assumed to be a continuous block or a series of
//   bosses already joined by webs that the bore passes through.  Future
//   slices can take a real multi-boss stock and produce N discrete bores by
//   leaving the inter-boss material untouched.
//
// Signature pattern:
//   - 1 cylindrical face spanning across all bore_positions
//   - 2 circular edges (top + bottom of the through cylinder)
//   - signature.pattern.bore_positions  (the original Z list)
//   - signature.pattern.dia_mm
//
// DFM checks:
//   DFM-LINEBORE-DIA    : dia ≥ 6 mm (boring bar minimum diameter)
//   DFM-LINEBORE-COUNT  : ≥ 2 bore positions (1 = degenerate to bore_cylindrical)
//   DFM-LINEBORE-AXIS   : all positions share the same axis (the input asserts
//                         this — we accept any Z values along the given axis).
//
// Recognize:
//   A cylindrical face whose two extreme circular edges span an axial
//   length ≥ 2 × dia, AND the cylinder passes through at least 2 distinct
//   planar walls perpendicular to its axis (the bosses).  Hard to
//   differentiate from a plain through_hole in slice-1, hence confidence 0.5.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace line_boring {

constexpr const char* kSkillId = "line_boring";

struct Input
{
    // Axis of the boring bar (location + direction).  In slice-1, the bar
    // runs along this axis through the workpiece.
    FaceCylinderByAxis axis;

    double      dia_mm              = 0.0;

    // Z positions where bores are located along the axis (axis-local Z, i.e.
    // signed distance from axis.Location() projected onto axis.Direction()).
    // Each entry is the CENTER Z of one bore segment.  In slice-1 these are
    // used only to compute the overall span of the through-cylinder.
    std::vector<double> bore_positions_z_mm;

    // How much extra material to bore through on each side of the extreme
    // bore_positions (used to ensure the bar exits cleanly past each end).
    double      segment_overhang_mm = 2.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace line_boring
}  // namespace koocadcam::skill
