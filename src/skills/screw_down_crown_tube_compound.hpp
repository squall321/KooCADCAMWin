#pragma once
// @lat: [[engine/skills#screw_down_crown_tube_compound]]
//
// screw_down_crown_tube_compound — Screw-down crown tube bore in a dive /
// sports watch case (slice 16, watch advanced complications).
//
// A screw-down crown threads onto a tube pressed/screwed into the case
// flank; the tube carries the gaskets that make the crown water-resistant.
// The case-side machining (cut radially from a side face) is:
//   - a crown tube bore (radial cylinder for the tube body)
//   - an external thread relief (annular relief where the tube's external
//     thread engages the case)
//   - 2 AS568 O-ring grooves (the dual static seals on the tube bore wall)
//   - a tube shoulder counterbore (wider mouth seating the tube flange)
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. crown tube bore (cylinder)
//   2. external thread relief (annular)
//   3. O-ring groove #1 (annular)
//   4. O-ring groove #2 (annular)
//   5. tube shoulder counterbore (annular relief at mouth)
//
// subfeature_count = 5.
//
// DFM:
//   DFM-INPUT     : tube_bore_dia/shoulder_dia must be > 0.
//   DFM-AS568     : o_ring_size_key must exist in central AS568 table.
//   DFM-THREAD    : thread_size_key must exist in central metric thread table.
//   DFM-SHOULDER  : shoulder_dia_mm must exceed tube_bore_dia_mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace screw_down_crown_tube_compound {

constexpr const char* kSkillId = "screw_down_crown_tube_compound";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      bore_origin       { 0.0, 0.0, 0.0 };   // point on the side face
    gp_Dir      bore_dir          { 1.0, 0.0, 0.0 };   // inward radial (e.g. -X)
    double      tube_bore_dia_mm   = 3.0;
    std::string thread_size_key;                       // metric, e.g. "M5"
    std::string o_ring_size_key;                       // AS568 dash, e.g. "-006"
    double      shoulder_dia_mm    = 6.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace screw_down_crown_tube_compound
}  // namespace koocadcam::skill
