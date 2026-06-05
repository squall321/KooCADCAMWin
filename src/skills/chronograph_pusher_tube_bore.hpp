#pragma once
// @lat: [[engine/skills#chronograph_pusher_tube_bore]]
//
// chronograph_pusher_tube_bore — Chronograph pusher tube bore cut radially
// from a watch case side (slice 16, watch advanced complications).
//
// A chronograph pusher (start/stop/reset button) passes through a threaded
// tube screwed into the case flank.  The case-side machining is:
//   - a radial through bore for the pusher tube body
//   - an AS568 O-ring gasket groove (annular ring on the bore axis) for the
//     water seal between tube and pusher stem
//   - a threaded retaining-ring relief (wider annular relief at the mouth)
//     so the threaded tube collar can be screwed home flush
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. pusher tube bore (cylinder cut along bore_dir from the side face)
//   2. AS568 O-ring gasket groove (annular ring, coaxial with bore)
//   3. threaded retaining-ring relief (annular relief at the bore mouth)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT     : pusher_bore_dia_mm must be > 0.
//   DFM-AS568     : o_ring_size_key must exist in central AS568 table.
//   DFM-THREAD    : thread_size_key must exist in central metric thread table.
//   DFM-CLEARANCE : retaining-ring relief OD must exceed pusher bore dia.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace chronograph_pusher_tube_bore {

constexpr const char* kSkillId = "chronograph_pusher_tube_bore";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      bore_origin       { 0.0, 0.0, 0.0 };   // point on the side face
    gp_Dir      bore_dir          { 1.0, 0.0, 0.0 };   // inward radial (e.g. -X)
    double      pusher_bore_dia_mm = 2.5;
    std::string o_ring_size_key;                       // AS568 dash, e.g. "-006"
    std::string thread_size_key;                       // metric, e.g. "M4"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace chronograph_pusher_tube_bore
}  // namespace koocadcam::skill
