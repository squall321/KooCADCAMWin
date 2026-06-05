#pragma once
// @lat: [[engine/skills#porthole_seal_compound]]
//
// porthole_seal_compound (slice 16) — REAL multi-cut compound feature for
// a marine porthole window opening with O-ring face seal groove and a
// circumferential bolt pattern.  Sub-features (single output shape):
//
//   1. central through-bore (porthole opening)              [pr::cut]
//   2. O-ring face groove (annular ring on entry face)       [pr::cut]
//   3. N bolt-hole pattern on bolt PCD                       [pr::cut × N
//                                                             via SEQUENTIAL
//                                                             gp_Trsf::SetRotation]
//
// CRITICAL — slice-13/14/15 root cause: OCCT 8.0 silently no-ops on
// interpenetrating compound cutters whose convex hulls overlap.  All N
// bolt cylinders share the same axial extent (entry plate thickness) and
// can be radially close, so use a SEQUENTIAL loop of pr::cut, NOT
// pr::cutMany on a compound.
//
// O-ring sizing comes from the central _as568_table.hpp via the AS568D
// dash key (e.g. "-212" → 21.95 mm ID, 3.53 mm CS, groove_depth = 0.75·CS,
// groove_width = 1.40·CS).
//
// Bolt size comes from _iso_thread_table.hpp (metric M-thread) via
// size_key (M6/M8/M10).  bolt_hole_dia = clearance_medium (ISO 273).
//
// DFM standard cited:
//   • Marine porthole opening: typ. 200 / 300 / 400 mm.
//   • bolt_count ∈ {8, 12, 16}.
//   • bolt PCD must satisfy: porthole_dia + 2·groove_width + 2·bolt_hole_dia
//     + 4 mm clearance < bolt_pcd < bolt_pcd + 2·bolt_hole_dia + 4 mm
//     within the entry face (caller-supplied bounds).
//   • AS568 dash and M-thread keys MUST exist in their central tables.
//
// Recognize (geometric primary):
//   1. Find one large axisymmetric cylindrical face (porthole bore).
//   2. Around it: ≥ 8 small coaxial cylinder faces on a single PCD → bolts.
//   3. One annular planar/face-groove pattern between porthole and PCD →
//      O-ring groove.
//   Confidence 0.85 (geometric) / 1.0 (metadata replay).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace porthole_seal_compound {

constexpr const char* kSkillId = "porthole_seal_compound";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm        = 0.0;
    double      center_y_mm        = 0.0;
    gp_Dir      axis_dir           { 0.0, 0.0, 1.0 };   // through-bore along +axis
    double      porthole_dia_mm    = 300.0;             // 200/300/400 mm typical
    std::string o_ring_size_key    = "-325";            // AS568 dash key
    int         bolt_count         = 12;                // 8/12/16
    double      bolt_pcd_mm        = 360.0;             // bolt circle diameter
    std::string bolt_thread_size_key = "M8";            // M6/M8/M10
    double      plate_thickness_mm = 12.0;              // bulkhead/plate thickness
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace porthole_seal_compound
}  // namespace koocadcam::skill
