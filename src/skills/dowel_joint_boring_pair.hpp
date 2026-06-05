#pragma once
// @lat: [[engine/skills#dowel_joint_boring_pair]]
//
// dowel_joint_boring_pair — Dowel-joint boring with a glue groove
// (slice 16, furniture / cabinetry hardware).
//
// Edge-to-edge panel joins use a row of fluted wood dowels for alignment and
// a shallow glue groove (channel) running between them so glue spreads
// evenly without starving the joint.  This skill machines that pattern from
// a flat (top) face:
//   1. shallow glue groove (long shallow box along +X through all dowels)
//   2..N+1. N dowel holes spaced at pitch_mm along +X from row_origin
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. glue groove        (long shallow box cut DOWN from top face — built
//                          FIRST as the large connecting feature)
//   2..N+1. dowel holes   (deep cylinders cut DOWN; sequential, the groove
//                          and holes are coaxial along the row — never use a
//                          compound boolean of overlapping cutters)
//
// subfeature_count = dowel_count + 1.
//
// DFM:
//   DFM-INPUT   : all dims must be > 0; dowel_count >= 2.
//   DFM-PITCH   : pitch_mm must exceed dowel_dia_mm (dowel holes not overlap).
//   DFM-GROOVE  : glue_groove_width_mm must be < dowel_dia_mm (groove must be
//                 narrower than the dowels so it does not undercut them).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace dowel_joint_boring_pair {

constexpr const char* kSkillId = "dowel_joint_boring_pair";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      row_origin           { 0.0, 0.0, 0.0 };   // first dowel centre
    double      dowel_dia_mm         { 8.0 };
    int         dowel_count          { 3 };
    double      pitch_mm             { 64.0 };            // dowel spacing
    double      dowel_depth_mm       { 15.0 };            // blind dowel depth
    double      glue_groove_width_mm { 3.0 };
    double      glue_groove_depth_mm { 1.0 };
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace dowel_joint_boring_pair
}  // namespace koocadcam::skill
