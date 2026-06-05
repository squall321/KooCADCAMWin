#pragma once
// @lat: [[engine/skills#headtube_bearing_seat_zs]]
//
// headtube_bearing_seat_zs — Zero-stack / integrated headset bearing seat.
//
// A "ZS" (zero-stack) headtube houses the cartridge headset bearings inside
// the head tube itself.  The bearings sit on 45-degree conical seats pressed
// into the top and bottom of the head-tube bore.  Geometry:
//   - a straight central head-tube bore (headtube_id) running the full tube
//   - a 45-degree conical seat at the top that opens out to bearing_od
//   - a 45-degree conical seat at the bottom that opens out to bearing_od
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. central head-tube bore (cylinder along the axis)
//   2. top 45-degree conical bearing seat (cone frustum)
//   3. bottom 45-degree conical bearing seat (cone frustum)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT     : every dimension must be > 0.
//   DFM-SEAT-OD   : bearing_od_mm must exceed headtube_id_mm (seat opens out).
//   DFM-ANGLE     : seat_angle_deg must be ~45 (industry zero-stack standard).
//   DFM-STOCK     : bearing_od + seat must fit inside the stock bounding box.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace headtube_bearing_seat_zs {

constexpr const char* kSkillId = "headtube_bearing_seat_zs";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    axis_origin     { 0.0, 0.0, 0.0 };  // head-tube axis entry (top)
    double    headtube_id_mm  = 30.5;             // central bore ID
    double    bearing_od_mm   = 41.0;             // cartridge bearing OD (ZS44)
    double    seat_angle_deg  = 45.0;             // conical seat half-angle
    double    seat_depth_mm   = 8.0;              // axial depth of each seat
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace headtube_bearing_seat_zs
}  // namespace koocadcam::skill
