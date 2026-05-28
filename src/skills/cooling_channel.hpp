#pragma once
// @lat: [[engine/skills#cooling_channel]]
//
// cooling_channel — small-diameter cylindrical channel carved through a mold
// to circulate coolant near the cavity walls.  Modelled as a polyline of
// straight cylindrical segments between consecutive waypoints, all of
// diameter `dia_mm`.
//
//        (wp0)───seg0───(wp1)
//                          │
//                         seg1
//                          │
//                        (wp2)───seg2───(wp3)
//
// Each segment is a single cylindrical cutter.  All cutters are fused into
// one compound BEFORE the cut so OCCT performs a single Boolean.
//
// Signature pattern:
//   pattern.kind            = "cooling_channel"
//   pattern.dia_mm          = channel diameter
//   pattern.waypoint_count  = N
//   pattern.segment_count   = N - 1
//
// DFM:
//   DFM-COOL-DIA   : dia ∈ [4.0, 16.0] mm (standard cooling line size)
//   DFM-COOL-N     : at least 2 waypoints required
//   DFM-COOL-LEN   : each segment length ≥ dia

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cooling_channel {

constexpr const char* kSkillId = "cooling_channel";

struct Waypoint
{
    double x_mm = 0.0;
    double y_mm = 0.0;
    double z_mm = 0.0;
};

struct Input
{
    std::vector<Waypoint>   waypoints;
    double                  dia_mm = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cooling_channel
}  // namespace koocadcam::skill
