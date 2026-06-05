#pragma once
// @lat: [[engine/skills#tracker_slew_pivot_bore]]
//
// tracker_slew_pivot_bore — Single-axis solar tracker slew/pivot bore
// (slice 16, solar / PV mounting).
//
// A horizontal single-axis tracker (HSAT) torque tube rotates in a pivot
// bearing.  This skill machines: the bushing seat bored to an H7 fit (so the
// pressed bushing OD locates precisely), a grease-fitting (zerk) tapped hole
// for lubrication, and a stop-pin hole that limits the slew travel.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. pivot bore + H7 bushing seat (cylinder bored to H7-max diameter)
//   2. grease-fitting tapped hole (M-thread pilot, side / vertical)
//   3. stop-pin hole (plain cylinder)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT   : every dimension must be > 0.
//   DFM-THREAD  : grease_thread_key must exist in central metric thread table.
//   DFM-SEAT    : bushing_od_mm must be > pivot_bore_dia_mm (seat is wider).
//   DFM-H7RANGE : bushing_od_mm must fall in the ISO 286-1 band coverage.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tracker_slew_pivot_bore {

constexpr const char* kSkillId = "tracker_slew_pivot_bore";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      axis_origin       { 0.0, 0.0, 0.0 };
    double      pivot_bore_dia_mm = 24.0;
    double      bushing_od_mm     = 30.0;
    double      bore_depth_mm     = 18.0;
    std::string grease_thread_key;               // from _iso_thread_table.hpp "M6"/"M8"
    double      stop_pin_dia_mm   = 8.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tracker_slew_pivot_bore
}  // namespace koocadcam::skill
