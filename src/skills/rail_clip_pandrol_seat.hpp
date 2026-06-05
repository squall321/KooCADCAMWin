#pragma once
// @lat: [[engine/skills#rail_clip_pandrol_seat]]
//
// rail_clip_pandrol_seat — Pandrol e-clip rail fastening seat machined into a
// concrete-sleeper shoulder plate / baseplate (slice 16, railway / rolling
// stock).
//
// A Pandrol e-clip fastening holds the rail foot down through three machined
// features on the baseplate:
//   1. shoulder recess — a rectangular box pocket sized to the rail-foot width
//      that locates the rail foot laterally.
//   2. clip housing pocket — a narrower, deeper rectangular pocket into which
//      the e-clip toe legs are driven.
//   3. anchor bolt hole — an M-thread clearance hole for the hold-down anchor.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. shoulder recess (box cut DOWN from top face)
//   2. clip housing pocket (box, offset from the shoulder recess)
//   3. anchor bolt clearance hole (cylinder, M-thread clearance, non-overlap)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT  : every dimension must be > 0.
//   DFM-THREAD : bolt_thread_key must exist in central _iso_thread_table.hpp.
//   DFM-FOOT   : clip_seat_depth_mm must be < rail_foot_width_mm (sanity).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rail_clip_pandrol_seat {

constexpr const char* kSkillId = "rail_clip_pandrol_seat";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy          { 0.0, 0.0, 0.0 };
    double      rail_foot_width_mm = 76.0;   // shoulder recess width
    double      clip_seat_depth_mm = 10.0;   // recess / pocket depth
    std::string bolt_thread_key;             // M-thread (anchor clearance)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rail_clip_pandrol_seat
}  // namespace koocadcam::skill
