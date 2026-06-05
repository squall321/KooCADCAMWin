#pragma once
// @lat: [[engine/skills#grounding_lug_layin_seat]]
//
// grounding_lug_layin_seat — WEEB / lay-in grounding lug seat for PV arrays
// (slice 16, solar / PV mounting).
//
// A lay-in lug bonds the equipment-grounding conductor (EGC) to the module
// rail.  The seat is a shallow pocket; a half-round groove receives the bare
// copper conductor laid into it; a set-screw (M-thread) clamps the conductor.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. lug seat pocket (box cut DOWN from top face)
//   2. lay-in conductor groove (horizontal half-cylinder along +X)
//   3. set-screw clearance hole (M-thread clearance cylinder, vertical)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT   : every dimension must be > 0.
//   DFM-THREAD  : set_screw_thread_key must exist in central metric thread table.
//   DFM-GROOVE  : conductor_groove_dia_mm must be < seat_wid_mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace grounding_lug_layin_seat {

constexpr const char* kSkillId = "grounding_lug_layin_seat";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy              { 0.0, 0.0, 0.0 };
    double      seat_len_mm            = 24.0;
    double      seat_wid_mm            = 12.0;
    double      seat_depth_mm          = 2.0;
    double      conductor_groove_dia_mm = 6.0;
    std::string set_screw_thread_key;             // from _iso_thread_table.hpp "M5".."M8"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace grounding_lug_layin_seat
}  // namespace koocadcam::skill
