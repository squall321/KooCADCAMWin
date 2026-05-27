#pragma once
// @lat: [[engine/skills#thread_mill]]
//
// thread_mill — external or internal thread produced by a thread-mill cutter
// following a 3-axis helical tool path.  Geometrically different from
// tap_thread (which uses a self-feeding tap): the thread mill is a smaller-
// diameter tool that traverses one revolution per pitch in the axial
// direction.  Same final pattern (helical groove), different machining.
//
// Signature pattern:
//   - In the FULL implementation: a helical groove face (internal) or a
//     helical ridge (external).
//   - In this APPROXIMATION (slice 2): geometry is UNCHANGED — metadata
//     only.  Process plans address thread-mill intent without altering the
//     pre-thread bore.
//
// DFM checks:
//   DFM-002 : tool_dia_mm ≥ 0.8 mm (end-mill / thread-mill minimum)
//   thread_depth_mm ≥ 1.5 × pitch_mm (engagement)
//
// Recognize:
//   Same approach as tap_thread — scan workpiece.features() for
//   previously registered thread_mill signatures.  Empty after STEP RT.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace thread_mill {

constexpr const char* kSkillId = "thread_mill";

struct Input
{
    FaceDatum   existing_hole_datum;       // for internal threads — bore face
    std::string thread_size      = "M3";
    double      pitch_mm         = 0.5;
    double      thread_depth_mm  = 0.0;
    bool        is_external      = false;  // true → thread an outer cylinder
    double      tool_dia_mm      = 1.0;    // thread-mill diameter
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace thread_mill
}  // namespace koocadcam::skill
