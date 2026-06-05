#pragma once
// @lat: [[engine/skills#axle_box_bearing_housing]]
//
// axle_box_bearing_housing — Rolling-stock axle-box roller-bearing housing
// bore (slice 16, railway / rolling stock).
//
// An axle-box (journal box) carries the wheelset bearing.  The housing is
// machined with three concentric / radial features:
//   1. press-fit bearing bore — sized to a P7 hole basis (interference fit
//      onto the bearing outer ring) via _iso286_fits.hpp p7_max_mm.
//   2. internal retaining-ring groove — DIN 472 internal ring that axially
//      locates the bearing, via _retaining_rings.hpp findDin472.
//   3. grease port hole — radial M-thread port for the grease nipple.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. bearing bore (cylinder cut DOWN the housing axis)
//   2. DIN 472 internal groove (annular ring around the bore)
//   3. grease port (radial cylinder, M-thread clearance, non-overlapping)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT  : every dimension must be > 0.
//   DFM-RING   : ring_size_key must exist in DIN 472 central table.
//   DFM-THREAD : grease_thread_key must exist in central _iso_thread_table.hpp.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace axle_box_bearing_housing {

constexpr const char* kSkillId = "axle_box_bearing_housing";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      axis_origin     { 0.0, 0.0, 0.0 };   // bore axis XY
    double      bearing_od_mm   = 130.0;             // bearing outer Ø (P7 hole)
    double      housing_depth_mm = 40.0;             // bore depth
    std::string ring_size_key;                       // DIN 472 internal
    std::string grease_thread_key;                   // M-thread grease port
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace axle_box_bearing_housing
}  // namespace koocadcam::skill
