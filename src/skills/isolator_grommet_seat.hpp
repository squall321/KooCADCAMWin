#pragma once
// @lat: [[engine/skills#isolator_grommet_seat]]
//
// isolator_grommet_seat — Smart-electronics compound skill for a vibration
// isolator grommet seat (sometimes called an "anti-vibration mount").  The
// seat receives a rubber isolator that decouples a sensitive component
// (HDD, optical sensor, IMU) from chassis vibration.
//
// Sub-cuts:
//   1. COUNTERBORE       — large-diameter blind pocket, dia = counterbore_dia_mm,
//                          depth = counterbore_depth_mm.  The rubber sits in here.
//   2. INSIDE-DIAMETER GROOVE — narrow annular ring CUT into the inside wall
//                          of the counterbore, the rubber's flange snaps in.
//   3. RETENTION NOTCH    — small rectangular keyway cut into the bottom of
//                          the counterbore — prevents the grommet from rotating
//                          under vibration.
//
// SPEC-TABLE DRIVEN by isolator part numbers (vendor-agnostic):
//   "ISO-S"  →  cb 8.0 × 3.0 mm,  groove 0.5 mm @ z=1.0, notch 1.0 × 0.6
//   "ISO-M"  →  cb 12.0 × 5.0 mm, groove 0.7 mm @ z=1.5, notch 1.5 × 0.8
//   "ISO-L"  →  cb 16.0 × 7.0 mm, groove 0.9 mm @ z=2.0, notch 2.0 × 1.0
//   "ISO-XL" →  cb 22.0 × 9.0 mm, groove 1.2 mm @ z=3.0, notch 2.5 × 1.2
//
// DFM:
//   SAE J2627       — isolator wall thickness ≥ 1.0 mm
//   DFM-002         — counterbore Ø ≥ 0.8 mm
//   DFM-GROOVE-WIDTH — groove width ≥ 0.5 mm
//   DFM-CB-DEPTH    — counterbore_depth_mm ≤ workpiece_thickness − 1 mm

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>

namespace koocadcam::skill {

class Workpiece;

namespace isolator_grommet_seat {

constexpr const char* kSkillId = "isolator_grommet_seat";

struct Input
{
    FaceDatum    entry_face;
    gp_Dir       axis_dir            { 0.0, 0.0, -1.0 };
    double       position_x_mm       = 0.0;
    double       position_y_mm       = 0.0;
    std::string  isolator_size       = "ISO-M";   // see spec table
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace isolator_grommet_seat
}  // namespace koocadcam::skill
