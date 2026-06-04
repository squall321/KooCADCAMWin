#pragma once
// @lat: [[engine/skills#camshaft_lobe_compound]]
//
// camshaft_lobe_compound — AUTOMOTIVE ICE engine compound: cam lobe profile
// on a circular camshaft journal.  Profile = base circle + symmetric ramp
// over `duration_deg` rising to `max_lift_mm` peak at the nose (0 deg).
//
// Implementation:
//   We synthesize the cam lobe by REMOVING material from a cylindrical
//   journal everywhere the desired profile r(theta) is smaller than the
//   journal radius.  The cut tool is constructed as a thin axial disc with
//   N polygonal vertices sampled around 360 deg: each vertex sits at the
//   journal OD outside the lobe sector, and pulls in by (max_lift) at the
//   nose.  The lobe sector is the loft-and-cut of (journal_cyl - profile_solid).
//
//   For numerical robustness we approximate: a cylindrical journal of
//   radius R0 + max_lift is cut at all azimuths OUTSIDE the lobe duration
//   to leave only the base circle of radius R0 (= base_circle_radius_mm)
//   except in the duration window where it tapers up to R0 + max_lift_mm.
//
// Sub-feature chain (2 ops):
//   1. CUT cylindrical "outside ring" (R0 + max_lift -> infinity) over
//      lobe_height to expose the journal at base_circle_radius_mm.
//   2. CUT N angular wedges OUTSIDE the duration window so only base_circle
//      remains away from the nose.
//
// Signature:
//   pattern.kind                  = "camshaft_lobe_compound"
//   pattern.engine_feature_type   = "camshaft_lobe"
//   pattern.subfeature_count      = N+1 (1 outer trim + N wedge cuts)
//   pattern.derived_volume_removed_mm3
//   pattern.profile_sample_count
//
// DFM gates:
//   - base_circle_radius > 0
//   - max_lift_mm > 0.5 mm  (SAE J: realistic cam lift)
//   - ramp_angle_deg in (0, 90)
//   - duration_deg in (10, 350)

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace camshaft_lobe_compound {

constexpr const char* kSkillId = "camshaft_lobe_compound";

struct Input
{
    int     face_id                 = -1;
    double  lobe_position_z_mm      =  0.0;
    double  base_circle_radius_mm   = 15.0;
    double  max_lift_mm             =  8.0;
    double  ramp_angle_deg          = 20.0;
    double  duration_deg            = 240.0;
    double  lobe_width_mm           = 12.0;  // axial thickness of the lobe
    int     profile_sample_count    = 36;    // N samples for cut wedges
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace camshaft_lobe_compound
}  // namespace koocadcam::skill
