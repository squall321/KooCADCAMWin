#pragma once
// @lat: [[engine/skills#exhaust_manifold_port_compound]]
//
// exhaust_manifold_port_compound — AUTOMOTIVE ICE engine compound: exhaust
// port that transitions from RECTANGULAR inlet (at the cylinder head face)
// to ROUND outlet (where the downpipe slip-fits).  Built as 3 ops:
//
//   1. CUT rectangular inlet pocket on entry face (W x H, shallow).
//   2. CUT lofted transition from rectangle to circle (BRepOffsetAPI_ThruSections).
//   3. CUT cylindrical outlet hole at the far face.
//
// Signature:
//   pattern.kind                  = "exhaust_manifold_port_compound"
//   pattern.engine_feature_type   = "exhaust_manifold_port"
//   pattern.subfeature_count      = 3
//   pattern.derived_volume_removed_mm3
//
// DFM gates:
//   - all positive
//   - port_angle_deg in [-45, 45]  (manifold slope from head normal)
//   - port_outlet_dia_mm > 0
//   - transition_length_mm > 0

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace exhaust_manifold_port_compound {

constexpr const char* kSkillId = "exhaust_manifold_port_compound";

struct Input
{
    int     face_id                  = -1;
    double  center_x_mm              =  0.0;
    double  center_y_mm              =  0.0;
    double  port_inlet_rect_w_mm     = 30.0;
    double  port_inlet_rect_h_mm     = 25.0;
    double  port_outlet_dia_mm       = 32.0;
    double  transition_length_mm     = 40.0;
    double  port_angle_deg           =  0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace exhaust_manifold_port_compound
}  // namespace koocadcam::skill
