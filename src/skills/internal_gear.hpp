#pragma once
// @lat: [[engine/skills#internal_gear]]
//
// internal_gear — REAL involute-profile internal (ring) gear (slice 13).
//
// Builds an annular ring (outer Ø ring_outer_dia, inner Ø = 2·r_root) and
// cuts z involute tooth gaps INWARD from the inner bore.  The teeth point
// radially INWARD (opposite of an external spur).
//
// DFM checks:
//   DFM-GEAR-MOD    module_mm > 0.3
//   DFM-GEAR-CT     teeth_count in [8, 200]
//   DFM-GEAR-FW     face_width > 0
//   DFM-GEAR-PA     pressure_angle in [14.5, 30]
//   DFM-GEAR-OD     ring_outer_dia > 2·r_tip + 4·m (wall thickness)

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace internal_gear {

constexpr const char* kSkillId = "internal_gear";

struct Input
{
    double  module_mm           = 2.0;
    int     teeth_count         = 30;
    double  face_width_mm       = 10.0;
    double  pressure_angle_deg  = 20.0;
    double  ring_outer_dia_mm   = 80.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace internal_gear
}  // namespace koocadcam::skill
