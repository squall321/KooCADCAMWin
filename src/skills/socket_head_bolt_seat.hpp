#pragma once
// @lat: [[engine/skills#socket_head_bolt_seat]]
//
// socket_head_bolt_seat — COMPOUND fastener-seat for a socket-head cap screw
// (SHCS, ISO 4762 / DIN 912).  Chains:
//
//   sub-feature 1: through pilot drill (shank clearance)
//   sub-feature 2: counterbore (cylindrical head seat: dia = head_dia + slip,
//                  depth = head_height + 0.5 mm so socket key has clearance)
//   sub-feature 3: chamfer at entry (45° × 0.3 mm) so head edge doesn't snag
//
// All three concentric.  Standard M-spec head dimensions (ISO 4762):
//
//   M-spec | clearance | head dia | head height
//   -------+-----------+----------+-------------
//   M2     | 2.4       | 3.8      | 2.0
//   M2.5   | 2.9       | 4.5      | 2.5
//   M3     | 3.4       | 5.5      | 3.0
//   M4     | 4.5       | 7.0      | 4.0
//   M5     | 5.5       | 8.5      | 5.0
//   M6     | 6.6       | 10.0    | 6.0
//   M8     | 9.0       | 13.0    | 8.0
//
// DFM:
//   DFM-002             : derived pilot < 0.8 mm
//   DFM-SHCS-SIZE       : fastener_size unknown
//   DFM-SHCS-THICKNESS  : stock thickness too thin to host counterbore depth
//   DFM-SHCS-MARGIN     : seat dia + slip falls below pilot dia (impossible)
//
// Recognize: metadata replay, confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace socket_head_bolt_seat {

constexpr const char* kSkillId = "socket_head_bolt_seat";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    std::string fastener_size;            // "M2".."M8"
    double      head_slip_mm   = 0.4;     // diametral slip fit on the head
};

double clearanceDiameterFor (const std::string& size);
double headDiameterFor      (const std::string& size);
double headHeightFor        (const std::string& size);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace socket_head_bolt_seat
}  // namespace koocadcam::skill
