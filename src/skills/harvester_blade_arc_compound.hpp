#pragma once
// @lat: [[engine/skills#harvester_blade_arc_compound]]
//
// harvester_blade_arc_compound — Reciprocating cutter blade (sickle) with
// N triangular knife sections per the standard mowing-machine sickle pattern
// (slice 16, agricultural/forestry).
//
// A combine / mower reciprocating cutter bar uses N triangular knives
// riveted along the sickle back.  Each knife is a flat triangular section
// whose cutting edges are sharpened to the blade edge angle (typ 18-22°).
// Standard pitch is 76.2 mm (3 inch) so a 60 cm bar carries ~8 knife
// sections and the long-line patterns carry ~18 per 60 cm.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   For each of N sections i in [0, N):
//     1. position triangular cutter at x = i * section_pitch
//     2. pr::cut(current, triangle_tool_i)  — one at a time
//
// subfeature_count = N.
//
// DFM:
//   DFM-INPUT      : positive dims required, blade_section_count > 0.
//   DFM-EDGE-ANGLE : edge_angle_deg in [10°, 35°] (typical sickle range).
//   DFM-PITCH      : section_pitch in [50, 100] mm (ASAE sickle pattern).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace harvester_blade_arc_compound {

constexpr const char* kSkillId = "harvester_blade_arc_compound";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    blade_origin       { 0.0, 0.0, 0.0 };  // start of the sickle bar
    int       blade_section_count = 8;               // typ 18 per 60 cm long-line
    double    section_pitch_mm   = 76.2;             // 3 inch ASAE pitch
    double    blade_height_mm    = 50.0;             // sickle width (front-to-back)
    double    edge_angle_deg     = 20.0;             // typ 18-22° per knife edge
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace harvester_blade_arc_compound
}  // namespace koocadcam::skill
