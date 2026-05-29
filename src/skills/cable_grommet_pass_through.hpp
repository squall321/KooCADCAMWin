#pragma once
// @lat: [[engine/skills#cable_grommet_pass_through]]
//
// cable_grommet_pass_through — Smart-electronics compound skill that
// drills a through hole for a rubber grommet that seals against the
// inside edge of the hole.
//
// Sub-cuts (concentric, fused before subtraction):
//   1. THROUGH BORE     — straight cylinder (full plate thickness),
//                         dia = bore_dia_mm
//   2. RETENTION GROOVE — thin annular ring CUT into the inside wall of
//                         the bore, at groove_z_offset_mm from entry,
//                         depth = groove_depth_mm.  This is what the
//                         grommet's rib snaps into.
//   3. ENTRY CHAMFER    — 45° lead-in to ease grommet insertion.
//
// SPEC-TABLE DRIVEN by the standard grommet sizes:
//   "GR-6"  → 6.0 mm bore, 0.4 mm groove, 0.3 mm chamfer
//   "GR-8"  → 8.0 mm bore, 0.5 mm groove, 0.4 mm chamfer
//   "GR-10" → 10.0 mm bore, 0.6 mm groove, 0.5 mm chamfer
//   "GR-12" → 12.0 mm bore, 0.7 mm groove, 0.6 mm chamfer
//   "GR-16" → 16.0 mm bore, 0.9 mm groove, 0.8 mm chamfer
//
// DFM:
//   ISO 16916          — grommet wall ≥ 1 mm
//   DFM-002            — bore Ø ≥ 0.8 mm
//   DFM-GROOVE-WIDTH   — groove width ≥ 0.5 mm (form-grind clearance)
//   DFM-PLATE-THICKNESS — plate ≥ groove_z_offset + groove_width + 1 mm

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>

namespace koocadcam::skill {

class Workpiece;

namespace cable_grommet_pass_through {

constexpr const char* kSkillId = "cable_grommet_pass_through";

struct Input
{
    FaceDatum    entry_face;
    gp_Dir       axis_dir            { 0.0, 0.0, -1.0 };
    double       position_x_mm       = 0.0;
    double       position_y_mm       = 0.0;
    std::string  grommet_size        = "GR-8";    // see spec table
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cable_grommet_pass_through
}  // namespace koocadcam::skill
