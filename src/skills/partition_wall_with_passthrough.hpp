#pragma once
// @lat: [[engine/skills#partition_wall_with_passthrough]]
//
// partition_wall_with_passthrough — COMPOUND, SPEC-DRIVEN macro that
// raises one rectangular partition wall across the workpiece top, then
// cuts a single rectangular passthrough opening through it.  The opening
// is sized from a spec library (common connector / cable cutouts).
//
//                              wall length L_wall
//          ◄──────────────────────────────────────────────►
//          ┌────────────────────────────────────────────────┐  ↑ wall_height
//          │              ┌────────┐                        │  │
//          │              │ cutout │                        │  │
//          │              │  Wc×Hc │ (positioned at cut_offset_x, cut_z) │
//          │              └────────┘                        │  │
//          └────────────────────────────────────────────────┘  ↓
//   ───────────────────────────────────────────────────────── entry_face
//          ◄── wall thickness T ─►   (depth into page)
//
// Spec library — `spec_class` selects the cutout size:
//
//   "cable_pass"      → 6 × 4 mm        (single cable bundle)
//   "connector_pass"  → 12 × 8 mm       (general connector)
//   "airflow_louver"  → 20 × 3 mm       (slim ventilation slot)
//   "usb_c_pass"      → 9 × 4 mm        (USB-C receptacle clearance)
//   "hdmi_pass"       → 16 × 7 mm       (full-size HDMI receptacle)
//
// Compound chain (subfeature_count = 2):
//   1. Build wall box (L × T × H) → fuse onto workpiece.
//   2. Build passthrough cutter box → cut through the wall.
//
// Signature pattern:
//   - kind = "partition_wall_with_passthrough"
//   - is_compound = true
//   - subfeature_count = 2
//   - spec_class, cutout_w_mm, cutout_h_mm
//   - wall_length, wall_thickness, wall_height
//   - cut_offset_x (along wall length), cut_z (above face)
//
// DFM:
//   DFM-PASS-UNKNOWN   : spec_class not in library.
//   DFM-WALL-MIN       : wall_thickness ≥ 0.4 mm.
//   DFM-PASS-FIT       : cutout_w < wall_length, cutout_h < wall_height.
//   DFM-PASS-STRAP     : residual material between cutout top and wall top
//                        ≥ 1 mm (else wall would collapse on press fit).
//   DFM-PASS-INPUT     : wall_length, wall_height > 0.
//
// Recognize:
//   Replay primary.  Geometric: a thin (in Y) extrusion above entry plane
//   with a single rectangular through-cut in its body.  Confidence 0.65/0.92.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace partition_wall_with_passthrough {

constexpr const char* kSkillId = "partition_wall_with_passthrough";

struct Input
{
    FaceDatum   entry_face;
    std::string spec_class;
    double      center_x_mm        = 0.0;        // wall midpoint X
    double      center_y_mm        = 0.0;        // wall midpoint Y
    double      wall_length_mm     = 0.0;        // along length_dir
    double      wall_thickness_mm  = 0.0;        // perpendicular in plane
    double      wall_height_mm     = 0.0;        // above entry face
    double      cut_offset_x_mm    = 0.0;        // cutout centre offset
                                                 // from wall mid along
                                                 // length_dir (default 0)
    double      cut_offset_z_mm    = 0.0;        // cutout centre Z above
                                                 // entry face — default
                                                 // resolved at apply()
                                                 // to (wall_height − cut_h)/2
    gp_Dir      length_dir         { 1.0, 0.0, 0.0 };
};

double specCutoutWFor(const std::string& spec_class);
double specCutoutHFor(const std::string& spec_class);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace partition_wall_with_passthrough
}  // namespace koocadcam::skill
