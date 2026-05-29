#pragma once
// @lat: [[engine/skills#face_seal_compound_compression]]
//
// face_seal_compound_compression — COMPOUND MACRO: face-seal O-ring groove
// PLUS a concentric compensating "energiser" spring groove that increases
// the effective compression of the primary seal under thermal cycling.
//
// Geometry (cross-section, axial direction = into the page):
//
//   ───┬────────────────────────┬───    ← face plane
//      │   primary seal groove  │
//      │  ┌──────────────────┐  │       depth = 0.75 × CS
//      │  │                  │  │       width = 1.40 × CS
//      │  └──────────────────┘  │
//      │ spring groove (deeper) │       depth = 0.75 × CS + spring_extra
//      │     ┌──────────┐       │       width = 0.8 × CS (narrower)
//      │     └──────────┘       │
//   ───┴────────────────────────┴───
//
// Sub-feature chain (3 cuts, fused → cut once):
//   1. PRIMARY SEAL GROOVE — annular ring, depth G = 0.75 × CS, width W =
//      1.40 × CS, at primary_radius_mm.
//   2. SPRING ENERGISER GROOVE — concentric annular ring at
//      spring_radius_mm, depth = G + spring_extra_depth_mm (deeper) and
//      width = 0.8 × CS (narrower), holds the canted-coil spring per
//      Bal Seal Engineering Catalog §"Spring-Energized Seal Design".
//   3. ID 15° lead-in cone on the primary seal — easy O-ring assembly.
//
// Signature pattern:
//   kind = "face_seal_compound_compression"
//   is_compound = true, subfeature_count = 3
//   spec_key = AS568 dash size for the primary O-ring
//
// DFM gates:
//   - dash_size must be in spec table.
//   - primary_radius_mm and spring_radius_mm must differ by ≥ W (so the
//     two grooves don't overlap).
//   - spring_extra_depth_mm > 0 (else it's not an energiser).
//   - both grooves must fit within face_dia_mm bbox heuristic.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace face_seal_compound_compression {

constexpr const char* kSkillId = "face_seal_compound_compression";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm           = 0.0;
    double      center_y_mm           = 0.0;
    gp_Dir      axis_dir              { 0.0, 0.0, -1.0 };
    double      primary_radius_mm     = 0.0;     // centerline of primary seal groove
    double      spring_radius_mm      = 0.0;     // centerline of energiser groove
    double      spring_extra_depth_mm = 0.5;     // how much deeper than primary
    std::string dash_size;                       // AS568 primary seal CS key
};

double crossSectionFor(const std::string& dash_size);
double primaryDepthFor (double cs_mm);   // 0.75 × cs
double primaryWidthFor (double cs_mm);   // 1.40 × cs
double springWidthFor  (double cs_mm);   // 0.80 × cs

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace face_seal_compound_compression
}  // namespace koocadcam::skill
