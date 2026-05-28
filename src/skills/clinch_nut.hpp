#pragma once
// @lat: [[engine/skills#clinch_nut]]
//
// clinch_nut — a self-clinching threaded nut pressed into a sheet-metal
// hole so the sheet's plastically-deformed material retains the nut.
// Common brands: PEM® / TR Fastenings.
//
//                     ▼ press direction (axis)
//                    ┌─────┐                      ▲
//                    │ ⌒ ⌒ │  ← knurled barrel    │ sheet_thickness_mm
//                    │     │                      │
//          ──────────┴─────┴──────────            ▼
//                       ↑
//                  clinch through hole
//                  (dia ≈ thread_dia + 0.5 mm)
//
// Slice-1 geometry: a clean cylindrical through-hole, sized so the nut's
// knurled barrel can be pressed in.  The real process plastically deforms
// the sheet to retain the nut; that displacement is RECORDED in metadata
// and not modelled in B-rep.
//
// Barrel-hole sizing convention (per PEM CL/CLS catalog ~values):
//   barrel_dia = thread_dia + 0.5 mm
//
// Signature pattern:
//   - 1 cylindrical face spanning the sheet thickness
//   - 2 circular edges (top + bottom)
//   - NO bottom planar face (always through, by definition)
//   - pattern.kind == "clinch_nut"
//   - pattern.thread_size, pattern.sheet_thickness_mm, pattern.barrel_dia_mm
//
// DFM checks:
//   DFM-CN-SIZE      : thread_size in supported set (M3/M4/M5/M6/M8)
//   DFM-CN-SHEET     : sheet_thickness_mm ∈ [0.5, 3.0] mm
//   DFM-CN-MISMATCH  : sheet_thickness disagrees with stock min-extent
//
// Recognize:
//   History replay first.  Topology fallback flags a through cylindrical
//   hole in thin stock (≤ 3 mm) whose diameter matches a clinch-nut
//   barrel spec (± 0.1 mm).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace clinch_nut {

constexpr const char* kSkillId = "clinch_nut";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm       = 0.0;
    double      position_y_mm       = 0.0;
    gp_Dir      axis_dir            { 0.0, 0.0, -1.0 };
    std::string thread_size;                       // "M3" | "M4" | "M5" | "M6" | "M8"
    // If 0, sheet thickness is sensed from the workpiece's smallest bbox extent.
    double      sheet_thickness_mm  = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Helpers exposed for tests.
double clinchBarrelDiameter(const std::string& thread_size);       // 0.0 if unknown
std::string threadSizeForBarrel(double barrel_dia_mm, double tol_mm = 0.10);

// Sense sheet thickness from the workpiece: smallest bbox extent.
double senseSheetThickness(const Workpiece& wp);

}  // namespace clinch_nut
}  // namespace koocadcam::skill
