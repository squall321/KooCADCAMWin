#pragma once
// @lat: [[engine/skills#usb_c_port_cutout]]
//
// usb_c_port_cutout — COMPOUND ENCLOSURE FEATURE that machines a USB Type-C
// receptacle panel cut.  Per USB-IF SP-30, the receptacle outer shell is
// 8.94 × 2.56 mm with R1.28 rounded ends.  Four primitive cuts fused into
// one signature:
//
//   1. Main through-pocket   — 8.94 × 2.56 mm rounded-rect through-cut for
//                              the receptacle shell.
//   2. EMI gasket lip groove — a 0.5 mm wide × 0.4 mm deep groove around the
//                              pocket perimeter on the OUTSIDE face for the
//                              EMI gasket lip (SP-30 §3.10).
//   3. Retention notch L     — small 0.8 × 0.5 mm notch on the left rounded
//                              end for the receptacle's PCB-mount tab.
//   4. Retention notch R     — mirror of #3 on the right end.
//
//          ┌────── EMI groove (#2) ──────┐
//          │  ┌─────────────────────────┐│
//          │  │     port (#1)           ││  2.56
//   notch L│←─│                         │→│notch R
//          │  └─────────────────────────┘│
//          └─────────────────────────────┘
//                     8.94
//
// DFM checks:
//   DFM-INPUT          : panel thickness in [0.6, 3.0] mm (USB-IF SP-30)
//   DFM-USBC-DIM       : pocket must be exactly 8.94 × 2.56 mm
//                        (deviation > 0.10 mm fails Type-C compliance)
//   DFM-002            : retention notch ≥ 0.4 mm
//   DFM-USBC-EMI       : EMI lip groove depth (0.4) > 0.3 × panel thickness
//                        — keeps the gasket lip from punching through.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace usb_c_port_cutout {

constexpr const char* kSkillId = "usb_c_port_cutout";

struct Input
{
    FaceDatum   face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    double      orientation_deg = 0.0;  // long-axis rotation (CCW from +X)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace usb_c_port_cutout
}  // namespace koocadcam::skill
