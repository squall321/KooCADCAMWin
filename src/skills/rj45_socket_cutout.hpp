#pragma once
// @lat: [[engine/skills#rj45_socket_cutout]]
//
// rj45_socket_cutout — COMPOUND ENCLOSURE FEATURE that machines a standard
// 8P8C (RJ-45) PCB-mount jack window into a panel.  Three primitive cuts
// fused into one signature, repeated `port_count` times:
//
//   1. Main through-pocket — 13.0 × 14.5 mm rounded-rect through-hole (the
//                            jack body slides through).
//   2. Locking-tab relief  — small 3.0 × 1.0 × 2.0 mm slot centred on the
//                            BOTTOM edge of the main pocket (clearance for
//                            the latch tab so the plug can fully seat).
//   3. Edge chamfer        — 0.5 × 45° chamfer on the front edge of the
//                            main pocket (de-burr + ergonomic insertion).
//
// All cuts share the same axis_dir (into the panel face).
//
//        ┌───────────────────┐
//        │                   │  ← main pocket 13 × 14.5
//        │      port body    │
//        │                   │
//        └──────┐    ┌───────┘
//               │tab │  ← locking-tab relief (3 × 1)
//               └────┘
//
// DFM checks:
//   DFM-INPUT          : port_count in [1, 24], spacing ≥ 14 mm
//   DFM-RJ45-PANEL     : panel thickness ≤ 5 mm (jack body has only 5 mm of
//                        thread engagement above the chamfer)
//   DFM-RJ45-PITCH     : port spacing ≥ 14 mm (TIA/EIA recommended)
//   DFM-002            : chamfer ≥ 0.3 mm (smaller is not machinable)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rj45_socket_cutout {

constexpr const char* kSkillId = "rj45_socket_cutout";

struct Input
{
    FaceDatum   face;
    double      position_x_mm = 0.0;   // first port centre (XY)
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    int         port_count    = 1;
    double      spacing_mm    = 16.0;  // standard PCB-mount jack pitch
    double      direction_deg = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rj45_socket_cutout
}  // namespace koocadcam::skill
