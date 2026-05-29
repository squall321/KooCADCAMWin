#pragma once
// @lat: [[engine/skills#banana_jack_receptacle]]
//
// banana_jack_receptacle — COMPOUND ENCLOSURE FEATURE that prepares a panel
// hole for a Ø4 mm banana-plug socket (IEC 61010 4 mm safety-banana).  Three
// primitive cuts fused into one signature, repeated `count` times in a line:
//
//   1. Through-hole       — Ø4.10 mm pilot through the panel face
//                           (4 mm nominal plug + 0.10 mm clearance fit).
//   2. Front counterbore  — Ø9.0 mm × 1.5 mm deep recess on the FRONT face
//                           (the plastic bezel of the jack sits flush here).
//   3. Rear snap-ring slot— Ø7.0 mm × 0.9 mm internal groove 6 mm in from the
//                           back face — receives the DIN 471-style retaining
//                           ring that locks the jack against the panel.
//
// Each repetition adds a fresh signature with index, count, and pitch.
//
//                    ┌── front (counterbore Ø9, 1.5 deep)
//                    ▼
//             ──────┬───────┬──────  panel front
//                   │  Ø9   │
//                   ├───────┤
//                   │  Ø4.1 │  ← through hole
//                   │       │
//             ──────┤  rear │──────  panel back
//                   │  Ø7   │  ← snap-ring slot (0.9 deep, 6 mm from back)
//                   └───────┘
//
// DFM checks:
//   DFM-INPUT          : count in [1, 16], spacing ≥ 12 mm
//   DFM-002            : through_dia (4.10) ≥ 0.8 mm (sanity)
//   DFM-BANANA-PANEL   : panel thickness must be ≥ 8 mm so the snap-ring slot
//                        sits clear of both the counterbore and back face.
//   DFM-DIN471         : snap-ring groove diameter (7.0) and depth (0.9) are
//                        from the DIN 471 internal-ring 7 mm table.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace banana_jack_receptacle {

constexpr const char* kSkillId = "banana_jack_receptacle";

struct Input
{
    FaceDatum   face;
    double      position_x_mm = 0.0;  // first jack centre (XY)
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    int         count         = 1;    // number of jacks in a row
    double      spacing_mm    = 19.05; // standard 0.75" banana pitch
    double      direction_deg = 0.0;  // row direction (CCW from +X)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace banana_jack_receptacle
}  // namespace koocadcam::skill
