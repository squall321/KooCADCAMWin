#pragma once
// @lat: [[engine/skills#ultrasonic_weld]]
//
// ultrasonic_weld — Ultrasonic Welding (USW).  A high-frequency vibrating
// horn (~20 kHz, several µm amplitude) is pressed against thin sheets, and
// solid-state bonding occurs at the contact interface in milliseconds.
// No bulk melting, no filler.  Industrial uses: battery foil tabs, copper
// wire harnesses, aluminum tube bonding, plastic packaging.
//
//   Side view:
//                       ▼  horn
//                     ┌────┐  ← spot_dia_mm (2 – 8 mm typ.)
//                     └────┘
//   ─────────────────┴──────┴─────────────  ← thin sheet (≤ 3 mm)
//                     ░░░░░░  ← shallow disc-shape impression
//                     (height 0.1 – 0.3 mm)
//
// Geometrically: a SMALL disc fused onto the sheet surface — analogous to
// spot_weld but with a smaller, shallower bead because there is no melt
// pool, just a localised plastic deformation knurl.
//
// Signature pattern:
//   - 1 cylindrical face (disc side wall)
//   - 2 circular edges
//   - 1 small planar disc on top
//   - pattern.kind = "ultrasonic_weld"
//   - small height ≤ 0.3 mm
//
// DFM checks:
//   spot_dia_mm  ∈ [1, 12]
//   bead_height  ≤ 0.5     (USW is shallow by definition)
//   sheet_thickness_mm ≤ 3 (USW is for THIN parts — info if missing)
//
// Recognize:
//   Geometric: small additive disc with very small h (< 0.5 mm).
//   Falls back to metadata if available.  Distinguishes from spot_weld by
//   the MUCH smaller height range (0.1 – 0.3 vs 0.3 – 1.5 mm).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ultrasonic_weld {

constexpr const char* kSkillId = "ultrasonic_weld";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm           = 0.0;
    double      position_y_mm           = 0.0;
    double      spot_dia_mm             = 0.0;   // 2 – 8 mm typ.
    double      bead_height_mm          = 0.2;   // 0.1 – 0.3 mm
    std::string material                = "aluminum";  // typically Al, Cu, plastic
    double      vibration_amplitude_um  = 20.0;  // 5 – 50 µm typ.
    double      frequency_khz           = 20.0;  // 15 / 20 / 40 kHz machines
    double      sheet_thickness_mm      = 1.0;   // INFO check, ≤ 3 mm
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ultrasonic_weld
}  // namespace koocadcam::skill
