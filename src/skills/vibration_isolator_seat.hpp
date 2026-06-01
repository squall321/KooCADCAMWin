#pragma once
// @lat: [[engine/skills#vibration_isolator_seat]]
//
// vibration_isolator_seat — COMPOUND MECHANICAL FEATURE for a rubber-bushed
// stud-mount vibration isolator seat (engine mounts, fan bushings, panel
// dampers in aerospace/automotive panels).
//
//                    ┌─ entry_face (planar, top)
//                    ▼
//              ──────┬─────────────────────┬──────
//                    │bushing_od           │      ↑ bushing_seat_depth
//                    │ ◄─────────────────► │      │
//                    │                     │      ↓
//                    │     ┌───┐           │      ↑
//                    │     │   │  cb_dia   │      │ cb_depth (counterbore for nut)
//                    │     │   │           │      ↓
//                    │     │   │           │      ↑
//                    │  ┌──┴───┴──┐ relief │      │ lockwasher_relief (small annular)
//                    │  │         │        │      ↓
//                    │  │ ─►stud_dia        │     ↑
//                    │  │   pilot through  │     │ plate-through hole
//                    │  └──────────────────┘     ↓
//                    └─────────────────────┘
//
// Sub-features (4):
//   1. Through hole at pilot diameter (= stud tap drill for stud_M, e.g.
//      M6 → 6.5 mm clearance) — goes all the way through the plate.
//   2. Counterbore for the stud-side nut (cb_dia = stud_M × 1.8, cb_depth =
//      stud_M × 0.6, per SAE J429 socket nut head sizes).
//   3. Recessed seat for the rubber bushing at top (bushing_od × bushing_od×0.4
//      depth, per Lord Corp / Trelleborg standard ranges).
//   4. Small lockwasher relief annular groove just above the counterbore
//      (relief_od = cb_dia + 1.5 mm, depth = 0.6 mm) — keeps the lockwasher
//      OD from interfering with the counterbore wall.
//
// All four are CONCENTRIC cylinders / annuli around the same axis.  The
// fuse_first_then_cut pattern from counterbore.cpp is used because the
// concentric cylinders have overlapping volumes.
//
// Input:
//   - face_id   : entry face (top of plate)
//   - position  : axis center XY
//   - stud_M    : stud size string ("M4" … "M12") — drives pilot, nut, etc.
//   - bushing_od: outer diameter of rubber bushing in mm
//
// DFM:
//   DFM-002      pilot_dia ≥ 0.8 mm
//   DFM-STUD     unknown stud_M
//   DFM-BUSHING  bushing_od < pilot_dia + 2 mm (no annular shoulder)
//   DFM-PLATE-T  plate thickness < total seat depth (would cut through seat)
//
// Pattern: kind=vibration_isolator_seat, is_compound=true,
// subfeature_count=4 + verbatim inputs + derived pilot_dia, cb_dia,
// cb_depth, relief_dia, relief_depth, bushing_depth.
//
// Recognize: METADATA REPLAY ONLY (compound geometry is hard to fingerprint —
// 3 concentric different-radius cylinders + an annular slot is not a unique
// signature in practice).  We scan FeatureSignature history for
// kind=="vibration_isolator_seat" and return confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace vibration_isolator_seat {

constexpr const char* kSkillId = "vibration_isolator_seat";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir { 0.0, 0.0, -1.0 };

    std::string stud_M;                // "M4" … "M12"
    double      bushing_od_mm  = 0.0;  // rubber bushing OD
    double      plate_thick_mm = 0.0;  // workpiece plate thickness
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// ── Geometry-formula helpers (single source of truth for tests + apply) ──
// Returns 0.0 if stud_M is unknown.
double bushingSeatDepth(double bushing_od_mm);
double reliefDepth();
double cbDepth(const std::string& stud_M);
// Sum of the 3 stacked seat features (bushing seat + relief + counterbore).
// Must stay STRICTLY below plate_thick_mm minus 1 mm clearance for DFM pass.
double totalSeatDepth(const std::string& stud_M, double bushing_od_mm);

}  // namespace vibration_isolator_seat
}  // namespace koocadcam::skill
