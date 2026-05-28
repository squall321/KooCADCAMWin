#pragma once
// @lat: [[engine/skills#threaded_insert]]
//
// threaded_insert — a helical-coil or press-in insert installed inside a
// tapped (or close-fit) hole so that softer parent material (aluminium /
// magnesium / plastic) gains a steel-grade thread.
//
//                          ┌─ entry_face
//                          ▼
//                    ──────┬──────────┬──────
//                          │ insert   │
//                          │ pilot    │       ↑ insert_length_mm
//                          │ ø        │       │  (≥ 1.5 × thread dia)
//                          │   ⨯      │       │
//                          │          │       ↓
//                          └──────────┘
//                          ↑
//                  insert_pilot_dia
//                  (= drill_and_tap pilot + small allowance)
//
// In slice-1 the geometry is just a cylindrical hole sized for the insert's
// own pilot.  The insert (helical coil or solid press-in) is recorded as
// METADATA on the signature; CAPP can later schedule the install operation.
//
// Pilot-diameter convention:
//   insert_pilot_dia = drill_and_tap_pilot(thread_size) + 0.2 mm allowance
// to account for the fact that a press-in insert's body is wider than the
// nominal tap drill.
//
// Signature pattern:
//   - 1 cylindrical face + 2 circular edges (same as drill_hole)
//   - 1 planar bottom face if not through
//   - pattern.kind == "threaded_insert"
//   - pattern.thread_size, pattern.insert_length_mm, pattern.material carry
//     the install spec.
//
// DFM checks:
//   DFM-TI-SIZE   : thread_size in supported set (M3/M4/M5/M6/M8)
//   DFM-TI-LEN    : insert_length_mm ≥ 1.5 × thread dia
//   DFM-INPUT     : insert_length_mm > 0; material non-empty
//
// Recognize:
//   History replay first.  Topology fallback flags cylindrical bores whose
//   diameter matches one of the standard insert pilots (± 0.1 mm).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace threaded_insert {

constexpr const char* kSkillId = "threaded_insert";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm     = 0.0;
    double      position_y_mm     = 0.0;
    gp_Dir      axis_dir          { 0.0, 0.0, -1.0 };
    std::string thread_size;                       // "M3" | "M4" | "M5" | "M6" | "M8"
    double      insert_length_mm  = 0.0;
    std::string insert_material   = "stainless_316";
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// thread_size → pilot diameter (mm) for THIS insert.  Internally reuses
// drill_and_tap's tap-drill chart and adds the insert allowance.  0.0 if
// unknown.
double insertPilotDiameter(const std::string& thread_size);

// Reverse-resolve pilot diameter → thread size (or "" if no match).
std::string threadSizeForInsertPilot(double pilot_dia_mm, double tol_mm = 0.10);

// Nominal thread diameter from "M3" → 3.0.  0.0 if unknown.
double threadNominalDia(const std::string& thread_size);

}  // namespace threaded_insert
}  // namespace koocadcam::skill
