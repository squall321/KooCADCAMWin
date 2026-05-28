#pragma once
// @lat: [[engine/skills#white_light_scan]]
//
// white_light_scan — INSPECTION skill: NON-CONTACT white-light
// interferometry surface scan.  Projects structured / coherent white
// light onto the target face and reconstructs a dense Z-height map by
// fringe analysis.  Used for sub-micron form / texture / step-height
// measurement of optically-cooperative surfaces (polished, lapped,
// freshly machined).  Geometrically a NO-OP.
//
//      light source ──▶ │ reference  │
//                       │ mirror     │      ┌────────┐
//                       └──── beam ──┴──▶  │ face    │  → fringe stack
//                       ◀── recombined ──── │ (target)│  → height map
//                                          └────────┘
//
// What we record:
//   - the target face datum (the surface scanned)
//   - the scan area in mm² (computed from face)
//   - the point density (points / mm²) — drives memory + scan time
//   - the measured RMS deviation (nm-class) of the surface form vs.
//     nominal — typical white-light interferometers report root-mean-
//     square form error in nm
//
// Input:
//   target_face_datum    — the face to scan
//   point_density_per_mm2— planar point density (default 100 pt/mm²)
//   measured_rms_nm      — measured RMS form deviation (nm), 0 ⇒ not yet measured
//   rms_tolerance_nm     — acceptance band (e.g. 50 nm for a precision optic)
//
// DFM checks:
//   target_face_datum resolves
//   face area > 0 (otherwise scan has no data)
//   point_density > 0
//   rms_tolerance_nm > 0
//   measured_rms_nm > rms_tolerance_nm ⇒ warning (out-of-spec form)
//
// Recognize:
//   Metadata-only — empty unless replayed.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace white_light_scan {

constexpr const char* kSkillId = "white_light_scan";

struct Input
{
    FaceDatum   target_face_datum;
    double      point_density_per_mm2 = 100.0;
    double      measured_rms_nm       = 0.0;     // 0 ⇒ planning, no measurement yet
    double      rms_tolerance_nm      = 50.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace white_light_scan
}  // namespace koocadcam::skill
