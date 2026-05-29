#pragma once
// @lat: [[engine/skills#threaded_npt_port]]
//
// threaded_npt_port — COMPOUND MACRO that machines an ANSI/ASME B1.20.1
// NPT (National Pipe Thread Tapered) port: a tapered tap hole at the
// ISO/ANSI standard 1°47'/foot (1.7899°) angle, with a counterbore
// recess at the mouth for clearance, and a 45° face chamfer to deburr
// the entry.
//
//                          ▼ face_id
//                  ────────┬───────────┬──────── face plane
//                          │ 45° chamfer (sub-cut #3)
//                          \_________/
//                          │ counterbore (sub-cut #2)
//                          │←─ d_seat ─→│
//                          ├───┬───┬───┤
//                          ╲   │   │   ╱   NPT taper 1.7899°/side
//                           ╲  │   │  ╱    (sub-cut #1, dominant)
//                            ╲ │   │ ╱
//                             ╲│   │╱
//                              ╲___╱  ← thin end (smaller dia)
//
// Standard NPT sizes (B1.20.1):
//   1/8  : nom OD = 10.29, threads/inch = 27, L1 = 4.10 mm
//   1/4  : nom OD = 13.72, threads/inch = 18, L1 = 5.79 mm
//   3/8  : nom OD = 17.15, threads/inch = 18, L1 = 6.10 mm
//   1/2  : nom OD = 21.34, threads/inch = 14, L1 = 8.13 mm
//   3/4  : nom OD = 26.67, threads/inch = 14, L1 = 8.61 mm
//   1    : nom OD = 33.40, threads/inch = 11.5, L1 = 10.16 mm
// (L1 = effective hand-tight thread engagement length per ASME B1.20.1)
//
// Sub-feature chain (3 primitive cuts):
//   1. NPT tapered hole — modelled with a cone frustum (1.7899°/side).
//      The cone diameter at the face is the nominal OD; at L1 depth it
//      tapers IN to (nominalOD − 2 × tan(1.7899°) × L1).  This is a
//      synthesis approximation of the helical NPT thread — the tapered
//      pilot region is what CAPP needs to schedule the tap operation.
//   2. Counterbore — straight cylinder at the mouth, diameter slightly
//      larger than nominalOD, depth = recess_depth_mm (default 1.5 mm)
//      to provide tool relief for the NPT tap entry.
//   3. Face chamfer — 45° conical chamfer around the mouth, leg =
//      chamfer_leg_mm.  Cosmetic + deburr.
//
// DFM gates:
//   DFM-INPUT          : recess_depth_mm <= 0 or chamfer_leg_mm < 0.
//   DFM-NPT-SIZE       : npt_size unknown.
//   DFM-NPT-WALL       : wall thickness around port < 2 × pitch (port
//                        would crack at threads under pressure).
//   DFM-NPT-DEPTH      : L1 thread depth exceeds part thickness datum.
//
// Recognize: metadata replay (NPT thread + counterbore is unique to the
// tooling history, not the recovered B-rep topology).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace threaded_npt_port {

constexpr const char* kSkillId = "threaded_npt_port";

struct Input
{
    FaceDatum   face_id;                                     // host planar face
    double      position_x_mm    = 0.0;
    double      position_y_mm    = 0.0;
    gp_Dir      axis_dir         { 0.0, 0.0, -1.0 };         // into the material
    std::string npt_size;                                    // "1/8" | "1/4" | "3/8" | "1/2" | "3/4" | "1"
    double      recess_depth_mm  = 1.5;                      // counterbore depth
    double      chamfer_leg_mm   = 0.5;                      // face chamfer leg
    double      part_thickness_mm = 20.0;                    // for DFM depth gate
};

// Resolve NPT size → (nominal OD mm, threads_per_inch, L1 effective length mm).
// Returns true on success; on failure all out-params untouched, return false.
bool resolveNptSize(const std::string& size,
                    double& nominal_od_mm,
                    int&    threads_per_inch,
                    double& l1_depth_mm);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace threaded_npt_port
}  // namespace koocadcam::skill
