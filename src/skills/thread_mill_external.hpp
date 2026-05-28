#pragma once
// @lat: [[engine/skills#thread_mill_external]]
//
// thread_mill_external — cuts an EXTERNAL thread onto a cylindrical surface
// (screw / stud / threaded shaft).  Opposite of `thread_mill` with
// is_external=true in the sense that the input datum is an EXTERNAL
// cylindrical face: we identify that face, sweep a V-thread of radius
// equal to its radius, and SUBTRACT the helical V-tool from the workpiece
// (which leaves the thread valleys recessed into the cylinder surface).
//
//                    │   │
//                    │   │    ← external cylinder
//                    │ ╲ │
//                    │  ╲│        ← V-groove valley cut from the surface
//                    │ ╱ │
//                    │╱  │
//                    │   │
//                    │ ╲ │
//                    │  ╲│
//                    │ ╱ │
//                    │   │
//                  cyl_datum (FaceCylinderByAxis on the OUTER surface)
//
// Signature pattern:
//   - External cylindrical face whose surface has been carved by a helical
//     V-groove tool (the helical "ridge" of remaining material between
//     valleys is the thread itself).
//
// DFM checks:
//   DFM-EXT-MIN     : outer cylinder diameter ≥ M3 minor (2.5 mm).
//   DFM-EXT-TURNS   : thread_length / pitch ≥ 3 (need enough turns).
//   DFM-INPUT       : pitch_mm > 0, thread_length_mm > 0.
//
// Recognize:
//   Topologically an external thread is hard to detect from B-spline /
//   swept-helix faces alone, especially after STEP I/O which discards the
//   helix parametrization.  Recognition is therefore metadata-driven for
//   slice 1: we replay signatures stored in `wp.features()`.  A future
//   slice can add a geometric check looking for an external cylindrical
//   face whose surface area exceeds a plain cylinder of the same radius
//   by ~ pitch × thread_length × turns × surface_area_per_turn.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace thread_mill_external {

constexpr const char* kSkillId = "thread_mill_external";

struct Input
{
    FaceDatum   cylinder_datum;            // EXTERNAL cylindrical face to thread
    std::string thread_size      = "M6";
    double      pitch_mm         = 1.0;    // metric pitch
    double      thread_length_mm = 0.0;    // axial length of threaded portion
    double      start_z_mm       = 0.0;    // optional axial start (world z)
    double      end_z_mm         = 0.0;    // optional axial end   (world z)
    double      tool_dia_mm      = 1.5;    // thread-mill diameter
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace thread_mill_external
}  // namespace koocadcam::skill
