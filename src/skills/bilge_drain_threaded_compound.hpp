#pragma once
// @lat: [[engine/skills#bilge_drain_threaded_compound]]
//
// bilge_drain_threaded_compound (slice 16) — REAL compound feature for a
// threaded marine bilge drain plug seat (BSPP G-thread).
//
// Sub-features (single output shape) — built SEQUENTIALLY:
//   1. drain through-bore (drain_dia)                              [pr::cut]
//   2. spotface counterbore at entry face                           [pr::cut]
//   3. cosmetic thread band (annular relief in the bore wall to
//      represent the BSPP/NPT G-thread; the actual helix is not modelled,
//      a slightly-wider annular cylinder of the thread minor dia depth
//      stands in for it)                                            [pr::cut]
//
// CRITICAL — slice-13/14/15 root cause: all three cuts share the same
// axis and overlapping radial range — must be SEQUENTIAL pr::cut, NOT
// pr::cutMany on a compound (OCCT 8.0 silent no-op).
//
// BSPP G-thread spec from central _hydraulic_ports.hpp (BspGSpec):
//   thread_major / thread_minor / pitch / drill / spotface / depth.
//
// DFM:
//   - dimensions positive.
//   - thread_size_key MUST exist in BspG central table.
//   - drain_dia > 0 and ≤ thread_minor (drain must fit inside thread).
//   - spotface_dia > thread_major (spotface accommodates plug head).
//   - spotface_depth ≥ 1 mm.
//
// Recognize:
//   - One axisymmetric small bore.
//   - One coaxial larger spotface counterbore step.
//   - One coaxial annular thread band slightly larger than bore.
//   Confidence 0.8 (geometric) / 1.0 (replay).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bilge_drain_threaded_compound {

constexpr const char* kSkillId = "bilge_drain_threaded_compound";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm        = 0.0;
    double      center_y_mm        = 0.0;
    gp_Dir      axis_dir           { 0.0, 0.0, 1.0 };
    std::string thread_size_key    = "G1/2";       // "G1/2" | "G3/4" | "G1"
    double      spotface_dia_mm    = 30.0;
    double      spotface_depth_mm  = 2.0;
    double      drain_dia_mm       = 14.0;         // < BspG minor
    double      plate_thickness_mm = 12.0;
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bilge_drain_threaded_compound
}  // namespace koocadcam::skill
