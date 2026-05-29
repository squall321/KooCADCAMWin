#pragma once
// @lat: [[engine/skills#sawmill_cut]]
//
// sawmill_cut — bulk lumber cut for a wooden workpiece (rip cut along the
// grain, cross-cut across the grain, or resaw splitting a board edge-to-edge
// into thinner stock).  Used at the *log → board* stage of woodworking.
//
//   rip cut (along grain — Y axis preserved)
//
//            ╱──────────────────╲
//          ╱   ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒  ╲          ─ rip ─►   ┌──────────┐
//        ╱    ▒  grain dir →   ▒  ╲                    │  rough   │
//        ╲    ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒  ╱                    │  board   │
//          ╲                    ╱                      └──────────┘
//            ╲──────────────────╱
//                  log
//
// Synthesis strategy (slice-1):
//   Rather than perform a Boolean cut (which would leave a kerf void), we
//   *rebuild* the workpiece as a smaller cuboid representing the resulting
//   board.  This mirrors the `rolling` rebuild pattern and is appropriate
//   because the "removed" sawdust is not part of any downstream feature.
//
//   The Input describes the target board dimensions along the cut axis,
//   subject to:
//     - cut_type = "rip"        — preserve Y/Z, shrink X
//     - cut_type = "cross_cut"  — preserve X/Z, shrink Y
//     - cut_type = "resaw"      — preserve X/Y, shrink Z (split into thinner
//                                 boards; result is the LARGER half)
//
// Signature pattern:
//   - 6 planar faces (cuboid board)
//   - cut_type, kerf_mm, cut_thickness_mm, original_xyz
//
// DFM checks:
//   DFM-SAWMILL-KERF        kerf_mm not in [2.5, 8.0] (industrial bandsaw + circular saw band)
//   DFM-SAWMILL-MIN         cut_thickness_mm < 5 mm (board too thin to mill)
//   DFM-SAWMILL-GROWTH      cut_thickness_mm >= current extent on the cut axis
//                           (sawmill REMOVES material)
//   DFM-INPUT               cut_thickness_mm <= 0; cut_type unknown
//
// Recognize:
//   Geometric recognition cannot distinguish a sawn board from any other
//   cuboid stock; we report a low-confidence candidate based on bbox
//   alone, and a high-confidence replay from feature metadata when present.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sawmill_cut {

constexpr const char* kSkillId = "sawmill_cut";

struct Input
{
    FaceDatum    entry_face;                       // reference face (top of log)
    std::string  cut_type           = "rip";       // "rip" | "cross_cut" | "resaw"
    double       cut_thickness_mm   = 0.0;         // result extent along cut axis
    double       kerf_mm            = 4.0;         // sawblade kerf width (mm)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sawmill_cut
}  // namespace koocadcam::skill
