#pragma once
// @lat: [[engine/skills#swarf_milling]]
//
// swarf_milling — 5-axis side-cut where the END-MILL'S SIDE WALL (not its
// bottom) cuts a long swept surface as the tool tilts along a guide.
//
// In real swarf-milling, the tool follows a 3D toolpath where:
//   - The TOOL POSITION sweeps along a guide curve (waypoint polyline)
//   - The TOOL AXIS tilts continuously to keep the tool's side wall
//     tangent to a target ruled surface (the "guide surface")
//
// This is genuinely 5-axis: simultaneous translation along the guide
// polyline AND rotation of the tool axis (typically about a B-axis or
// A-axis) at every CL point.  3-axis CAM cannot produce this geometry —
// the tool axis must change orientation during the cut.
//
//                            ▲ workpiece
//                            │
//          guide_polyline:   │
//             P0─────P1──────┴─────P2─────P3
//                     ╲              ╲          ← tool tilted at each Pi
//                      ╲              ╲              by tilt_angle_deg
//                       ╲              ╲
//                        ▼ tool side    ▼ tool side
//                          (cylindrical)  (cylindrical)
//
// ─── APPROXIMATION NOTE (slice 1) ───────────────────────────────────────
//
// True swarf-milling needs a continuous 5-axis toolpath generator (CL
// point interpolation, ruled-surface tangent calculation, machine-
// kinematics post).  That is out of scope here.
//
// We APPROXIMATE the swept cut by:
//   1. Building one cylinder PER waypoint of `guide_polyline`.
//   2. Each cylinder is tilted by `tilt_angle_deg` away from vertical,
//      around the polyline-tangent axis (so the tool leans "outward").
//   3. The cylinders are fused into one compound and cut from the
//      workpiece in a single Boolean op (per the OCCT-8 cookbook —
//      overlapping cylinders + compound cut performs better and avoids
//      degenerate Boolean histories than N sequential cuts).
//
// This gives a faceted approximation of the swarf cut — sufficient for
// the FeatureSignature pattern recording and downstream CAPP, but NOT
// machine-ready toolpath geometry.
//
// ────────────────────────────────────────────────────────────────────────
//
// Signature pattern:
//   - A chain of cylindrical faces along the 3D guide polyline.
//   - Cylinder axes are NOT parallel to any global axis (they're tilted).
//   - No flat bottom (the cut sweeps laterally through material).
//   - is_5axis_cut = true.
//
// DFM checks:
//   DFM-002 : tool_dia_mm ≥ 1.5 mm  (smallest practical 5-axis end-mill)
//   DFM-005 info — "swarf_milling requires 5-axis machine kinematics"
//   tilt_angle_deg ∈ [0, 60]
//   guide_polyline.size() ≥ 2
//
// Recognize:
//   Recognition by topology alone is HARD — a chain of overlapping cylinders
//   from above looks like a string of drill_holes.  Confidence ~ 0.4.
//   We claim a candidate when ≥ 2 cylindrical faces exist whose axes are
//   tilted (NOT parallel to any global axis) and whose centers form a
//   roughly linear sequence.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace swarf_milling {

constexpr const char* kSkillId = "swarf_milling";

struct Input
{
    FaceDatum                              entry_face_datum;
    // Guide polyline: list of 3D (x, y, z) points — the tool centerline
    // sweeps through these.
    std::vector<std::array<double, 3>>     guide_polyline;
    double                                 tool_dia_mm   = 0.0;
    double                                 tool_length_mm = 0.0;
    double                                 tilt_angle_deg = 0.0;   // [0, 60]
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace swarf_milling
}  // namespace koocadcam::skill
