#pragma once
// @lat: [[engine/skills#connector_cutout_with_keepout]]
//
// connector_cutout_with_keepout — Smart-electronics compound skill that
// punches a through-pocket for a named connector type (D-sub-9, USB-A,
// USB-C, RJ45, HDMI) and emits the EMI / fastener keep-out region as
// metadata so downstream CAPP knows where NOT to route traces or place
// other features.
//
// The skill is SPEC-TABLE DRIVEN.  Each entry in the lookup table
// (embedded in the .cpp) carries:
//   - body_w_mm / body_h_mm          → rectangular through-pocket size
//   - corner_radius_mm               → rounded-rectangle corner radius
//   - retention_notch_count          → number of small side notches for
//                                      the connector's metal retention tabs
//   - retention_notch_w_mm / d_mm    → notch geometry
//   - chamfer_mm                     → 45° lead-in chamfer around the
//                                      entry edge of the pocket
//   - keepout_w_mm / h_mm            → EMI clearance rectangle (metadata)
//
// Sub-cuts per cutout:
//   1. THROUGH POCKET   — rounded-rectangle through hole
//   2. CHAMFER          — entry-edge bevel (frustum subtraction)
//   3. RETENTION NOTCHES — N rectangular pockets on long sides
//
// CONTEXT-AWARE: chooses the entry face based on the FaceDatum, then
// computes the pocket axis (into material) from the face normal.  The
// EMI keepout box is stored as JSON in the signature's pattern.
//
// DFM:
//   USB-IF, HDMI Licensing, IEC 60603 — body dimensions per spec
//   IPC-2221                          — EMI keepout ≥ 1 mm on all sides
//   DFM-CONN-SIZE                     — unknown connector_type rejected
//   DFM-PLATE-THICKNESS               — face plate must be ≥ chamfer + 0.5

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>

namespace koocadcam::skill {

class Workpiece;

namespace connector_cutout_with_keepout {

constexpr const char* kSkillId = "connector_cutout_with_keepout";

struct Input
{
    FaceDatum    face;                            // entry face (panel)
    gp_Dir       axis_dir            { 0.0, 0.0, -1.0 };  // into-material direction
    std::string  connector_type      = "USB-A";   // "D-sub-9"/"USB-A"/"USB-C"/"RJ45"/"HDMI"
    double       position_x_mm       = 0.0;
    double       position_y_mm       = 0.0;
    double       chamfer_override_mm = -1.0;      // -1 → use spec default
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace connector_cutout_with_keepout
}  // namespace koocadcam::skill
