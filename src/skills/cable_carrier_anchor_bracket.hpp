#pragma once
// @lat: [[engine/skills#cable_carrier_anchor_bracket]]
//
// cable_carrier_anchor_bracket — Energy-chain (drag-chain) END ANCHOR bracket
// (slice 16, robotics / automation).
//
// The fixed-end anchor bracket that ties an energy chain (cable carrier /
// drag chain) to the machine frame.  It carries two elongated bolt slots
// (for fore/aft adjustment of the chain end) and a single chain-link pin
// bore that captures the chain's end-connector pin.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. bolt slot A  (one slot tool = box fused with 2 end cylinders)
//   2. bolt slot B  (one slot tool = box fused with 2 end cylinders)
//   3. chain-link pin bore (cylinder cut)
//
// Each slot tool is built by FUSING its box and two end-cylinders into a
// single watertight tool, THEN cut sequentially — never a compound of
// overlapping cutters.
//
// subfeature_count = 2 + 1.   (2 slots + 1 pin bore)
//
// DFM:
//   DFM-INPUT          : every dimension must be > 0.
//   DFM-ROBOTICS-SLOT  : bolt_slot_len_mm must exceed bolt_slot_wid_mm
//                        (an elongated slot, not a round hole).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cable_carrier_anchor_bracket {

constexpr const char* kSkillId = "cable_carrier_anchor_bracket";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy            { 0.0, 0.0, 0.0 };
    double    chain_width_mm        = 30.0;
    double    bolt_slot_len_mm      = 14.0;
    double    bolt_slot_wid_mm      = 6.5;
    double    bolt_slot_spacing_mm  = 40.0;   // center-to-center of the 2 slots
    double    pin_bore_dia_mm       = 8.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cable_carrier_anchor_bracket
}  // namespace koocadcam::skill
