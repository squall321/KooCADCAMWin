#pragma once
// @lat: [[engine/skills#speaker_basket_spider_seat]]
//
// speaker_basket_spider_seat — Loudspeaker driver basket: spider mounting
// seat (slice 16, pro audio hardware).
//
// A dynamic-driver basket (frame) carries the motor and suspension.  At the
// neck of the basket the spider (damper) is glued to a machined ledge that
// surrounds the voice-coil former.  This skill machines that interface from
// a flat top face:
//   1. spider mounting ledge — a shallow counterbore (wide, shallow)
//   2. voice-coil clearance bore — a narrow through bore for the VC former
//   3. N vent holes around the ledge — release trapped air under the spider
//      (cooling / pneumatic damping), patterned by SetRotation.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. spider ledge counterbore  (cylinder cut DOWN from top face)
//   2. voice-coil clearance bore (narrow through cylinder)
//   3..(2+N). N vent holes on a bolt-circle (gp_Trsf::SetRotation)
//
// subfeature_count = 2 + vent_count.
//
// DFM:
//   DFM-INPUT     : every dimension / count must be > 0.
//   DFM-VC-BORE   : vc_clearance_dia must be < spider_ledge_dia.
//   DFM-VENT-FIT  : vent circle must fit between the VC bore and ledge OD,
//                   and vents must not overlap circumferentially.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace speaker_basket_spider_seat {

constexpr const char* kSkillId = "speaker_basket_spider_seat";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy            { 0.0, 0.0, 0.0 };
    double    spider_ledge_dia_mm  { 0.0 };   // OD of the shallow ledge
    double    ledge_depth_mm       { 0.0 };   // counterbore depth
    double    vc_clearance_dia_mm  { 0.0 };   // through bore for VC former
    int       vent_count           { 0 };     // vent holes around the ledge
    double    vent_dia_mm          { 0.0 };
    double    vent_circle_dia_mm   { 0.0 };   // PCD of vent holes
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace speaker_basket_spider_seat
}  // namespace koocadcam::skill
