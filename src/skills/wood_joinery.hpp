#pragma once
// @lat: [[engine/skills#wood_joinery]]
//
// wood_joinery — record a joint between two wooden workpieces.  This is a
// METADATA-ONLY skill (no geometric change) in slice-1 because joinery
// involves an inter-part relationship rather than a self-modification of a
// single workpiece.  The mating geometry (mortise pocket, tenon, dovetails)
// is produced by the subtractive skills (mill_rect_pocket, dovetail_slot, …)
// and the joinery signature records that they belong to one joint.
//
//   joint_type = "mortise_tenon":
//      ┌────────┐               ┌────────┐    ┌────────┐
//      │  PART  │   stick into  │  PART  │ +  │ tenon  │
//      │   A    │   ───────►   │   A   │    │ on B   │
//      │  ▣ mor │               │ ▢ mor │    │ ▣      │
//      └────────┘               └────────┘    └────────┘
//
//   joint_type = "dovetail":
//      angled flares that lock parts together (drawer fronts).
//
//   joint_type = "finger":         joint_type = "lap":
//      Box / finger joint.            Simple half-lap overlap.
//
// Signature pattern (no geometry change):
//   pattern.kind             = "wood_joinery"
//   pattern.joint_type       = "mortise_tenon" | "dovetail" | "finger" | "lap"
//   pattern.length_mm        = primary dimension (mortise depth / dovetail length)
//   pattern.width_mm         = secondary dimension
//   pattern.thickness_mm     = tenon thickness / pin width
//   pattern.geometry_changed = false
//
// DFM checks:
//   DFM-JOINT-TYPE     joint_type not in supported list (error)
//   DFM-JOINT-LEN      length_mm <= 0                  (error)
//   DFM-JOINT-WID      width_mm  <= 0                  (error)
//   DFM-JOINT-THK      thickness_mm <= 0               (error)
//   DFM-JOINT-RATIO    length / thickness > 5          (info — tenon brittle)
//
// Recognize:
//   Pure metadata replay — joints recorded on the workpiece's feature
//   history are returned as RecognizedFeature with confidence 1.0.  An
//   isolated piece of wood reveals no joint without metadata.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace wood_joinery {

constexpr const char* kSkillId = "wood_joinery";

struct Input
{
    std::string  joint_type    = "mortise_tenon";   // "mortise_tenon"|"dovetail"|"finger"|"lap"
    double       length_mm     = 0.0;               // mortise depth / dovetail length
    double       width_mm      = 0.0;               // mortise width / pin width
    double       thickness_mm  = 0.0;               // tenon thickness / pin thickness
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace wood_joinery
}  // namespace koocadcam::skill
