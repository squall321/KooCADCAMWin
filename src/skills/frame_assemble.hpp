#pragma once
// @lat: [[engine/skills#frame_assemble]]
//
// frame_assemble — site/shop assembly of a structural frame from prepared
// members.  Frame type may be steel (bolted moment frame, braced bay) or
// wood (light-frame stud wall, timber post-and-beam).  KooCADCAM phase-1
// captures the count of members and joints to drive crew sizing and
// erection-sequence planning; the individual member B-reps live in their
// own workpieces.
//
//              ┌──────┬──────┬──────┐
//              │      │      │      │   ← frame_type: steel | wood
//              │      │      │      │     member_count = beams + columns
//              └──────┴──────┴──────┘     joint_count  = connections
//
// Signature pattern:
//   - pattern.kind             = "frame_assemble"
//   - pattern.frame_type       = "steel" | "wood"
//   - pattern.member_count     = number of members in the frame
//   - pattern.joint_count      = number of joints / connections
//   - pattern.joints_per_member = joint_count / member_count
//   - pattern.geometry_changed = false
//
// DFM checks:
//   member_count ≥ 2                          (DFM-INPUT error — frame needs ≥ 2)
//   joint_count  ≥ 1                          (DFM-INPUT error)
//   frame_type   ∈ {"steel","wood"}           (DFM-FRAME-TYPE error)
//   joint_count  > 4 × member_count           (DFM-FRAME-JOINT info — unusually busy)
//   joint_count  < member_count − 1           (DFM-FRAME-JOINT info — frame may be disconnected)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace frame_assemble {

constexpr const char* kSkillId = "frame_assemble";

struct Input
{
    std::string  frame_type    = "steel";   // "steel" | "wood"
    int          member_count  = 4;         // beams + columns + braces
    int          joint_count   = 4;         // connections / fasteners groups
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace frame_assemble
}  // namespace koocadcam::skill
