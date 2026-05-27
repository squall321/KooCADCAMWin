#pragma once
// @lat: [[engine/skills#face_milling]]
//
// face_milling — remove a uniform thickness layer from a planar face
// (also called "skim cut" or "resurfacing").
//
//   Before:                       After (depth = d):
//
//        ┌─────────────┐               ┌─────────────┐    ↑
//        │             │               │             │    │
//        │             │               │ (new face,  │    │ d
//        │             │               │  parallel)  │    ↓
//        │             │               ├─────────────┤
//        │             │               │             │
//        │             │               │             │
//        │             │               │             │
//        └─────────────┘               └─────────────┘
//        (entry_face                    (entry_face has been
//         is top of stock)               offset inward by depth_mm)
//
// Signature pattern:
//   - The original entry face has been replaced by a parallel planar face
//     offset by `depth_mm` along the face's outward normal (inward into
//     the material).  The replacement face has the same projected
//     footprint as the entry face (no sidewall change).
//
// DFM checks:
//   depth_mm > 0
//   depth_mm < 5.0  (warning if greater — typical face mill pass ≤ 5 mm)
//
// Recognize (limited):
//   Without prior workpiece state, full recognition is undecidable — every
//   planar face of a milled block could equally be "stock with no skim" or
//   "stock skimmed by an arbitrary amount."  We therefore emit a recognise
//   candidate per planar face that matches the stock bounding box's
//   projected area at that level, with low confidence (0.40).  Round-trip
//   tests work because the test fixture knows the prior face area.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace face_milling {

constexpr const char* kSkillId = "face_milling";

struct Input
{
    FaceDatum entry_face;        // a planar face on the workpiece
    double    depth_mm = 0.0;    // how much material to remove
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace face_milling
}  // namespace koocadcam::skill
