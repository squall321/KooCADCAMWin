#pragma once
// @lat: [[engine/skills#reference_plane_offset]]
//
// reference_plane_offset — construct an OFFSET REFERENCE PLANE positioned at
// a signed distance from a source planar face's centre, along the source
// face's outward normal.
//
// This is the SolidWorks "Plane → Offset Distance" datum.  It is a *reference
// geometry annotation*: the workpiece B-Rep is NOT modified.  The recovered
// plane (origin + normal) is recorded on the FeatureSignature so downstream
// skills (sketch projection, sectioning, milling reference) can consume it
// without re-resolving the source face.
//
//        source_face (normal n)                offset plane
//        body face                              - - - - - - - -
//                          ── offset_mm ──▶    |             |
//        workpiece                              |  reference  |
//                                               |             |
//                                               - - - - - - - -
//
// Signature pattern:
//   - pattern.kind             = "reference_plane_offset"
//   - pattern.is_reference     = true
//   - pattern.datum_class      = "plane"
//   - pattern.named_id         = <input.name>
//   - pattern.source_face_id   = <input.source_face_id>
//   - pattern.source_face_area = <from face props>
//   - pattern.offset_mm        = <input.offset_mm>
//   - pattern.plane_origin_xyz = [x, y, z]       (faceCenter + n * offset)
//   - pattern.plane_normal_xyz = [nx, ny, nz]    (unit, copies source normal)
//   - pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             : name empty, source_face_id out of range, or
//                           |offset_mm| < 1 µm  (degenerate plane request)
//   DFM-REF-NONPLANAR     : source face is not planar  (error)
//
// Recognize:
//   Reference geometry leaves no B-Rep trace.  recognize() returns ONLY the
//   replayed signatures (confidence 1.0).  No geometric fallback.

#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace reference_plane_offset {

constexpr const char* kSkillId = "reference_plane_offset";

struct Input
{
    int          source_face_id = -1;
    double       offset_mm      = 10.0;
    std::string  name           = "DATUM_A_OFF_10";
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace reference_plane_offset
}  // namespace koocadcam::skill
