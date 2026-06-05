#pragma once
// @lat: [[engine/skills#fastener_countersink_array_nas]]
//
// fastener_countersink_array_nas — NAS flush-rivet countersink array along a
// seam (aerospace structures).
//
// Flush (countersunk) rivets along a skin seam each need a drilled pilot hole
// plus a conical countersink so the rivet head sits flush.  NAS standards
// (e.g. NAS523/NAS618) use 100-degree countersinks predominantly; 82 and 120
// degree are also catalogued.  We model N (bore + countersink) pairs.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   per fastener: 1) drilled bore   2) conical countersink (coneFrustum)
//   subfeature_count = count * 2.
//
// DFM:
//   DFM-INPUT  : every dimension/count must be > 0.
//   DFM-ANGLE  : csk_angle_deg must be one of {82, 100, 120}.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace fastener_countersink_array_nas {

constexpr const char* kSkillId = "fastener_countersink_array_nas";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    row_origin      { 0.0, 0.0, 0.0 };   // first fastener centre (XY)
    double    fastener_dia_mm { 4.0 };             // rivet shank diameter
    double    csk_angle_deg   { 100.0 };           // included countersink angle
    int       count           { 4 };
    double    pitch_mm        { 12.0 };            // centre-to-centre along +X
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace fastener_countersink_array_nas
}  // namespace koocadcam::skill
