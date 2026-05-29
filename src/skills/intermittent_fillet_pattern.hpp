#pragma once
// @lat: [[engine/skills#intermittent_fillet_pattern]]
//
// intermittent_fillet_pattern — COMPOUND MACRO that lays out a pattern of N
// short fillet-weld zones along an edge with a given pitch (AWS A2.4
// "intermittent fillet" symbol).
//
// Geometric sub-features (≥ 2, segment_count drives the actual count):
//   - For each segment we make a SHALLOW witness mark on the entry face
//     (0.05 mm deep small slot box) so downstream inspection can recover
//     the pattern from B-rep.  Welding itself is metadata only — the parent
//     spec calls out "no geometric change — pattern only" but the contract
//     also requires ≥ 2 primitive cuts; the witness marks resolve that.
//
// Pattern layout (segment_count segments separated by pitch along edge_dir):
//
//        ▓▓░░▓▓░░▓▓░░▓▓░░▓▓        ▓ = segment_length  ░ = (pitch - segment_length)
//
// AWS symbol notation: "L - P" where L = segment_length, P = pitch.
//
// DFM checks (AWS D1.1 / ISO 2553):
//   DFM-WELD-IF-COUNT  : segment_count >= 2.
//   DFM-WELD-IF-PITCH  : pitch >= segment_length.
//   DFM-WELD-IF-LEN    : segment_length >= 25 mm  (AWS D1.1 minimum).
//
// Recognize: metadata replay only (confidence = 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace intermittent_fillet_pattern {

constexpr const char* kSkillId = "intermittent_fillet_pattern";

struct Input
{
    FaceDatum   entry_face;
    gp_Dir      edge_dir         { 1.0, 0.0,  0.0 };
    int         segment_count    = 0;
    double      segment_length   = 0.0;
    double      pitch            = 0.0;
    double      edge_center_x_mm = 0.0;
    double      edge_center_y_mm = 0.0;
    double      witness_depth_mm = 0.05;   // shallow witness mark depth
    double      witness_width_mm = 1.0;    // witness mark width across the edge
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace intermittent_fillet_pattern
}  // namespace koocadcam::skill
