#pragma once
// @lat: [[engine/skills#heat_sink_fin_array]]
//
// heat_sink_fin_array — COMPOUND, SPEC-DRIVEN feature
//
// Fuses N rectangular fins onto a planar base face, forming an extruded
// heat-sink fin array.  The fins are upright rectangular prisms whose long
// axis is aligned with the airflow direction; their short axis is the
// inter-fin pitch.
//
//                        airflow →
//                ┌──┐ ┌──┐ ┌──┐ ┌──┐ ┌──┐  ← N fins, height fin_height_mm
//                │  │ │  │ │  │ │  │ │  │
//                ├──┴─┴──┴─┴──┴─┴──┴─┴──┤  ← base (existing workpiece face)
//                │           BASE          │
//                └──────────────────────┘
//
//                 │←pitch→│
//                 ├──┤←fin_thickness
//
// Spec library (manufacturable AR ≤ 10):
//   "T-30"  : pitch=3.0,  fin_thickness=1.0, fin_height=8    (small electronics, AR=8)
//   "T-50"  : pitch=5.0,  fin_thickness=1.5, fin_height=15   (medium, AR=10)
//   "T-80"  : pitch=8.0,  fin_thickness=2.0, fin_height=20   (power supplies, AR=10)
//   "T-120" : pitch=12.0, fin_thickness=3.0, fin_height=30   (large equipment, AR=10)
//
// All three dimensions are mutually consistent and stay within the
// manufacturable AR ≤ 10 gate.  The caller passes a spec key + fin_count +
// base_face; everything else is derived.
//
// Compound chain:
//   for i in 0..N-1:    build_fin_box(i)   (rectangular prism above base)
//   fuse_many           → single compound fin manifold
//   fuse                → fuse onto workpiece (additive — heat sink fins ADD)
//
// DFM gates:
//   DFM-FIN-AR : fin aspect ratio (fin_height / fin_thickness) ≤ 10 : 1
//                (manufacturable extrusion / mill / EDM aspect ratio).
//   DFM-FIN-SP : unknown spec key.
//   DFM-FIN-N  : fin_count < 2 (a single fin is just a rib, not an array).
//   DFM-FIN-FIT: N · fin_thickness + (N−1) · gap > base_face length →
//                fins won't fit on the base.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace heat_sink_fin_array {

constexpr const char* kSkillId = "heat_sink_fin_array";

struct Input
{
    FaceDatum   base_face;
    int         fin_count          = 0;
    std::string spec_key;                 // "T-30" | "T-50" | "T-80" | "T-120" | ""
    // Optional explicit overrides (used only when spec_key is empty).
    double      fin_thickness_mm   = 0.0;
    double      fin_height_mm      = 0.0;
    double      fin_pitch_mm       = 0.0;
    // Fin length axis (in-plane on the base).  Defaults to +X.
    gp_Dir      airflow_dir        { 1.0, 0.0, 0.0 };
    // Origin of the FIRST fin's left-back corner (XY of base).
    double      origin_x_mm        = 0.0;
    double      origin_y_mm        = 0.0;
    // Fin length along the airflow axis.  Defaults to 30 mm if 0.
    double      fin_length_mm      = 0.0;
};

struct FinSpec {
    double pitch_mm        = 0.0;
    double fin_thickness_mm = 0.0;
    double fin_height_mm   = 0.0;
};

// Spec-table lookup.  Returns a zeroed FinSpec when key is unknown.
FinSpec finSpecFor(const std::string& spec_key);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace heat_sink_fin_array
}  // namespace koocadcam::skill
