#pragma once
// @lat: [[engine/skills#parametric_rib_array]]
//
// parametric_rib_array — COMPOUND, SPEC-DRIVEN macro that places N
// rectangular reinforcing ribs at uniform pitch along a base direction,
// raised from a planar face.
//
//                        ◄── pitch ──►◄── pitch ──►◄── pitch ──►
//                        │           │           │           │
//                  ┌─┐   ┌─┐   ┌─┐   ┌─┐   ┌─┐   ┌─┐   ┌─┐   ┌─┐  ↑ height
//                  │ │   │ │   │ │   │ │   │ │   │ │   │ │   │ │   │
//        ──────────┴─┴───┴─┴───┴─┴───┴─┴───┴─┴───┴─┴───┴─┴───┴─┴──  entry_face
//                                                                  (planar)
//                        ◄────────── total span ──────────►
//
// Each rib is a box of (length × thickness × height) standing on the entry
// face, oriented so that:
//   • the long dimension `length` runs perpendicular to the array axis
//     (each rib looks like a fence picket — long across, narrow along axis),
//   • `thickness` runs along the array axis,
//   • `height` extends along the face's outward normal.
//
// Spec library — `spec_class` selects a (thickness, max H/T, draft) preset
// for the most common rib styles in mould design:
//
//   "mold_thin"      → thickness 0.8 mm, max height = 3×T, draft 1°
//   "mold_standard"  → thickness 1.2 mm, max height = 3×T, draft 1°
//   "mold_heavy"     → thickness 2.0 mm, max height = 4×T, draft 1°
//   "structural"     → thickness 3.0 mm, max height = 5×T, draft 0.5°
//   "air_flow_fin"   → thickness 0.6 mm, max height = 8×T, draft 0° (heatsink)
//
// `thickness_mm` is OPTIONAL.  If 0, the spec table thickness is used;
// otherwise the caller's value wins (with DFM validating it against the
// spec range).
//
// Implementation strategy (slice 1 — REAL multi-cut compound geometry):
//   1. Resolve the spec_class to a (default_thickness, max_HT_ratio, draft).
//   2. Compute N rib center XYs at uniform `pitch_mm` along the axis,
//      centred on (anchor_x, anchor_y).
//   3. For each rib, build a box solid and fuse-add to the workpiece.
//   4. Record per-rib position + thickness in `signature.pattern.ribs[]`.
//
// Signature pattern:
//   - kind = "parametric_rib_array"
//   - is_compound = true
//   - subfeature_count = N
//   - spec_class
//   - rib_count, pitch_mm, length_mm, thickness_mm, height_mm
//   - axis_dir (the array axis), extrude_dir (the face outward normal)
//   - ribs[]: array of N { id, center_x, center_y }
//
// DFM (uses mould-rule standard table):
//   DFM-WALL-MIN     : thickness ≥ 0.4 mm (universal mould minimum).
//   DFM-RIB-SPACING  : pitch ≥ 2 × thickness (else fuse-fail / sink marks).
//   DFM-RIB-COUNT    : rib_count ≥ 1.
//   DFM-RIB-ASPECT   : height / thickness ≤ max_HT_ratio (from spec).
//   DFM-RIB-UNKNOWN  : spec_class not in library.
//   DFM-RIB-LENGTH   : length_mm > 0.
//
// Recognize:
//   Replay strategy is primary (uses feature history).  Geometric fallback:
//   scan vertical planar wall faces, group by parallel-perpendicular pairs
//   at uniform stride along a common axis.  Confidence ≈ 0.55 on geometry,
//   0.92 on history replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace parametric_rib_array {

constexpr const char* kSkillId = "parametric_rib_array";

struct Input
{
    FaceDatum   entry_face;                       // planar base face
    std::string spec_class;                       // "mold_standard", "structural", …
    int         rib_count       = 0;              // N (must be ≥ 1)
    double      pitch_mm        = 0.0;            // distance between rib centres
    double      length_mm       = 0.0;            // each rib's long dimension
    double      height_mm       = 0.0;            // extrusion height (above face)
    double      thickness_mm    = 0.0;            // 0 → lookup from spec_class
    double      anchor_x_mm     = 0.0;            // centre of the array (world XY)
    double      anchor_y_mm     = 0.0;
    gp_Dir      array_axis      { 1.0, 0.0, 0.0 };// direction of the pitch
    gp_Dir      extrude_dir     { 0.0, 0.0, 1.0 };// rib growth dir (face outward)
};

// Spec lookup — returns spec_thickness_mm (0.0 if unknown).
double specThicknessFor (const std::string& spec_class);
double specMaxHTRatioFor(const std::string& spec_class);
double specDraftDegFor  (const std::string& spec_class);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace parametric_rib_array
}  // namespace koocadcam::skill
