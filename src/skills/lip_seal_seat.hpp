#pragma once
// @lat: [[engine/skills#lip_seal_seat]]
//
// lip_seal_seat — COMPOUND MACRO: shaft-seal (oil-seal / lip-seal) seat
// per ISO 6194 / DIN 3760: bore + lip-seal counterbore + dust-lip clearance
// groove + nose chamfer for installation.
//
//                           ────┬───────────┬────   ← body_face entry
//          15° nose         ────┘╲         ╱└────   ← seal nose chamfer
//                                 ╲       ╱        seal_thickness T
//          seal seat            ╔══╗     ╔══╗      counterbore at OD
//                          ─────╨──╨─────╨──╨─────  ← dust-lip groove
//                                                   (clearance for dust lip)
//          shaft bore            │  shaft_id  │
//                                │            │
//                          ─────────────────────
//
// Sub-feature chain (geometric):
//   1. SHAFT BORE — full-depth cylindrical bore at shaft_id (the seal's
//      sliding ID must clear this).
//   2. LIP-SEAL COUNTERBORE — coaxial larger-diameter cylindrical seat
//      (seal_od × seal_thickness) at the body_face for the seal OD press fit.
//   3. DUST-LIP CLEARANCE GROOVE — small annular relief at the BOTTOM of
//      the counterbore (between bore and counterbore) to clear the seal's
//      secondary dust lip.  Depth ≈ 0.5 mm, width ≈ 1.5 mm.
//   4. NOSE CHAMFER — 15° lead-in chamfer at the body face entry, on the
//      counterbore OD — assists seal installation without nicking the
//      sealing surface.
//
// Counterbore + chamfer use the "fuse_first_then_cut" pattern: build
// the lip-seal counterbore tool + nose-chamfer cone frustum, fuse them
// into one tool, then perform a single pr::cut.
//
// Reference: ISO 6194-1 (rotary shaft seals) + SAE J518 (hydraulic):
//   - Counterbore tolerance: H8 typical, H7 for tight fits.
//   - Bore surface finish: Ra ≤ 1.6 μm on the seal seat.
//   - Nose chamfer: 15° × 1.5 mm typical (DIN 3760 fig. 4).
//   - Dust-lip clearance: 0.5 mm radial × 1.5 mm axial.
//
// Signature pattern: kind = "lip_seal_seat", is_compound = true,
// subfeature_count = 4.
//
// DFM gates:
//   - shaft_id ≥ 1.0 mm
//   - seal_od > shaft_id + 2.0 mm (seal wall ≥ 1 mm)
//   - seal_thickness > 0
//   - body_face must resolve to a planar face.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lip_seal_seat {

constexpr const char* kSkillId = "lip_seal_seat";

struct Input
{
    FaceDatum   body_face;
    double      center_x_mm        = 0.0;
    double      center_y_mm        = 0.0;
    gp_Dir      axis_dir           { 0.0, 0.0, -1.0 };
    double      shaft_id_mm        = 0.0;   // shaft bore diameter
    double      seal_od_mm         = 0.0;   // seal OD (counterbore dia)
    double      seal_thickness_mm  = 0.0;   // seal axial thickness (cb depth)
    double      bore_depth_mm      = 0.0;   // shaft bore total depth (incl. CB)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lip_seal_seat
}  // namespace koocadcam::skill
