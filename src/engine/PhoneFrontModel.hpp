#pragma once
// @lat: [[engine/feature-phone#Build Sequence]]
//
// PhoneFrontModel — first parametric product to compose koocadcam::engine::prim
// helpers as the encapsulation contract.  Validates that the primitives layer
// extracted in [[engine/parametric-templates#기하 프리미티브 레이어]] handles
// rectangular geometries as cleanly as the round watch case.

#include "ProductFrontModel.hpp"
#include "dfm/DFMReport.hpp"

#include <TopoDS_Shape.hxx>

#include <nlohmann/json_fwd.hpp>

#include <vector>

namespace koocadcam::engine {

// 7-step rectangular-slab phone front-case builder.  Steps 1–4 (M2-phase-1):
// base, corner rim fillets, display pocket, rear camera holes.  Steps 5–7
// (M2-phase-2): side buttons, USB-C port, camera deco rings.
class PhoneFrontModel : public ProductFrontModel<PhoneFrontModel>
{
public:
    PhoneFrontModel() = default;

    // ── Step 1: rectangular base
    // spec["base"] = { width_mm, height_mm, thickness_mm, initial_corner_r_mm }
    // Box centred on XY, fillets the 4 vertical corner edges.
    StepResult buildBase(const nlohmann::json& spec);

    // ── Step 2: front/back rim fillets
    // spec["corner_radius"] = { r_top_mm, r_side_mm }
    // Top-face edges → r_top_mm, bottom-face edges → r_side_mm.  Single-pass
    // filletEdgesMulti so the OCCT solver handles both rims together.
    StepResult applyCornerRadius(const TopoDS_Shape& in,
                                 const nlohmann::json& spec);

    // ── Step 3: display pocket (large rectangular recess on front face)
    // spec["display_pocket"] missing → pass-through.
    // Composes prim::roundedRectPocketTool + prim::cut.
    StepResult buildDisplayPocket(const TopoDS_Shape& in,
                                  const nlohmann::json& spec);

    // ── Step 4: rear camera holes (cylindrical lens cut-outs)
    // spec["cameras"] missing or empty → pass-through.
    // Each camera = { offset_x_mm, offset_y_mm, hole_dia_mm, depth_mm }.
    // Holes are perpendicular to the rear (bottom) face, going up into body.
    // Compound + single Boolean via prim::cutMany.
    StepResult addCameraHoles(const TopoDS_Shape& in,
                              const nlohmann::json& spec);

    // ── Step 5: side buttons (rectangular pockets on the flat ±X faces)
    // spec["side_buttons"] missing or empty → pass-through.  Each button =
    // { side ("left"|"right"), center_y_mm, center_z_mm, length_mm, height_mm,
    //   depth_mm }.  Compound + single Boolean via prim::cutMany.
    StepResult addSideButtons(const TopoDS_Shape& in,
                              const nlohmann::json& spec);

    // ── Step 6: USB-C port (obround cut-out on the bottom −Y edge)
    // spec["port_hole"] missing → pass-through.
    // { width_mm, height_mm, depth_mm, center_x_mm, center_z_mm }.
    StepResult addPortHole(const TopoDS_Shape& in,
                           const nlohmann::json& spec);

    // ── Step 7: camera deco rings (recessed annuli around rear lenses)
    // spec["camera_deco_rings"] missing or empty → pass-through.  Each ring =
    // { offset_x_mm, offset_y_mm, outer_dia_mm, inner_dia_mm, depth_mm }.
    StepResult addCameraDecoRings(const TopoDS_Shape& in,
                                  const nlohmann::json& spec);

    // ── Convenience: run all 7 steps in order.
    static TopoDS_Shape buildAll(const nlohmann::json& spec,
                                 std::vector<BuildWarning>& warnings);

    // ── DFM rule-catalog validation (shared engine, phone profile) ──
    // Same DFM-001..DFM-023 catalog as the watch via dfm::runProductDFM —
    // phone profile: minWall 0.40 mm, DFM-013 display flatness enabled.
    static DFMReport runDFM(const TopoDS_Shape& shape,
                            const nlohmann::json& spec);

    // Default spec — 76 × 160 × 8 mm modern-smartphone baseline.
    // Used by "File > New Phone (Rectangular)" menu and phone_test fixture.
    static nlohmann::json defaultSpec();
};

}  // namespace koocadcam::engine
