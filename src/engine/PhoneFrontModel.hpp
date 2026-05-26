#pragma once
// @lat: [[engine/feature-phone#Build Sequence]]
//
// PhoneFrontModel — first parametric product to compose koocadcam::engine::prim
// helpers as the encapsulation contract.  Validates that the primitives layer
// extracted in [[engine/parametric-templates#기하 프리미티브 레이어]] handles
// rectangular geometries as cleanly as the round watch case.

#include "ProductFrontModel.hpp"

#include <TopoDS_Shape.hxx>

#include <nlohmann/json_fwd.hpp>

#include <vector>

namespace koocadcam::engine {

// M2-phase-1: 4-step rectangular-slab phone front-case builder.
// Steps 5–7 (side buttons, port hole, camera deco rings) land in M2-phase-2.
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

    // ── Convenience: run all 4 M2-phase-1 steps in order.
    static TopoDS_Shape buildAll(const nlohmann::json& spec,
                                 std::vector<BuildWarning>& warnings);

    // Default spec — 76 × 160 × 8 mm modern-smartphone baseline.
    // Used by "File > New Phone (Rectangular)" menu and phone_test fixture.
    static nlohmann::json defaultSpec();
};

}  // namespace koocadcam::engine
