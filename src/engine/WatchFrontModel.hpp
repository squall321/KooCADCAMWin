#pragma once
// @lat: [[engine/feature-watch#Build Sequence]]

#include "ProductFrontModel.hpp"

#include <TopoDS_Shape.hxx>

#include <nlohmann/json_fwd.hpp>

#include <vector>

namespace koocadcam::engine {

// M1.1: First three build steps (buildBase, applyCornerRadius, buildBezel).
// Steps 4–11 land in M1.2 / M1.5.
class WatchFrontModel : public ProductFrontModel<WatchFrontModel>
{
public:
    WatchFrontModel() = default;

    // ── Step 1: base puck (round or square form factor)
    // `spec.form_factor` = "round" → BRepPrimAPI_MakeCylinder(diameter/2, thickness)
    //                    = "square" → BRepPrimAPI_MakeBox(width, height, thickness)
    //                                  + initial fillet on 4 vertical edges
    StepResult buildBase(const nlohmann::json& spec);

    // ── Step 2: top/side corner fillets
    // Walks edges via TopExp_Explorer, filters by Z-coordinate of midpoint:
    //   - top-rim edges  → r_top_mm
    //   - bottom-rim     → r_side_mm
    StepResult applyCornerRadius(const TopoDS_Shape& in,
                                 const nlohmann::json& spec);

    // ── Step 3: bezel annular pocket
    // outer cylinder − inner cylinder = annular tool; BRepAlgoAPI_Cut from base.
    // taper_deg > 0 → cone frustum outer minus straight inner cylinder.
    StepResult buildBezel(const TopoDS_Shape& in,
                          const nlohmann::json& spec);

    // ── Step 4: display pocket (circular recess on top face)
    // spec["display_pocket"] missing → pass-through.
    StepResult buildDisplayPocket(const TopoDS_Shape& in, const nlohmann::json& spec);

    // ── Step 5: crown cavity + shaft hole on side surface (optional)
    // spec["crown_cavity"] missing → pass-through.
    StepResult addCrownCavity(const TopoDS_Shape& in, const nlohmann::json& spec);

    // ── Step 6: side button pockets from array spec (optional)
    // spec["side_buttons"] missing or empty → pass-through.
    StepResult addSideButtons(const TopoDS_Shape& in, const nlohmann::json& spec);

    // ── Convenience: run all M1.2 steps (1–6) in order, collecting warnings.
    // Returns null shape on failure; warnings carries diagnostics.
    static TopoDS_Shape buildAll(const nlohmann::json& spec,
                                 std::vector<BuildWarning>& warnings);

    // Default spec helper — a 44 mm round watch baseline.
    // Used by "File > New Watch (Round)" menu and the watch_test fixture.
    static nlohmann::json defaultSpec();
};

}  // namespace koocadcam::engine
