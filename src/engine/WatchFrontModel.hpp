#pragma once
// @lat: [[engine/feature-watch#Build Sequence]]

#include "ProductFrontModel.hpp"

#include <TopoDS_Shape.hxx>

#include <nlohmann/json_fwd.hpp>

#include <string>
#include <vector>

namespace koocadcam::engine {

// ── DFM finding (per [[engine/dfm-rules]] catalog) ────────────────────────
struct DFMFinding
{
    std::string code;       // "DFM-002", "DFM-009", ...
    std::string severity;   // "error" | "warning" | "info"
    std::string message;
};

struct DFMReport
{
    bool                     passed = true;   // false if any "error" finding
    std::vector<DFMFinding>  findings;

    // Uniform accumulator (mirrors skill::DFMReport::add) — prerequisite
    // for the product-agnostic DFM rule registry (breakthrough plan B5).
    void add(std::string code, std::string severity, std::string message)
    {
        if (severity == "error") passed = false;
        findings.push_back(DFMFinding{ std::move(code), std::move(severity),
                                       std::move(message) });
    }
};

// M1.5: full 11-step watch front-case builder (M1.1 steps 1-3, M1.2 steps
// 4-6, M1.5 steps 7-10 + runDFM).
class WatchFrontModel : public ProductFrontModel<WatchFrontModel>
{
public:
    WatchFrontModel() = default;

    // ── Step 1: base puck (round or square form factor)
    StepResult buildBase(const nlohmann::json& spec);

    // ── Step 2: top/side corner fillets (single filletEdgesMulti pass)
    StepResult applyCornerRadius(const TopoDS_Shape& in,
                                 const nlohmann::json& spec);

    // ── Step 3: bezel annular pocket (cone-frustum outer if taper_deg > 0)
    StepResult buildBezel(const TopoDS_Shape& in,
                          const nlohmann::json& spec);

    // ── Step 4: display pocket (circular recess on top face) — optional
    StepResult buildDisplayPocket(const TopoDS_Shape& in,
                                  const nlohmann::json& spec);

    // ── Step 5: crown cavity + shaft hole on side surface — optional
    StepResult addCrownCavity(const TopoDS_Shape& in,
                              const nlohmann::json& spec);

    // ── Step 6: side button pockets from array spec — optional
    StepResult addSideButtons(const TopoDS_Shape& in,
                              const nlohmann::json& spec);

    // ── Step 7: speaker grille (rows × cols pattern of small holes on side)
    // spec["speaker_grille"] missing → pass-through.
    // spec.speaker_grille = { angle_deg, height_pos_mm, rows, cols,
    //                         row_spacing_mm, col_spacing_mm,
    //                         hole_dia_mm, depth_mm }
    StepResult addSpeakerGrille(const TopoDS_Shape& in,
                                const nlohmann::json& spec);

    // ── Step 8: rear sensor holes (HR/PPG optical windows on bottom face)
    // spec["rear_sensors"] missing or empty → pass-through.
    // Each item = { offset_x_mm, offset_y_mm, dia_mm, depth_mm }
    StepResult addRearSensors(const TopoDS_Shape& in,
                              const nlohmann::json& spec);

    // ── Step 9: strap lugs (material protrusions, prim::fuseMany)
    // spec["lugs"] missing or empty → pass-through.
    // Each item = { angle_deg, length_mm, width_mm, thickness_mm,
    //               pin_hole_dia_mm (optional) }
    StepResult addLugs(const TopoDS_Shape& in,
                       const nlohmann::json& spec);

    // ── Step 10: secondary fillets — polish display rim + bezel inner step
    // spec["secondary_fillets"] missing → pass-through.
    // spec.secondary_fillets = { r_mm }
    StepResult addSecondaryFillets(const TopoDS_Shape& in,
                                   const nlohmann::json& spec);

    // ── Convenience: run all 10 steps in order, collecting warnings.
    static TopoDS_Shape buildAll(const nlohmann::json& spec,
                                 std::vector<BuildWarning>& warnings);

    // ── Step 11: DFM rule catalog validation (separate from buildAll).
    // Reads from [[engine/dfm-rules]] — currently implements DFM-002 / 009 /
    // 014 / 020 / 022 (spec-driven + topology-driven mix).  Returns a report
    // even if the shape is null (will produce an error finding).
    static DFMReport runDFM(const TopoDS_Shape& shape,
                            const nlohmann::json& spec);

    // Default spec — 44 mm round watch baseline, all 10 steps populated.
    static nlohmann::json defaultSpec();
};

}  // namespace koocadcam::engine
