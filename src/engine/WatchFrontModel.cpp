// @lat: [[engine/feature-watch#Build Sequence]]
// Watch front-case build sequence (6 of 11 steps, M1.2).
//
// Each step body is wrapped by `prim::runStep`, which provides uniform
// exception → E_OCCT, BRepCheck → W_BREPCHECK, and spdlog::debug logging.
// The step bodies themselves consist of: read spec values → describe geometry
// using `prim::` helpers → return shape.  Pass-through (no key in spec)
// short-circuits before runStep so we don't re-validate the unchanged input.

#include "WatchFrontModel.hpp"

#include "primitives/Bbox.hpp"
#include "primitives/Cuts.hpp"
#include "primitives/Fillets.hpp"
#include "primitives/Frames.hpp"
#include "primitives/StepGuard.hpp"
#include "primitives/Tools.hpp"

#include <Standard_Failure.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <iterator>

namespace koocadcam::engine {

namespace pr = prim;

// @lat: [[engine/feature-watch#스텝 1 — buildBase]]
StepResult WatchFrontModel::buildBase(const nlohmann::json& spec)
{
    return pr::runStep("WatchFrontModel::buildBase",
        [&](std::vector<BuildWarning>&) -> TopoDS_Shape {
            const std::string formFactor = spec.value("form_factor", "round");

            if (formFactor == "round") {
                const double diameter  = spec["base"]["diameter_mm"].get<double>();
                const double thickness = spec["base"]["thickness_mm"].get<double>();
                return pr::cylinder(pr::axisAtZ(gp_Pnt(0, 0, 0)),
                                    diameter / 2.0, thickness);
            }

            if (formFactor == "square") {
                const double width     = spec["base"]["width_mm"].get<double>();
                const double height    = spec["base"]["height_mm"].get<double>();
                const double thickness = spec["base"]["thickness_mm"].get<double>();
                const double initR     = spec.value(
                    nlohmann::json::json_pointer("/base/initial_corner_r_mm"), 2.0);

                const TopoDS_Shape boxShape = pr::box(
                    gp_Ax2(gp_Pnt(-width / 2.0, -height / 2.0, 0.0), gp::DZ()),
                    width, height, thickness);

                return pr::filletEdges(boxShape, initR,
                    pr::verticalCornerEdges(width / 2.0, height / 2.0, thickness));
            }

            throw Standard_Failure(
                ("buildBase: unknown form_factor '" + formFactor + "'").c_str());
        },
        /*checkValidity=*/false);   // primitive solids are always valid
}

// @lat: [[engine/feature-watch#스텝 2 — applyCornerRadius]]
StepResult WatchFrontModel::applyCornerRadius(const TopoDS_Shape& in,
                                               const nlohmann::json& spec)
{
    return pr::runStep("WatchFrontModel::applyCornerRadius",
        [&](std::vector<BuildWarning>&) {
            const double rTop  = spec["corner_radius"]["r_top_mm"].get<double>();
            const double rSide = spec["corner_radius"]["r_side_mm"].get<double>();
            const auto bb = pr::optimalBbox(in);
            return pr::filletEdgesMulti(in, {
                { rTop,  pr::edgesAtZ(bb.zMax) },
                { rSide, pr::edgesAtZ(bb.zMin) }
            });
        });
}

// @lat: [[engine/feature-watch#스텝 3 — buildBezel]]
StepResult WatchFrontModel::buildBezel(const TopoDS_Shape& in,
                                       const nlohmann::json& spec)
{
    return pr::runStep("WatchFrontModel::buildBezel",
        [&](std::vector<BuildWarning>& warnings) {
            const auto bb = pr::optimalBbox(in);
            const double outerR     = bb.outerRadiusXY();
            const double bezelWidth = spec["bezel"]["width_mm"].get<double>();
            const double bezelDepth = spec["bezel"]["depth_mm"].get<double>();
            const double taperDeg   = spec["bezel"].value("taper_deg", 0.0);
            const double innerR     = outerR - bezelWidth;

            const gp_Ax2 ax = pr::axisAtZ(gp_Pnt(0.0, 0.0, bb.zMax - bezelDepth));

            TopoDS_Shape tool;
            if (taperDeg > 0.0) {
                const double bottomR =
                    outerR - bezelDepth * std::tan(taperDeg * M_PI / 180.0);
                if (bottomR > innerR) {
                    tool = pr::annularConeRing(ax, bottomR, outerR, innerR, bezelDepth);
                } else {
                    warnings.push_back(BuildWarning{
                        "W_BEZEL_TAPER_FALLBACK",
                        "bezel taper would invert inner radius; using straight walls"
                    });
                    tool = pr::annularRing(ax, outerR, innerR, bezelDepth);
                }
            } else {
                tool = pr::annularRing(ax, outerR, innerR, bezelDepth);
            }
            return pr::cut(in, tool);
        });
}

// @lat: [[engine/feature-watch#스텝 4 — buildDisplayPocket]]
StepResult WatchFrontModel::buildDisplayPocket(const TopoDS_Shape& in,
                                                const nlohmann::json& spec)
{
    if (!spec.contains("display_pocket")) return StepResult{ in, {} };

    return pr::runStep("WatchFrontModel::buildDisplayPocket",
        [&](std::vector<BuildWarning>&) {
            const auto& dp = spec["display_pocket"];
            const double dPocket     = dp["d_pocket_mm"].get<double>();
            const double depthPocket = dp["depth_pocket_mm"].get<double>();
            const double glassOff    = dp.value("glass_offset_mm", 0.0);

            const auto bb = pr::optimalBbox(in);
            const double recessAmt = std::max(0.0, -glassOff);
            const double zBottom   = bb.zMax - depthPocket - recessAmt;
            const double height    = depthPocket + std::abs(glassOff);

            const TopoDS_Shape pocket = pr::cylinder(
                pr::axisAtZ(gp_Pnt(0.0, 0.0, zBottom)), dPocket / 2.0, height);
            return pr::cut(in, pocket);
        });
}

// @lat: [[engine/feature-watch#스텝 5 — addCrownCavity]]
StepResult WatchFrontModel::addCrownCavity(const TopoDS_Shape& in,
                                            const nlohmann::json& spec)
{
    if (!spec.contains("crown_cavity")) return StepResult{ in, {} };

    return pr::runStep("WatchFrontModel::addCrownCavity",
        [&](std::vector<BuildWarning>&) {
            const auto& cc = spec["crown_cavity"];
            const double angleDeg = cc["side_angle_deg"].get<double>();
            const double heightZ  = cc["height_pos_mm"].get<double>();
            const double depth    = cc["depth_mm"].get<double>();
            const double diameter = cc["diameter_mm"].get<double>();
            const double shaftDia = cc["shaft_dia_mm"].get<double>();

            const double R       = pr::optimalBbox(in).outerRadiusXY();
            const auto   frame   = pr::sideFrameAt(R, angleDeg, heightZ);
            const gp_Ax2 ax      = frame.ax2InwardRadial();

            const TopoDS_Shape cavity = pr::cylinder(ax, diameter / 2.0, depth);
            const TopoDS_Shape shaft  = pr::cylinder(ax, shaftDia / 2.0, R + 1.0);
            return pr::cutMany(in, { cavity, shaft });
        });
}

// @lat: [[engine/feature-watch#스텝 6 — addSideButtons]]
StepResult WatchFrontModel::addSideButtons(const TopoDS_Shape& in,
                                            const nlohmann::json& spec)
{
    if (!spec.contains("side_buttons") || spec["side_buttons"].empty())
        return StepResult{ in, {} };

    return pr::runStep("WatchFrontModel::addSideButtons",
        [&](std::vector<BuildWarning>& warnings) {
            const double R = pr::optimalBbox(in).outerRadiusXY();
            const auto& buttons = spec["side_buttons"];

            std::vector<TopoDS_Shape> tools;
            tools.reserve(buttons.size());
            for (const auto& btn : buttons) {
                if (btn.value("taper_deg", 0.0) > 0.0) {
                    warnings.push_back(BuildWarning{
                        "W_TAPER_M1_5",
                        "side_buttons taper_deg > 0 not implemented (M1.5); treated as 0"
                    });
                }
                const auto frame = pr::sideFrameAt(R,
                    btn["angle_deg"].get<double>(),
                    btn["height_mm"].get<double>());
                tools.push_back(pr::sidePocketBox(frame,
                    btn["depth_mm"].get<double>(),
                    btn["length_mm"].get<double>(),
                    btn["width_mm"].get<double>()));
            }
            return pr::cutMany(in, tools);
        });
}

// ── Convenience: chain all M1.2 steps (1–6) ────────────────────────────────
TopoDS_Shape WatchFrontModel::buildAll(const nlohmann::json& spec,
                                        std::vector<BuildWarning>& warnings)
{
    WatchFrontModel m;

    auto absorb = [&warnings](StepResult&& r, const char* tag) -> TopoDS_Shape {
        warnings.insert(warnings.end(),
                        std::make_move_iterator(r.warnings.begin()),
                        std::make_move_iterator(r.warnings.end()));
        if (r.shape.IsNull())
            spdlog::error("WatchFrontModel::buildAll aborted at {}", tag);
        return r.shape;
    };

    TopoDS_Shape s;
    s = absorb(m.buildBase(spec),             "step1 buildBase");      if (s.IsNull()) return {};
    s = absorb(m.applyCornerRadius(s, spec),  "step2 cornerRad");      if (s.IsNull()) return {};
    s = absorb(m.buildBezel(s, spec),         "step3 buildBezel");     if (s.IsNull()) return {};
    s = absorb(m.buildDisplayPocket(s, spec), "step4 displayPocket");  if (s.IsNull()) return {};
    s = absorb(m.addCrownCavity(s, spec),     "step5 crownCavity");    if (s.IsNull()) return {};
    s = absorb(m.addSideButtons(s, spec),     "step6 sideButtons");    if (s.IsNull()) return {};
    return s;
}

// ── Default spec: 44 mm round watch baseline ───────────────────────────────
nlohmann::json WatchFrontModel::defaultSpec()
{
    return nlohmann::json{
        { "schema_version", "1.0.0" },
        { "product_name",   "Default Round Watch 44mm" },
        { "form_factor",    "round" },
        { "base", {
            { "diameter_mm",  44.0 },
            { "thickness_mm", 10.0 }
        }},
        { "corner_radius", {
            { "r_top_mm",  1.0 },
            { "r_side_mm", 0.6 }
        }},
        { "bezel", {
            { "width_mm",   3.0 },
            { "depth_mm",   1.0 },
            { "taper_deg",  0.0 }
        }},
        { "display_pocket", {
            { "d_pocket_mm",      36.0  },
            { "depth_pocket_mm",   0.8  },
            { "glass_offset_mm",  -0.1  }
        }},
        { "crown_cavity", {
            { "side_angle_deg",  0.0 },
            { "height_pos_mm",   5.0 },
            { "depth_mm",        2.5 },
            { "diameter_mm",     3.0 },
            { "shaft_dia_mm",    1.5 }
        }},
        { "side_buttons", nlohmann::json::array({
            nlohmann::json{
                { "angle_deg",  60.0 },
                { "height_mm",   5.0 },
                { "length_mm",   5.0 },
                { "width_mm",    2.0 },
                { "depth_mm",    0.8 },
                { "taper_deg",   0.0 }
            }
        })}
    };
}

}  // namespace koocadcam::engine
