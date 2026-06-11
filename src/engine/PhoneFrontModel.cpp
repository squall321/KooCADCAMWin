// @lat: [[engine/feature-phone#Build Sequence]]
// First non-watch product model — composes only primitives from
// src/engine/primitives/.  Mirrors WatchFrontModel.cpp's structure to keep
// the contract identical: each step body is wrapped by prim::runStep, no
// raw OCCT (Bnd_Box, BRepCheck, BRepAlgoAPI_*, gp_Ax2) appears here.

#include "PhoneFrontModel.hpp"

#include "dfm/ProductDFM.hpp"

#include "primitives/Bbox.hpp"
#include "primitives/Cuts.hpp"
#include "primitives/Fillets.hpp"
#include "primitives/Frames.hpp"
#include "primitives/StepGuard.hpp"
#include "primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <iterator>

namespace koocadcam::engine {

namespace pr = prim;

// @lat: [[engine/feature-phone#스텝 1 — buildBase]]
StepResult PhoneFrontModel::buildBase(const nlohmann::json& spec)
{
    return pr::runStep("PhoneFrontModel::buildBase",
        [&](std::vector<BuildWarning>&) {
            const double w = spec["base"]["width_mm"].get<double>();
            const double h = spec["base"]["height_mm"].get<double>();
            const double t = spec["base"]["thickness_mm"].get<double>();
            const double r = spec["base"].value("initial_corner_r_mm", 8.0);

            const TopoDS_Shape boxShape = pr::box(
                gp_Ax2(gp_Pnt(-w / 2.0, -h / 2.0, 0.0), gp::DZ()),
                w, h, t);

            if (r <= 0.0) return boxShape;
            return pr::filletEdges(boxShape, r,
                pr::verticalCornerEdges(w / 2.0, h / 2.0, t));
        });
}

// @lat: [[engine/feature-phone#스텝 2 — applyCornerRadius]]
StepResult PhoneFrontModel::applyCornerRadius(const TopoDS_Shape& in,
                                               const nlohmann::json& spec)
{
    return pr::runStep("PhoneFrontModel::applyCornerRadius",
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

// @lat: [[engine/feature-phone#스텝 3 — buildDisplayPocket]]
StepResult PhoneFrontModel::buildDisplayPocket(const TopoDS_Shape& in,
                                                const nlohmann::json& spec)
{
    if (!spec.contains("display_pocket")) return StepResult{ in, {} };

    return pr::runStep("PhoneFrontModel::buildDisplayPocket",
        [&](std::vector<BuildWarning>&) {
            const auto& dp = spec["display_pocket"];
            const double w     = dp["width_mm"].get<double>();
            const double h     = dp["height_mm"].get<double>();
            const double depth = dp["depth_mm"].get<double>();
            const double rCorn = dp.value("corner_r_mm", 0.0);
            const double offX  = dp.value("offset_x_mm", 0.0);
            const double offY  = dp.value("offset_y_mm", 0.0);

            const auto   bb     = pr::optimalBbox(in);
            // Overhang the tool slightly above zMax so the Boolean leaves a
            // clean rim instead of a coincident surface.
            const double kOverhang = 0.05;
            const gp_Pnt bottomCenter(offX, offY, bb.zMax - depth);

            const TopoDS_Shape pocket = pr::roundedRectPocketTool(
                bottomCenter, w, h, depth + kOverhang, rCorn);
            return pr::cut(in, pocket);
        });
}

// @lat: [[engine/feature-phone#스텝 4 — addCameraHoles]]
StepResult PhoneFrontModel::addCameraHoles(const TopoDS_Shape& in,
                                            const nlohmann::json& spec)
{
    if (!spec.contains("cameras") || spec["cameras"].empty())
        return StepResult{ in, {} };

    return pr::runStep("PhoneFrontModel::addCameraHoles",
        [&](std::vector<BuildWarning>&) {
            const auto bb = pr::optimalBbox(in);
            const auto& cams = spec["cameras"];

            std::vector<TopoDS_Shape> tools;
            tools.reserve(cams.size());
            for (const auto& c : cams) {
                const double x   = c["offset_x_mm"].get<double>();
                const double y   = c["offset_y_mm"].get<double>();
                const double dia = c["hole_dia_mm"].get<double>();
                const double dep = c["depth_mm"].get<double>();

                // Hole axis = +Z; start 0.05 mm below the rear face so the
                // cut clears the surface cleanly, height = depth + overhang.
                const double kOverhang = 0.05;
                const gp_Ax2 ax(gp_Pnt(x, y, bb.zMin - kOverhang), gp::DZ());
                tools.push_back(pr::cylinder(ax, dia / 2.0, dep + kOverhang));
            }
            return pr::cutMany(in, tools);
        });
}

// ── Convenience: chain all M2-phase-1 steps (1–4) ──────────────────────────
TopoDS_Shape PhoneFrontModel::buildAll(const nlohmann::json& spec,
                                        std::vector<BuildWarning>& warnings)
{
    PhoneFrontModel m;

    auto absorb = [&warnings](StepResult&& r, const char* tag) -> TopoDS_Shape {
        warnings.insert(warnings.end(),
                        std::make_move_iterator(r.warnings.begin()),
                        std::make_move_iterator(r.warnings.end()));
        if (r.shape.IsNull())
            spdlog::error("PhoneFrontModel::buildAll aborted at {}", tag);
        return r.shape;
    };

    TopoDS_Shape s;
    s = absorb(m.buildBase(spec),              "step1 buildBase");      if (s.IsNull()) return {};
    s = absorb(m.applyCornerRadius(s, spec),   "step2 cornerRad");      if (s.IsNull()) return {};
    s = absorb(m.buildDisplayPocket(s, spec),  "step3 displayPocket");  if (s.IsNull()) return {};
    s = absorb(m.addCameraHoles(s, spec),      "step4 cameraHoles");    if (s.IsNull()) return {};
    return s;
}

// ── Default spec: 76 × 160 × 8 mm modern smartphone baseline ───────────────
nlohmann::json PhoneFrontModel::defaultSpec()
{
    return nlohmann::json{
        { "schema_version", "0.1.0" },
        { "product_name",   "Default Phone 76x160" },
        { "base", {
            { "width_mm",            76.0 },
            { "height_mm",          160.0 },
            { "thickness_mm",         8.0 },
            { "initial_corner_r_mm",  8.0 }
        }},
        { "corner_radius", {
            { "r_top_mm",  1.0 },
            { "r_side_mm", 0.6 }
        }},
        { "display_pocket", {
            { "width_mm",     70.0 },
            { "height_mm",   152.0 },
            { "depth_mm",      0.6 },
            { "corner_r_mm",   6.0 },
            { "offset_x_mm",   0.0 },
            { "offset_y_mm",   0.0 }
        }},
        { "cameras", nlohmann::json::array({
            nlohmann::json{
                { "offset_x_mm", -22.0 }, { "offset_y_mm", -55.0 },
                { "hole_dia_mm",   8.0 }, { "depth_mm",      2.0 }
            },
            nlohmann::json{
                { "offset_x_mm", -22.0 }, { "offset_y_mm", -67.0 },
                { "hole_dia_mm",   8.0 }, { "depth_mm",      2.0 }
            },
            nlohmann::json{
                { "offset_x_mm", -10.0 }, { "offset_y_mm", -61.0 },
                { "hole_dia_mm",   6.0 }, { "depth_mm",      1.5 }
            }
        })}
    };
}

// ── DFM: shared product-agnostic catalog, phone profile ───────────────────
// @lat: [[engine/feature-phone#runDFM]]
DFMReport PhoneFrontModel::runDFM(const TopoDS_Shape& shape,
                                   const nlohmann::json& spec)
{
    return dfm::runProductDFM(shape, spec, dfm::phoneProfile());
}

}  // namespace koocadcam::engine
