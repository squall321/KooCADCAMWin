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

#include "io/SpecExpr.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
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

// @lat: [[engine/feature-phone#스텝 5 — addSideButtons]]
// Rectangular button pockets on the flat ±X side faces (volume / power).  The
// phone's sides are planar, so — unlike the round watch — a box cutter is
// positioned against the side face directly (no radial frame).
StepResult PhoneFrontModel::addSideButtons(const TopoDS_Shape& in,
                                            const nlohmann::json& spec)
{
    if (!spec.contains("side_buttons") || spec["side_buttons"].empty())
        return StepResult{ in, {} };

    return pr::runStep("PhoneFrontModel::addSideButtons",
        [&](std::vector<BuildWarning>& warnings) {
            const auto bb = pr::optimalBbox(in);
            const double kOverhang = 0.05;
            const auto& buttons = spec["side_buttons"];

            std::vector<TopoDS_Shape> tools;
            tools.reserve(buttons.size());
            for (const auto& btn : buttons) {
                const std::string side = btn.value("side", std::string("right"));
                const double cy    = btn["center_y_mm"].get<double>();
                const double cz    = btn["center_z_mm"].get<double>();
                const double len   = btn["length_mm"].get<double>();   // along Y
                const double hgt   = btn["height_mm"].get<double>();   // along Z
                const double depth = btn["depth_mm"].get<double>();    // into X

                if (depth >= (bb.xMax - bb.xMin) * 0.5) {
                    warnings.push_back(BuildWarning{
                        "W_BUTTON_DEPTH",
                        "side button depth >= half the body width; clamped by DFM"
                    });
                }
                // Cutter spans `depth` inward from the chosen face.
                const bool right = (side != "left");
                const double x0 = right ? (bb.xMax - depth) : (bb.xMin - kOverhang);
                const gp_Ax2 corner(gp_Pnt(x0, cy - len / 2.0, cz - hgt / 2.0),
                                    gp::DZ());
                tools.push_back(pr::box(corner, depth + kOverhang, len, hgt));
            }
            return pr::cutMany(in, tools);
        });
}

// @lat: [[engine/feature-phone#스텝 6 — addPortHole]]
// USB-C port: an obround (stadium) cut-out on the bottom (−Y) edge, extruded
// into +Y.  Composed as a box flanked by two end cylinders (fuseMany), then a
// single Boolean cut — the same "compound tool, one cut" idiom as the cameras.
StepResult PhoneFrontModel::addPortHole(const TopoDS_Shape& in,
                                        const nlohmann::json& spec)
{
    if (!spec.contains("port_hole")) return StepResult{ in, {} };

    return pr::runStep("PhoneFrontModel::addPortHole",
        [&](std::vector<BuildWarning>&) {
            const auto& ph = spec["port_hole"];
            const double w     = ph["width_mm"].get<double>();    // along X
            const double h     = ph["height_mm"].get<double>();   // along Z
            const double depth = ph["depth_mm"].get<double>();    // into +Y
            const double cx    = ph.value("center_x_mm", 0.0);
            const double cz    = ph.value("center_z_mm", 0.0);

            const auto   bb    = pr::optimalBbox(in);
            const double kOverhang = 0.05;
            const double y0    = bb.yMin - kOverhang;             // bottom face
            const double dy    = depth + kOverhang;
            const double rEnd  = h / 2.0;
            const double midLen = w - h;                          // straight span

            // Obround = central box + two semicircular ends (full cylinders,
            // axis +Y).  Falls back to a single cylinder if not wider than tall.
            TopoDS_Shape tool;
            if (midLen > 1e-3) {
                const gp_Ax2 corner(gp_Pnt(cx - midLen / 2.0, y0, cz - rEnd),
                                    gp::DZ());
                const TopoDS_Shape boxPart = pr::box(corner, midLen, dy, h);
                const TopoDS_Shape capL = pr::cylinder(
                    gp_Ax2(gp_Pnt(cx - midLen / 2.0, y0, cz), gp::DY()), rEnd, dy);
                const TopoDS_Shape capR = pr::cylinder(
                    gp_Ax2(gp_Pnt(cx + midLen / 2.0, y0, cz), gp::DY()), rEnd, dy);
                tool = pr::fuseMany(boxPart, { capL, capR });
            } else {
                tool = pr::cylinder(
                    gp_Ax2(gp_Pnt(cx, y0, cz), gp::DY()), std::min(w, h) / 2.0, dy);
            }
            return pr::cut(in, tool);
        });
}

// @lat: [[engine/feature-phone#스텝 7 — addCameraDecoRings]]
// Recessed annular rings around the rear camera lenses.  Each ring tool is a
// tube (outer cylinder minus inner cylinder); cutting it leaves the lens
// surround raised and a decorative groove around it.
StepResult PhoneFrontModel::addCameraDecoRings(const TopoDS_Shape& in,
                                               const nlohmann::json& spec)
{
    if (!spec.contains("camera_deco_rings") || spec["camera_deco_rings"].empty())
        return StepResult{ in, {} };

    return pr::runStep("PhoneFrontModel::addCameraDecoRings",
        [&](std::vector<BuildWarning>& warnings) {
            const auto bb = pr::optimalBbox(in);
            const double kOverhang = 0.05;
            const auto& rings = spec["camera_deco_rings"];

            std::vector<TopoDS_Shape> tools;
            tools.reserve(rings.size());
            for (const auto& r : rings) {
                const double x     = r["offset_x_mm"].get<double>();
                const double y     = r["offset_y_mm"].get<double>();
                const double outer = r["outer_dia_mm"].get<double>();
                const double inner = r["inner_dia_mm"].get<double>();
                const double depth = r["depth_mm"].get<double>();
                if (inner >= outer) {
                    warnings.push_back(BuildWarning{
                        "W_DECO_RING",
                        "deco ring inner_dia >= outer_dia; ring skipped"
                    });
                    continue;
                }
                // Outer (the groove envelope) and a taller inner punch so the
                // tube is clean through the groove depth.
                const gp_Ax2 axOuter(gp_Pnt(x, y, bb.zMin - kOverhang), gp::DZ());
                const gp_Ax2 axInner(gp_Pnt(x, y, bb.zMin - kOverhang - 1.0), gp::DZ());
                const TopoDS_Shape outerCyl = pr::cylinder(axOuter, outer / 2.0,
                                                           depth + kOverhang);
                const TopoDS_Shape innerCyl = pr::cylinder(axInner, inner / 2.0,
                                                           depth + kOverhang + 2.0);
                tools.push_back(pr::cut(outerCyl, innerCyl));   // annular tube
            }
            if (tools.empty()) return in;
            return pr::cutMany(in, tools);
        });
}

// ── Convenience: chain all steps (1–7) ─────────────────────────────────────
TopoDS_Shape PhoneFrontModel::buildAll(const nlohmann::json& specIn,
                                        std::vector<BuildWarning>& warnings)
{
    // Driven-dimension pre-pass (see io/SpecExpr.hpp): resolve "=expr" fields to
    // numbers before the steps read them; a spec with none is unchanged.
    nlohmann::json spec = specIn;
    std::string exErr;
    if (!io::resolveExpressions(spec, exErr)) {
        warnings.push_back(BuildWarning{ "E_SPEC_EXPR", exErr });
        return {};
    }

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
    s = absorb(m.addSideButtons(s, spec),      "step5 sideButtons");    if (s.IsNull()) return {};
    s = absorb(m.addPortHole(s, spec),         "step6 portHole");       if (s.IsNull()) return {};
    s = absorb(m.addCameraDecoRings(s, spec),  "step7 decoRings");      if (s.IsNull()) return {};
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
        })},
        // Step 5 — power button (right) + volume rocker (left).
        { "side_buttons", nlohmann::json::array({
            nlohmann::json{
                { "side", "right" }, { "center_y_mm", 30.0 }, { "center_z_mm", 4.0 },
                { "length_mm", 12.0 }, { "height_mm", 3.0 }, { "depth_mm", 0.8 }
            },
            nlohmann::json{
                { "side", "left" }, { "center_y_mm", 38.0 }, { "center_z_mm", 4.0 },
                { "length_mm", 22.0 }, { "height_mm", 3.0 }, { "depth_mm", 0.8 }
            }
        })},
        // Step 6 — USB-C port, centred on the bottom edge.
        { "port_hole", {
            { "width_mm",     8.6 }, { "height_mm",   3.2 },
            { "depth_mm",     6.0 }, { "center_x_mm", 0.0 }, { "center_z_mm", 4.0 }
        }},
        // Step 7 — decorative rings around the two main rear cameras.  Sized so
        // the ring-to-ring land (2 mm) and both annular lands (0.5 mm) stay
        // above the phone DFM minimum wall (0.40 mm).
        { "camera_deco_rings", nlohmann::json::array({
            nlohmann::json{
                { "offset_x_mm", -22.0 }, { "offset_y_mm", -55.0 },
                { "outer_dia_mm", 10.0 }, { "inner_dia_mm", 9.0 }, { "depth_mm", 0.3 }
            },
            nlohmann::json{
                { "offset_x_mm", -22.0 }, { "offset_y_mm", -67.0 },
                { "outer_dia_mm", 10.0 }, { "inner_dia_mm", 9.0 }, { "depth_mm", 0.3 }
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
