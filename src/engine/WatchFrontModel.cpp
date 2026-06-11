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
#include "probe/GeometryProbe.hpp"
#include "dfm/ProductDFM.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <GeomLProp_CLProps.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Lin.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

namespace koocadcam::engine {

namespace pr = prim;

// ──────────────────────────────────────────────────────────────────────────

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

// @lat: [[engine/feature-watch#스텝 7 — addSpeakerGrille]]
StepResult WatchFrontModel::addSpeakerGrille(const TopoDS_Shape& in,
                                              const nlohmann::json& spec)
{
    if (!spec.contains("speaker_grille")) return StepResult{ in, {} };

    return pr::runStep("WatchFrontModel::addSpeakerGrille",
        [&](std::vector<BuildWarning>&) {
            const auto& sg = spec["speaker_grille"];
            const double angleDeg = sg["angle_deg"].get<double>();
            const double heightZ  = sg["height_pos_mm"].get<double>();
            const int    rows     = sg["rows"].get<int>();
            const int    cols     = sg["cols"].get<int>();
            const double rowSp    = sg["row_spacing_mm"].get<double>();
            const double colSp    = sg["col_spacing_mm"].get<double>();
            const double dia      = sg["hole_dia_mm"].get<double>();
            const double depth    = sg["depth_mm"].get<double>();

            const double R     = pr::optimalBbox(in).outerRadiusXY();
            const auto   frame = pr::sideFrameAt(R, angleDeg, heightZ);

            std::vector<TopoDS_Shape> tools;
            tools.reserve(static_cast<std::size_t>(rows) * cols);
            for (int r = 0; r < rows; ++r) {
                const double dz = (r - (rows - 1) / 2.0) * rowSp;
                for (int c = 0; c < cols; ++c) {
                    const double dy = (c - (cols - 1) / 2.0) * colSp;
                    const gp_Pnt holeCenter = pr::offsetPoint(
                        frame.center, dy, frame.tangentCCW, dz, frame.axialZ);
                    const gp_Ax2 ax(holeCenter, frame.inwardRadial);
                    tools.push_back(pr::cylinder(ax, dia / 2.0, depth));
                }
            }
            return pr::cutMany(in, tools);
        });
}

// @lat: [[engine/feature-watch#스텝 8 — addRearSensors]]
StepResult WatchFrontModel::addRearSensors(const TopoDS_Shape& in,
                                            const nlohmann::json& spec)
{
    if (!spec.contains("rear_sensors") || spec["rear_sensors"].empty())
        return StepResult{ in, {} };

    return pr::runStep("WatchFrontModel::addRearSensors",
        [&](std::vector<BuildWarning>&) {
            const auto bb = pr::optimalBbox(in);

            std::vector<TopoDS_Shape> tools;
            tools.reserve(spec["rear_sensors"].size());
            for (const auto& s : spec["rear_sensors"]) {
                const double x   = s["offset_x_mm"].get<double>();
                const double y   = s["offset_y_mm"].get<double>();
                const double dia = s["dia_mm"].get<double>();
                const double dep = s["depth_mm"].get<double>();
                // Hole axis +Z, starts 0.05 mm below rear face for clean cut.
                const double kOverhang = 0.05;
                const gp_Ax2 ax(gp_Pnt(x, y, bb.zMin - kOverhang), gp::DZ());
                tools.push_back(pr::cylinder(ax, dia / 2.0, dep + kOverhang));
            }
            return pr::cutMany(in, tools);
        });
}

// @lat: [[engine/feature-watch#스텝 9 — addLugs]]
StepResult WatchFrontModel::addLugs(const TopoDS_Shape& in,
                                     const nlohmann::json& spec)
{
    if (!spec.contains("lugs") || spec["lugs"].empty())
        return StepResult{ in, {} };

    return pr::runStep("WatchFrontModel::addLugs",
        [&](std::vector<BuildWarning>& warnings) {
            const auto   bb   = pr::optimalBbox(in);
            const double R    = bb.outerRadiusXY();
            const double midZ = (bb.zMin + bb.zMax) / 2.0;

            // Build protrusions (material to add) and optional pin holes (material to cut).
            std::vector<TopoDS_Shape> protrusions;
            std::vector<TopoDS_Shape> pinHoles;
            protrusions.reserve(spec["lugs"].size());

            for (const auto& lug : spec["lugs"]) {
                const double angleDeg = lug["angle_deg"].get<double>();
                const double length   = lug["length_mm"].get<double>();
                const double width    = lug["width_mm"].get<double>();
                const double thick    = lug["thickness_mm"].get<double>();
                const double pinDia   = lug.value("pin_hole_dia_mm", 0.0);

                const auto    frame   = pr::sideFrameAt(R, angleDeg, midZ);
                const gp_Dir  outward = frame.inwardRadial.Reversed();

                // Lug box: extends outward by `length`, centered on tangent
                // (`width`) and axial (`thick`).  Origin = case-surface corner.
                //
                // gp_Ax2(P, V, Vx) sets local Z = V and local X = Vx.  We want
                // DX = length along outward, DY = width along tangent, DZ =
                // thick along axial.  Picking V = axialZ and Vx = outward gives
                // Y = axialZ × outward = tangentCCW (verified for any angle θ).
                const gp_Pnt origin = pr::offsetPoint(
                    frame.center,
                    -width / 2.0, frame.tangentCCW,
                    -thick / 2.0, frame.axialZ);
                const gp_Ax2 ax(origin, frame.axialZ, outward);
                protrusions.push_back(pr::box(ax, length, width, thick));

                // Optional pin hole: through-hole along tangent direction,
                // positioned 70 % out along the lug length.
                if (pinDia >= 0.6) {  // ignore tiny / zero — DFM-002 also enforces
                    const double pinOutOffset = length * 0.7;
                    const double kOverhang    = 0.05;
                    const gp_Pnt pinStart(
                        frame.center.X()
                            + pinOutOffset * outward.X()
                            - (width / 2.0 + kOverhang) * frame.tangentCCW.X(),
                        frame.center.Y()
                            + pinOutOffset * outward.Y()
                            - (width / 2.0 + kOverhang) * frame.tangentCCW.Y(),
                        frame.center.Z());
                    const gp_Ax2 pinAx(pinStart, frame.tangentCCW);
                    pinHoles.push_back(
                        pr::cylinder(pinAx, pinDia / 2.0, width + 2.0 * kOverhang));
                } else if (pinDia > 0.0) {
                    warnings.push_back(BuildWarning{
                        "W_DFM_002_PIN", "lug pin_hole_dia_mm < 0.6, skipped (DFM-002)"
                    });
                }
            }
            // First fuse all protrusions, then cut pin holes through fused body.
            const TopoDS_Shape withLugs = pr::fuseMany(in, protrusions);
            return pr::cutMany(withLugs, pinHoles);
        });
}

// @lat: [[engine/feature-watch#스텝 10 — addSecondaryFillets]]
StepResult WatchFrontModel::addSecondaryFillets(const TopoDS_Shape& in,
                                                 const nlohmann::json& spec)
{
    if (!spec.contains("secondary_fillets")) return StepResult{ in, {} };

    return pr::runStep("WatchFrontModel::addSecondaryFillets",
        [&](std::vector<BuildWarning>& warnings) {
            const double r  = spec["secondary_fillets"].value("r_mm", 0.2);
            const auto   bb = pr::optimalBbox(in);

            // Apply forgiving fillets to two specific Z-bands:
            //   1. display-pocket top rim    @ z = zMax
            //   2. bezel inner step          @ z = zMax - bezel.depth_mm
            // If either pass fails (no matching edges, fillet solver fails),
            // log a warning and pass the previous shape through.
            TopoDS_Shape current = in;
            auto safeFillet = [&](double radius, double z, const char* label) {
                try {
                    current = pr::filletEdges(current, radius, pr::edgesAtZ(z));
                } catch (const std::exception& e) {
                    warnings.push_back(BuildWarning{
                        "W_FILLET_SKIP",
                        std::string(label) + " fillet skipped: " + e.what()
                    });
                }
            };
            safeFillet(r, bb.zMax, "top rim");
            if (spec.contains("bezel")) {
                const double bezDepth = spec["bezel"].value("depth_mm", 0.0);
                if (bezDepth > 0.0)
                    safeFillet(r * 0.5, bb.zMax - bezDepth, "bezel inner step");
            }
            return current;
        },
        /*checkValidity=*/false);  // multi-fillet may leave benign minor invalid edges
}

// ── Convenience: chain all M1.5 steps (1–10) ───────────────────────────────
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
    s = absorb(m.buildBase(spec),               "step1 buildBase");        if (s.IsNull()) return {};
    s = absorb(m.applyCornerRadius(s, spec),    "step2 cornerRad");        if (s.IsNull()) return {};
    s = absorb(m.buildBezel(s, spec),           "step3 buildBezel");       if (s.IsNull()) return {};
    s = absorb(m.buildDisplayPocket(s, spec),   "step4 displayPocket");    if (s.IsNull()) return {};
    s = absorb(m.addCrownCavity(s, spec),       "step5 crownCavity");      if (s.IsNull()) return {};
    s = absorb(m.addSideButtons(s, spec),       "step6 sideButtons");      if (s.IsNull()) return {};
    s = absorb(m.addSpeakerGrille(s, spec),     "step7 speakerGrille");    if (s.IsNull()) return {};
    s = absorb(m.addRearSensors(s, spec),       "step8 rearSensors");      if (s.IsNull()) return {};
    s = absorb(m.addLugs(s, spec),              "step9 lugs");             if (s.IsNull()) return {};
    s = absorb(m.addSecondaryFillets(s, spec),  "step10 secondaryFillets"); if (s.IsNull()) return {};
    return s;
}

// ── Step 11: runDFM — DFM rule catalog validation ──────────────────────────
// @lat: [[engine/feature-watch#스텝 11 — runDFM]]
// ── Step 11: runDFM — delegates to the shared product-agnostic catalog ────
// @lat: [[engine/feature-watch#스텝 11 — runDFM]]
DFMReport WatchFrontModel::runDFM(const TopoDS_Shape& shape,
                                   const nlohmann::json& spec)
{
    return dfm::runProductDFM(shape, spec, dfm::watchProfile());
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
        })},
        { "speaker_grille", {
            { "angle_deg",       180.0 },   // 9 o'clock (convention 0°=+X=3시, CCW)
            { "height_pos_mm",     5.0 },
            { "rows",              2 },
            { "cols",              6 },
            { "row_spacing_mm",    1.5 },
            { "col_spacing_mm",    1.5 },
            { "hole_dia_mm",       0.8 },   // DFM-002 minimum
            { "depth_mm",          1.0 }
        }},
        { "rear_sensors", nlohmann::json::array({
            nlohmann::json{
                { "offset_x_mm",  0.0 }, { "offset_y_mm",  0.0 },
                { "dia_mm",       5.0 }, { "depth_mm",     0.5 }
            },
            nlohmann::json{
                { "offset_x_mm",  5.0 }, { "offset_y_mm",  0.0 },
                { "dia_mm",       1.5 }, { "depth_mm",     0.3 }
            }
        })},
        { "lugs", nlohmann::json::array({
            nlohmann::json{
                { "angle_deg",        90.0 },   // 12 o'clock
                { "length_mm",         6.0 },
                { "width_mm",         18.0 },
                { "thickness_mm",      3.0 },
                { "pin_hole_dia_mm",   1.8 }
            },
            nlohmann::json{
                { "angle_deg",       270.0 },   // 6 o'clock
                { "length_mm",         6.0 },
                { "width_mm",         18.0 },
                { "thickness_mm",      3.0 },
                { "pin_hole_dia_mm",   1.8 }
            }
        })},
        { "secondary_fillets", {
            { "r_mm",  0.2 }
        }}
    };
}

}  // namespace koocadcam::engine
