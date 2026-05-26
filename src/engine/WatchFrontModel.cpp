#include "WatchFrontModel.hpp"
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Compound.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <cmath>
#include <exception>

namespace koocadcam::engine {

// @lat: [[engine/feature-watch#스텝 1 — buildBase]]
StepResult WatchFrontModel::buildBase(const nlohmann::json& spec)
{
    try {
        const std::string formFactor = spec.value("form_factor", "round");

        if (formFactor == "round") {
            const double diameter  = spec["base"]["diameter_mm"].get<double>();
            const double thickness = spec["base"]["thickness_mm"].get<double>();

            BRepPrimAPI_MakeCylinder maker(
                gp_Ax2(gp::Origin(), gp::DZ()),
                diameter / 2.0,
                thickness);
            maker.Build();

            spdlog::debug("WatchFrontModel::buildBase round d={} t={}", diameter, thickness);
            return StepResult{ maker.Shape(), {} };
        }
        else if (formFactor == "square") {
            // M1.2: rationalize square corner_radius & bezel paths
            // TODO M1.2: rationalize square corner_radius & bezel paths
            const double width     = spec["base"]["width_mm"].get<double>();
            const double height    = spec["base"]["height_mm"].get<double>();
            const double thickness = spec["base"]["thickness_mm"].get<double>();
            const double initR     = spec.value(
                nlohmann::json::json_pointer("/base/initial_corner_r_mm"), 2.0);

            // Box centred on origin: offset by -width/2, -height/2 so it straddles origin
            BRepPrimAPI_MakeBox boxMaker(
                gp_Pnt(-width / 2.0, -height / 2.0, 0.0),
                width, height, thickness);
            boxMaker.Build();
            TopoDS_Shape boxShape = boxMaker.Shape();

            // Initial fillet on 4 vertical edges
            BRepFilletAPI_MakeFillet fillet(boxShape);
            TopExp_Explorer exp(boxShape, TopAbs_EDGE);
            for (; exp.More(); exp.Next()) {
                const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());
                BRepAdaptor_Curve curve(edge);
                const double mid = (curve.FirstParameter() + curve.LastParameter()) / 2.0;
                const gp_Pnt midPt = curve.Value(mid);

                // Vertical edges run along Z: X and Y are near the corners,
                // Z midpoint is near thickness/2
                const bool nearMidZ = (midPt.Z() > 1e-3) && (midPt.Z() < thickness - 1e-3);
                const bool atCornerX = (std::abs(midPt.X()) > width / 2.0 - 1e-3);
                const bool atCornerY = (std::abs(midPt.Y()) > height / 2.0 - 1e-3);
                if (nearMidZ && atCornerX && atCornerY) {
                    fillet.Add(initR, edge);
                }
            }
            fillet.Build();
            if (!fillet.IsDone()) {
                throw Standard_Failure("buildBase(square): initial fillet failed");
            }

            spdlog::debug("WatchFrontModel::buildBase square {}x{} t={} r={}",
                          width, height, thickness, initR);
            return StepResult{ fillet.Shape(), {} };
        }
        else {
            throw Standard_Failure(
                ("buildBase: unknown form_factor '" + formFactor + "'").c_str());
        }
    }
    catch (const std::exception& e) {
        spdlog::error("WatchFrontModel::buildBase exception: {}", e.what());
        return StepResult{ TopoDS_Shape(), { BuildWarning{ "E_OCCT", e.what() } } };
    }
}

// @lat: [[engine/feature-watch#스텝 2 — applyCornerRadius]]
StepResult WatchFrontModel::applyCornerRadius(const TopoDS_Shape& in,
                                               const nlohmann::json& spec)
{
    try {
        const double rTop  = spec["corner_radius"]["r_top_mm"].get<double>();
        const double rSide = spec["corner_radius"]["r_side_mm"].get<double>();

        // Determine Z extents of the shape
        Bnd_Box bbox;
        BRepBndLib::Add(in, bbox);
        double xMin, yMin, zMin, xMax, yMax, zMax;
        bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

        BRepFilletAPI_MakeFillet fillet(in);
        TopExp_Explorer exp(in, TopAbs_EDGE);
        for (; exp.More(); exp.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve curve(edge);
            const double mid   = (curve.FirstParameter() + curve.LastParameter()) / 2.0;
            const double midZ  = curve.Value(mid).Z();

            if (std::abs(midZ - zMax) < 1e-3) {
                fillet.Add(rTop, edge);
            }
            else if (std::abs(midZ - zMin) < 1e-3) {
                fillet.Add(rSide, edge);
            }
        }

        fillet.Build();
        if (!fillet.IsDone()) {
            throw Standard_Failure("applyCornerRadius: fillet.Build() failed");
        }

        const TopoDS_Shape result = fillet.Shape();
        BRepCheck_Analyzer checker(result);
        if (!checker.IsValid()) {
            spdlog::warn("WatchFrontModel::applyCornerRadius: BRepCheck invalid — returning warning");
            return StepResult{ TopoDS_Shape(),
                               { BuildWarning{ "W_BREPCHECK",
                                               "applyCornerRadius produced invalid BRep" } } };
        }

        spdlog::debug("WatchFrontModel::applyCornerRadius rTop={} rSide={}", rTop, rSide);
        return StepResult{ result, {} };
    }
    catch (const std::exception& e) {
        spdlog::error("WatchFrontModel::applyCornerRadius exception: {}", e.what());
        return StepResult{ TopoDS_Shape(), { BuildWarning{ "E_OCCT", e.what() } } };
    }
}

// @lat: [[engine/feature-watch#스텝 3 — buildBezel]]
StepResult WatchFrontModel::buildBezel(const TopoDS_Shape& in,
                                        const nlohmann::json& spec)
{
    try {
        // Compute bounding box to derive maxZ and outer radius
        Bnd_Box bbox;
        BRepBndLib::Add(in, bbox);
        double xMin, yMin, zMin, xMax, yMax, zMax;
        bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

        const double xExtent  = xMax - xMin;
        const double yExtent  = yMax - yMin;
        const double outerR   = std::max(xExtent, yExtent) / 2.0;

        const double bezelWidth = spec["bezel"]["width_mm"].get<double>();
        const double bezelDepth = spec["bezel"]["depth_mm"].get<double>();
        const double taperDeg   = spec["bezel"].value("taper_deg", 0.0);

        const gp_Pnt bezelOrigin(0.0, 0.0, zMax - bezelDepth);
        const double innerR = outerR - bezelWidth;

        // Build outer tool: cone frustum if taper > 0, cylinder otherwise
        TopoDS_Shape outerTool;
        if (taperDeg > 0.0) {
            try {
                // Cone frustum: bottom at zMax-bezelDepth, top at zMax.
                // Bottom radius is narrowed by taper; top radius stays outerR.
                const double taperRad  = taperDeg * M_PI / 180.0;
                const double bottomR   = outerR - bezelDepth * std::tan(taperRad);
                if (bottomR <= innerR) {
                    throw Standard_Failure("buildBezel: taper makes bottom radius <= innerR");
                }
                BRepPrimAPI_MakeCone coneMaker(
                    gp_Ax2(bezelOrigin, gp::DZ()),
                    bottomR,   // r1 at bottom (zMax - depth)
                    outerR,    // r2 at top (zMax)
                    bezelDepth);
                coneMaker.Build();
                if (!coneMaker.IsDone()) {
                    throw Standard_Failure("buildBezel: cone frustum build failed");
                }
                outerTool = coneMaker.Shape();
                spdlog::debug("WatchFrontModel::buildBezel tapered: taper_deg={} bottomR={}",
                              taperDeg, bottomR);
            }
            catch (const std::exception& ex) {
                spdlog::warn("WatchFrontModel::buildBezel: taper cone failed ({}); "
                             "falling back to straight walls", ex.what());
                BRepPrimAPI_MakeCylinder cylMaker(
                    gp_Ax2(bezelOrigin, gp::DZ()), outerR, bezelDepth);
                cylMaker.Build();
                outerTool = cylMaker.Shape();
            }
        }
        else {
            BRepPrimAPI_MakeCylinder cylMaker(
                gp_Ax2(bezelOrigin, gp::DZ()), outerR, bezelDepth);
            cylMaker.Build();
            outerTool = cylMaker.Shape();
        }

        // Inner cylinder (removed to form the annular pocket)
        BRepPrimAPI_MakeCylinder innerMaker(
            gp_Ax2(bezelOrigin, gp::DZ()),
            innerR,
            bezelDepth);
        innerMaker.Build();

        // Annular tool = outer − inner
        BRepAlgoAPI_Cut annularTool(outerTool, innerMaker.Shape());
        annularTool.Build();
        if (!annularTool.IsDone()) {
            throw Standard_Failure("buildBezel: annular tool cut failed");
        }

        // Subtract annular tool from the body
        BRepAlgoAPI_Cut result(in, annularTool.Shape());
        result.Build();
        if (!result.IsDone()) {
            throw Standard_Failure("buildBezel: body cut failed");
        }

        spdlog::debug("WatchFrontModel::buildBezel w={} d={} outerR={} taper={}",
                      bezelWidth, bezelDepth, outerR, taperDeg);
        return StepResult{ result.Shape(), {} };
    }
    catch (const std::exception& e) {
        spdlog::error("WatchFrontModel::buildBezel exception: {}", e.what());
        return StepResult{ TopoDS_Shape(), { BuildWarning{ "E_OCCT", e.what() } } };
    }
}

// @lat: [[engine/feature-watch#스텝 4 — buildDisplayPocket]]
StepResult WatchFrontModel::buildDisplayPocket(const TopoDS_Shape& in,
                                                const nlohmann::json& spec)
{
    if (!spec.contains("display_pocket")) {
        return StepResult{ in, {} };  // pass-through
    }

    try {
        const auto& dp      = spec["display_pocket"];
        const double dPocket      = dp["d_pocket_mm"].get<double>();
        const double depthPocket  = dp["depth_pocket_mm"].get<double>();
        const double glassOffset  = dp.value("glass_offset_mm", 0.0);

        // Determine top Z via optimal bbox
        Bnd_Box bbox;
        BRepBndLib::AddOptimal(in, bbox);
        double xMin, yMin, zMin, xMax, yMax, zMax;
        bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

        // zBottom: pocket starts below topZ by depth + recessed amount (if glass_offset < 0)
        const double recessAmt = std::max(0.0, -glassOffset);
        const double zBottom   = zMax - depthPocket - recessAmt;
        const double height    = depthPocket + std::abs(glassOffset);

        BRepPrimAPI_MakeCylinder pocketCyl(
            gp_Ax2(gp_Pnt(0.0, 0.0, zBottom), gp::DZ()),
            dPocket / 2.0,
            height);
        pocketCyl.Build();
        if (!pocketCyl.IsDone()) {
            throw Standard_Failure("buildDisplayPocket: cylinder build failed");
        }

        BRepAlgoAPI_Cut cut(in, pocketCyl.Shape());
        cut.Build();
        if (!cut.IsDone()) {
            throw Standard_Failure("buildDisplayPocket: cut failed");
        }

        const TopoDS_Shape result = cut.Shape();
        BRepCheck_Analyzer checker(result);
        if (!checker.IsValid()) {
            spdlog::warn("WatchFrontModel::buildDisplayPocket: BRepCheck invalid");
            return StepResult{ TopoDS_Shape(),
                               { BuildWarning{ "W_BREPCHECK",
                                               "buildDisplayPocket produced invalid BRep" } } };
        }

        spdlog::debug("WatchFrontModel::buildDisplayPocket d={} depth={} glassOff={}",
                      dPocket, depthPocket, glassOffset);
        return StepResult{ result, {} };
    }
    catch (const std::exception& e) {
        spdlog::error("WatchFrontModel::buildDisplayPocket exception: {}", e.what());
        return StepResult{ TopoDS_Shape(), { BuildWarning{ "E_OCCT", e.what() } } };
    }
}

// @lat: [[engine/feature-watch#스텝 5 — addCrownCavity]]
StepResult WatchFrontModel::addCrownCavity(const TopoDS_Shape& in,
                                            const nlohmann::json& spec)
{
    if (!spec.contains("crown_cavity")) {
        return StepResult{ in, {} };  // pass-through
    }

    try {
        const auto& cc        = spec["crown_cavity"];
        const double angleDeg = cc["side_angle_deg"].get<double>();
        const double heightZ  = cc["height_pos_mm"].get<double>();
        const double depth    = cc["depth_mm"].get<double>();
        const double diameter = cc["diameter_mm"].get<double>();
        const double shaftDia = cc["shaft_dia_mm"].get<double>();

        // Derive case outer radius from bbox
        Bnd_Box bbox;
        BRepBndLib::AddOptimal(in, bbox);
        double xMin, yMin, zMin, xMax, yMax, zMax;
        bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);
        const double R = std::max(xMax - xMin, yMax - yMin) / 2.0;

        const double angleRad = angleDeg * M_PI / 180.0;
        const double cosA     = std::cos(angleRad);
        const double sinA     = std::sin(angleRad);

        // Center on side surface
        gp_Pnt center(R * cosA, R * sinA, heightZ);
        // Inward radial direction (into the case)
        gp_Dir inward(-cosA, -sinA, 0.0);

        // Cavity cylinder: starts at side surface, extends inward by depth
        BRepPrimAPI_MakeCylinder cavityCyl(
            gp_Ax2(center, inward),
            diameter / 2.0,
            depth);
        cavityCyl.Build();
        if (!cavityCyl.IsDone()) {
            throw Standard_Failure("addCrownCavity: cavity cylinder build failed");
        }

        BRepAlgoAPI_Cut cut1(in, cavityCyl.Shape());
        cut1.Build();
        if (!cut1.IsDone()) {
            throw Standard_Failure("addCrownCavity: cavity cut failed");
        }

        BRepCheck_Analyzer check1(cut1.Shape());
        if (!check1.IsValid()) {
            spdlog::warn("WatchFrontModel::addCrownCavity: BRepCheck invalid after cavity cut");
            return StepResult{ TopoDS_Shape(),
                               { BuildWarning{ "W_BREPCHECK",
                                               "addCrownCavity cavity cut produced invalid BRep" } } };
        }

        // Shaft hole: same axis inward, through full case diameter + 1 mm margin
        const double shaftHeight = R + 1.0;
        BRepPrimAPI_MakeCylinder shaftCyl(
            gp_Ax2(center, inward),
            shaftDia / 2.0,
            shaftHeight);
        shaftCyl.Build();
        if (!shaftCyl.IsDone()) {
            throw Standard_Failure("addCrownCavity: shaft cylinder build failed");
        }

        BRepAlgoAPI_Cut cut2(cut1.Shape(), shaftCyl.Shape());
        cut2.Build();
        if (!cut2.IsDone()) {
            throw Standard_Failure("addCrownCavity: shaft cut failed");
        }

        BRepCheck_Analyzer check2(cut2.Shape());
        if (!check2.IsValid()) {
            spdlog::warn("WatchFrontModel::addCrownCavity: BRepCheck invalid after shaft cut");
            return StepResult{ TopoDS_Shape(),
                               { BuildWarning{ "W_BREPCHECK",
                                               "addCrownCavity shaft cut produced invalid BRep" } } };
        }

        spdlog::debug("WatchFrontModel::addCrownCavity angle={} h={} depth={} dia={} shaftDia={}",
                      angleDeg, heightZ, depth, diameter, shaftDia);
        return StepResult{ cut2.Shape(), {} };
    }
    catch (const std::exception& e) {
        spdlog::error("WatchFrontModel::addCrownCavity exception: {}", e.what());
        return StepResult{ TopoDS_Shape(), { BuildWarning{ "E_OCCT", e.what() } } };
    }
}

// @lat: [[engine/feature-watch#스텝 6 — addSideButtons]]
StepResult WatchFrontModel::addSideButtons(const TopoDS_Shape& in,
                                            const nlohmann::json& spec)
{
    if (!spec.contains("side_buttons") || spec["side_buttons"].empty()) {
        return StepResult{ in, {} };  // pass-through
    }

    try {
        const auto& buttons = spec["side_buttons"];

        // Derive case outer radius from bbox
        Bnd_Box bbox;
        BRepBndLib::AddOptimal(in, bbox);
        double xMin, yMin, zMin, xMax, yMax, zMax;
        bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);
        const double R = std::max(xMax - xMin, yMax - yMin) / 2.0;

        // Collect all button pockets into a compound for single cut
        TopoDS_Compound compound;
        BRep_Builder builder;
        builder.MakeCompound(compound);

        std::vector<BuildWarning> warnings;
        int count = 0;

        for (const auto& btn : buttons) {
            const double angleDeg = btn["angle_deg"].get<double>();
            const double heightMm = btn["height_mm"].get<double>();
            const double lengthMm = btn["length_mm"].get<double>();
            const double widthMm  = btn["width_mm"].get<double>();
            const double depthMm  = btn["depth_mm"].get<double>();
            const double taperDeg = btn.value("taper_deg", 0.0);

            if (taperDeg > 0.0) {
                warnings.push_back(BuildWarning{
                    "W_TAPER_M1_5",
                    "side_buttons taper_deg > 0 not implemented (M1.5); treated as 0"
                });
                spdlog::warn("WatchFrontModel::addSideButtons: taper_deg={} not yet "
                             "implemented (M1.5), treating as 0", taperDeg);
            }

            const double angleRad = angleDeg * M_PI / 180.0;
            const double cosA     = std::cos(angleRad);
            const double sinA     = std::sin(angleRad);

            // Local frame on case side surface
            // x_loc = inward radial (depth direction into case)
            gp_Dir xLoc(-cosA, -sinA, 0.0);
            // y_loc = tangent CCW (length direction)
            gp_Dir yLoc(-sinA,  cosA, 0.0);
            // z_loc = Z+ (width direction)
            gp_Dir zLoc(0.0, 0.0, 1.0);

            // Center on side surface
            gp_Pnt center(R * cosA, R * sinA, heightMm);

            // Shift origin so box is centred in length (y_loc) and width (z_loc)
            gp_Pnt origin(
                center.X() - (lengthMm / 2.0) * yLoc.X() - (widthMm / 2.0) * zLoc.X(),
                center.Y() - (lengthMm / 2.0) * yLoc.Y() - (widthMm / 2.0) * zLoc.Y(),
                center.Z() - (lengthMm / 2.0) * yLoc.Z() - (widthMm / 2.0) * zLoc.Z()
            );

            gp_Ax2 ax(origin, xLoc, yLoc);
            BRepPrimAPI_MakeBox boxMaker(ax, depthMm, lengthMm, widthMm);
            boxMaker.Build();
            if (!boxMaker.IsDone()) {
                spdlog::warn("WatchFrontModel::addSideButtons: box build failed for button {}",
                             count);
                continue;
            }

            builder.Add(compound, boxMaker.Shape());
            ++count;
        }

        if (count == 0) {
            spdlog::warn("WatchFrontModel::addSideButtons: no button pockets built");
            return StepResult{ in, warnings };
        }

        BRepAlgoAPI_Cut cut(in, compound);
        cut.Build();
        if (!cut.IsDone()) {
            throw Standard_Failure("addSideButtons: compound cut failed");
        }

        const TopoDS_Shape result = cut.Shape();
        BRepCheck_Analyzer checker(result);
        if (!checker.IsValid()) {
            spdlog::warn("WatchFrontModel::addSideButtons: BRepCheck invalid");
            return StepResult{ TopoDS_Shape(),
                               { BuildWarning{ "W_BREPCHECK",
                                               "addSideButtons produced invalid BRep" } } };
        }

        spdlog::debug("WatchFrontModel::addSideButtons: {} button(s) processed", count);
        return StepResult{ result, warnings };
    }
    catch (const std::exception& e) {
        spdlog::error("WatchFrontModel::addSideButtons exception: {}", e.what());
        return StepResult{ TopoDS_Shape(), { BuildWarning{ "E_OCCT", e.what() } } };
    }
}

// ── Convenience: chain all M1.2 steps (1–6)
TopoDS_Shape WatchFrontModel::buildAll(const nlohmann::json& spec,
                                        std::vector<BuildWarning>& warnings)
{
    auto bboxOf = [](const TopoDS_Shape& s) {
        if (s.IsNull()) return std::string("<null>");
        Bnd_Box b; BRepBndLib::Add(s, b);
        if (b.IsVoid()) return std::string("<void>");
        double x0,y0,z0,x1,y1,z1; b.Get(x0,y0,z0,x1,y1,z1);
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "[%.3f..%.3f]x[%.3f..%.3f]x[%.3f..%.3f]  dx=%.3f dy=%.3f dz=%.3f",
                      x0,x1,y0,y1,z0,z1, x1-x0, y1-y0, z1-z0);
        return std::string(buf);
    };

    WatchFrontModel model;

    StepResult s1 = model.buildBase(spec);
    warnings.insert(warnings.end(), s1.warnings.begin(), s1.warnings.end());
    if (s1.shape.IsNull()) {
        spdlog::error("WatchFrontModel::buildAll aborted at step 1");
        return TopoDS_Shape{};
    }
    spdlog::info("WatchFrontModel::buildAll step1 buildBase   bbox: {}", bboxOf(s1.shape));

    StepResult s2 = model.applyCornerRadius(s1.shape, spec);
    warnings.insert(warnings.end(), s2.warnings.begin(), s2.warnings.end());
    if (s2.shape.IsNull()) {
        spdlog::error("WatchFrontModel::buildAll aborted at step 2");
        return TopoDS_Shape{};
    }
    spdlog::info("WatchFrontModel::buildAll step2 cornerRad   bbox: {}", bboxOf(s2.shape));

    StepResult s3 = model.buildBezel(s2.shape, spec);
    warnings.insert(warnings.end(), s3.warnings.begin(), s3.warnings.end());
    if (s3.shape.IsNull()) {
        spdlog::error("WatchFrontModel::buildAll aborted at step 3");
        return TopoDS_Shape{};
    }
    spdlog::info("WatchFrontModel::buildAll step3 buildBezel  bbox: {}", bboxOf(s3.shape));

    StepResult s4 = model.buildDisplayPocket(s3.shape, spec);
    warnings.insert(warnings.end(), s4.warnings.begin(), s4.warnings.end());
    if (s4.shape.IsNull()) {
        spdlog::error("WatchFrontModel::buildAll aborted at step 4");
        return TopoDS_Shape{};
    }
    spdlog::info("WatchFrontModel::buildAll step4 displayPkt  bbox: {}", bboxOf(s4.shape));

    StepResult s5 = model.addCrownCavity(s4.shape, spec);
    warnings.insert(warnings.end(), s5.warnings.begin(), s5.warnings.end());
    if (s5.shape.IsNull()) {
        spdlog::error("WatchFrontModel::buildAll aborted at step 5");
        return TopoDS_Shape{};
    }
    spdlog::info("WatchFrontModel::buildAll step5 crownCav    bbox: {}", bboxOf(s5.shape));

    StepResult s6 = model.addSideButtons(s5.shape, spec);
    warnings.insert(warnings.end(), s6.warnings.begin(), s6.warnings.end());
    if (s6.shape.IsNull()) {
        spdlog::error("WatchFrontModel::buildAll aborted at step 6");
        return TopoDS_Shape{};
    }
    spdlog::info("WatchFrontModel::buildAll step6 sideButtons bbox: {}", bboxOf(s6.shape));

    return s6.shape;
}

// ── Default spec: 44 mm round watch baseline
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
