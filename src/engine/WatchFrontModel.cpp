#include "WatchFrontModel.hpp"
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
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

        if (taperDeg > 0.0) {
            // TODO M1.2: ThruSections taper — falling back to 0-taper for M1.1
            spdlog::warn("WatchFrontModel::buildBezel: taper_deg={} > 0 not yet implemented "
                         "(M1.2); falling back to straight walls", taperDeg);
        }

        const gp_Pnt bezelOrigin(0.0, 0.0, zMax - bezelDepth);

        // Outer annular cylinder
        BRepPrimAPI_MakeCylinder outerMaker(
            gp_Ax2(bezelOrigin, gp::DZ()),
            outerR,
            bezelDepth);
        outerMaker.Build();

        // Inner cylinder (removed to form the annular pocket)
        const double innerR = outerR - bezelWidth;
        BRepPrimAPI_MakeCylinder innerMaker(
            gp_Ax2(bezelOrigin, gp::DZ()),
            innerR,
            bezelDepth);
        innerMaker.Build();

        // Annular tool = outer − inner
        BRepAlgoAPI_Cut annularTool(outerMaker.Shape(), innerMaker.Shape());
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

        spdlog::debug("WatchFrontModel::buildBezel w={} d={} outerR={}",
                      bezelWidth, bezelDepth, outerR);
        return StepResult{ result.Shape(), {} };
    }
    catch (const std::exception& e) {
        spdlog::error("WatchFrontModel::buildBezel exception: {}", e.what());
        return StepResult{ TopoDS_Shape(), { BuildWarning{ "E_OCCT", e.what() } } };
    }
}

// ── Convenience: chain all three M1.1 steps
TopoDS_Shape WatchFrontModel::buildAll(const nlohmann::json& spec,
                                        std::vector<BuildWarning>& warnings)
{
    WatchFrontModel model;

    // Step 1
    StepResult s1 = model.buildBase(spec);
    warnings.insert(warnings.end(), s1.warnings.begin(), s1.warnings.end());
    if (s1.shape.IsNull()) {
        spdlog::error("WatchFrontModel::buildAll aborted at step 1");
        return TopoDS_Shape{};
    }

    // Step 2
    StepResult s2 = model.applyCornerRadius(s1.shape, spec);
    warnings.insert(warnings.end(), s2.warnings.begin(), s2.warnings.end());
    if (s2.shape.IsNull()) {
        spdlog::error("WatchFrontModel::buildAll aborted at step 2");
        return TopoDS_Shape{};
    }

    // Step 3
    StepResult s3 = model.buildBezel(s2.shape, spec);
    warnings.insert(warnings.end(), s3.warnings.begin(), s3.warnings.end());
    if (s3.shape.IsNull()) {
        spdlog::error("WatchFrontModel::buildAll aborted at step 3");
        return TopoDS_Shape{};
    }

    return s3.shape;
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
        }}
    };
}

}  // namespace koocadcam::engine
