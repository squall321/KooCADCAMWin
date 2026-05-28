// @lat: [[engine/skills#coining]]

#include "coining.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::coining {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double sheetThicknessOf(const Workpiece& wp)
{
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    return std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.depth_mm <= 0.0)
        r.add("DFM-INPUT", "error", "coining depth_mm must be > 0");
    if (in.shape == Shape::Circle && in.diameter_mm <= 0.0)
        r.add("DFM-INPUT", "error", "coining circle requires diameter_mm > 0");
    if (in.shape == Shape::Rectangle) {
        if (in.length_mm <= 0.0 || in.width_mm <= 0.0)
            r.add("DFM-INPUT", "error",
                  "coining rectangle requires length_mm > 0 and width_mm > 0");
        if (in.corner_r_mm < 0.0)
            r.add("DFM-INPUT", "error", "coining corner_r_mm must be >= 0");
    }

    const double t = sheetThicknessOf(wp);
    if (t > 5.0) {
        r.add("DFM-C-SHEET", "error",
              "coining: stock thickness " + std::to_string(t) +
              " > 5 mm — not sheet metal");
    }
    if (in.depth_mm > 0.0 && t > 0.0 && in.depth_mm > 0.3 * t) {
        r.add("DFM-C-MAX-DEPTH", "error",
              "coining depth " + std::to_string(in.depth_mm) +
              " mm > 30% × sheet_thickness (" + std::to_string(0.3 * t) +
              " mm) — coining is intentionally shallow; use emboss for deeper");
    }
    if (in.depth_mm > 0.0 && t > 0.0 && in.depth_mm < 0.05 * t) {
        r.add("DFM-C-MIN-DEPTH", "error",
              "coining depth " + std::to_string(in.depth_mm) +
              " mm < 5% × sheet_thickness (" + std::to_string(0.05 * t) +
              " mm) — too shallow to register visibly");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "coining DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t = sheetThicknessOf(wp);

    // Cutter: a vertical cylinder OR rounded-rect prism sitting just above
    // the sheet's top face, extending DOWN by depth_mm + small overhang.
    const double kOverhang = 0.05;
    TopoDS_Shape cutter;
    if (in.shape == Shape::Circle) {
        // gp_Ax2 with mainDir = -Z so the cylinder extends downward by height.
        const gp_Ax2 ax(gp_Pnt(in.position_x_mm, in.position_y_mm,
                                zMax + kOverhang),
                        gp_Dir(0.0, 0.0, -1.0));
        cutter = pr::cylinder(ax, in.diameter_mm / 2.0,
                               in.depth_mm + kOverhang);
    } else {
        const gp_Pnt bottomCenter(in.position_x_mm, in.position_y_mm,
                                   zMax - in.depth_mm);
        cutter = pr::roundedRectPocketTool(bottomCenter,
                                            in.length_mm, in.width_mm,
                                            in.depth_mm + kOverhang,
                                            in.corner_r_mm);
    }

    TopoDS_Shape newShape;
    try {
        newShape = pr::cut(wp.shape(), cutter);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("coining: cut failed: ") + ex.what());
    }

    // Compute footprint area & removed volume.
    double footprintArea = 0.0;
    if (in.shape == Shape::Circle) {
        footprintArea = M_PI * (in.diameter_mm / 2.0) * (in.diameter_mm / 2.0);
    } else {
        const double rc = in.corner_r_mm;
        footprintArea = in.length_mm * in.width_mm
                      - 4.0 * (rc * rc) * (1.0 - M_PI / 4.0);
    }
    const double removedVol = footprintArea * in.depth_mm;

    // Signature
    json dimsJson;
    if (in.shape == Shape::Circle)
        dimsJson = { { "diameter_mm", in.diameter_mm } };
    else
        dimsJson = { { "length_mm",   in.length_mm },
                     { "width_mm",    in.width_mm  } };
    json params = {
        { "position_x_mm", in.position_x_mm },
        { "position_y_mm", in.position_y_mm },
        { "shape",         shapeToString(in.shape) },
        { "dims",          dimsJson },
        { "depth_mm",      in.depth_mm },
        { "corner_r_mm",   in.corner_r_mm },
    };
    json pattern = {
        { "kind",                   kSkillId },
        { "shape",                  shapeToString(in.shape) },
        { "depth_mm",               in.depth_mm },
        { "footprint_area_mm2",     footprintArea },
        { "shallow_pocket_present", true },
        { "sheet_thickness_mm",     t },
        { "depth_ratio",            in.depth_mm / t },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "coining_die";
    tooling.tool_dia_mm       = (in.shape == Shape::Circle)
                                ? in.diameter_mm
                                : std::min(in.length_mm, in.width_mm);
    tooling.tool_length_mm    = in.depth_mm;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = 0.3;        // very fast single stroke
    tooling.extra["machining_constraint"] =
        "Coining — high-tonnage cold-flow stamp; depth ≤ 30% of thickness; "
        "produces sharp impression edges suitable for serial numbers";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::coining applied: shape={} depth={} faces {}→{}",
                  shapeToString(in.shape), in.depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// A coined feature creates: (a) a small planar BOTTOM face below the main
// sheet top by ~ depth_mm (with normal +Z), and (b) a short vertical wall
// (cylindrical or planar) above it.  The depth is shallow (< 30% × t).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThicknessOf(wp);
    if (t <= 0.0 || t > 5.0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // Identify main sheet top Z = max area +Z face.
    int mainTopId = -1;
    double mainTopArea = -1.0;
    double mainTopZ = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double a = wp.faceArea(i);
        if (a > mainTopArea) {
            mainTopArea = a;
            mainTopId   = i;
            mainTopZ    = wp.faceCenter(i).Z();
        }
    }
    if (mainTopId < 0) return out;

    // Vertical cylinder candidates (Circle shape).
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface surf(wp.face(fIdx));
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius   = cyl.Radius();
        const gp_Dir adir     = cyl.Axis().Direction();
        if (std::abs(adir.Z()) < 0.9) continue;

        // The cylinder wall sits between the sheet top and a shallow
        // bottom face.  Find an adjacent +Z face below mainTopZ within
        // 0.3 × t.
        int bottomFace = -1;
        double bottomZ = 0.0;
        for (int g = 0; g < wp.faceCount(); ++g) {
            if (g == mainTopId) continue;
            if (!wp.isFacePlanar(g)) continue;
            gp_Dir n;
            try { n = wp.faceNormal(g); } catch (...) { continue; }
            if (n.Z() < 0.9) continue;
            const double cZ = wp.faceCenter(g).Z();
            const double dz = mainTopZ - cZ;
            if (dz < 0.02 * t) continue;
            if (dz > 0.35 * t) continue;
            // Same XY as cylinder axis?
            const gp_Pnt fc = wp.faceCenter(g);
            const double dxy = std::hypot(fc.X() - cyl.Axis().Location().X(),
                                          fc.Y() - cyl.Axis().Location().Y());
            if (dxy > radius * 1.5) continue;
            bottomFace = g;
            bottomZ = cZ;
            break;
        }
        if (bottomFace < 0) continue;

        const double depthEst = mainTopZ - bottomZ;

        json recovered = {
            { "position_x_mm", cyl.Axis().Location().X() },
            { "position_y_mm", cyl.Axis().Location().Y() },
            { "shape",         "circle" },
            { "dims",          { { "diameter_mm", 2.0 * radius } } },
            { "depth_mm",      depthEst },
            { "corner_r_mm",   0.0 },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "bottom_face_id",      bottomFace },
            { "depth_mm",            depthEst },
            { "sheet_thickness_mm",  t },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.70, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::coining
