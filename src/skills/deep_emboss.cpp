// @lat: [[engine/skills#deep_emboss]]

#include "deep_emboss.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::deep_emboss {

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

    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "deep_emboss diameter_mm must be > 0");
    }
    if (in.emboss_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "deep_emboss emboss_depth_mm must be > 0");
    }

    if (in.emboss_depth_mm > 0.0 && in.emboss_depth_mm < 1.0) {
        r.add("DFM-DEMB-DEPTH-MIN", "error",
              "deep_emboss emboss_depth " + std::to_string(in.emboss_depth_mm) +
              " mm < 1.0 mm — use `emboss` skill for shallow features");
    }

    const double t = sheetThicknessOf(wp);
    if (t > 5.0) {
        r.add("DFM-DEMB-SHEET", "error",
              "deep_emboss: stock thickness " + std::to_string(t) +
              " > 5 mm — not sheet metal");
    }
    if (t <= 0.0) {
        r.add("DFM-DEMB-SHEET", "error",
              "deep_emboss: degenerate stock thickness");
    }

    if (in.emboss_depth_mm > 0.0 && t > 0.0 && in.emboss_depth_mm > 5.0 * t) {
        r.add("DFM-DEMB-DEPTH-MAX", "error",
              "deep_emboss depth " + std::to_string(in.emboss_depth_mm) +
              " mm > 5 × sheet_thickness (" + std::to_string(5.0 * t) +
              " mm) — deep-draw critical limit exceeded (tearing)");
    }

    if (in.emboss_depth_mm > 0.0 && t > 0.0) {
        const double ratio = in.emboss_depth_mm / t;
        if (ratio > 4.0 && ratio <= 5.0) {
            r.add("DFM-DEMB-RATIO", "info",
                  "deep_emboss depth/thickness ratio " +
                  std::to_string(ratio) +
                  " > 4 — verify lubrication and step-form sequence to avoid tearing");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "deep_emboss DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double t           = sheetThicknessOf(wp);
    const double sheetTopZ   = zMax;
    const double ratio       = (t > 0.0) ? in.emboss_depth_mm / t : 0.0;

    // Additive raised cylinder.
    const gp_Ax2 ax(gp_Pnt(in.position_x_mm, in.position_y_mm, sheetTopZ),
                    gp::DZ());
    const TopoDS_Shape bump = pr::cylinder(ax, in.diameter_mm / 2.0,
                                            in.emboss_depth_mm);
    const TopoDS_Shape result = pr::fuse(wp.shape(), bump);

    // Signature
    json params = {
        { "position_x_mm",     in.position_x_mm },
        { "position_y_mm",     in.position_y_mm },
        { "diameter_mm",       in.diameter_mm },
        { "emboss_depth_mm",   in.emboss_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "emboss_depth_mm",            in.emboss_depth_mm },
        { "diameter_mm",                in.diameter_mm },
        { "depth_to_thickness_ratio",   ratio },
        { "sheet_thickness_mm",         t },
        { "axis_perpendicular_z",       true },
        { "raised_face_present",        true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "deep_emboss_die";
    tooling.tool_dia_mm       = in.diameter_mm;
    tooling.tool_length_mm    = in.emboss_depth_mm;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 1.0;
    tooling.extra["depth_to_thickness_ratio"] = ratio;
    tooling.extra["machining_constraint"]    =
        "Deep emboss — paired punch + die with controlled draw bead; "
        "sheet thins proportionally to depth ratio; lubricate to prevent tearing";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::deep_emboss applied: depth={} dia={} ratio={}",
                  in.emboss_depth_mm, in.diameter_mm, ratio);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Find a vertical-axis cylindrical face whose lower ring sits on the sheet
// top and whose span ≥ 1 mm.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThicknessOf(wp);
    if (t <= 0.0 || t > 5.0) return out;

    // Find sheet top Z.
    int mainTop = -1;
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
            mainTop     = i;
            mainTopZ    = wp.faceCenter(i).Z();
        }
    }
    if (mainTop < 0) return out;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius   = cyl.Radius();
        const gp_Ax1  axis    = cyl.Axis();
        const gp_Dir  adir    = axis.Direction();

        if (std::abs(adir.Z()) < 0.9) continue;

        std::vector<double> zCircles;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            zCircles.push_back(crv.Circle().Location().Z());
        }
        if (zCircles.size() < 2) continue;
        const double zLow  = *std::min_element(zCircles.begin(), zCircles.end());
        const double zHigh = *std::max_element(zCircles.begin(), zCircles.end());
        const double span  = zHigh - zLow;

        // Deep emboss: lower ring at sheet top, span ≥ 1 mm.
        if (std::abs(zLow - mainTopZ) > 0.05) continue;
        if (zHigh <= mainTopZ + 1e-3) continue;
        if (span < 1.0 - 1e-3) continue;       // < 1 mm → plain emboss
        if (span > 5.0 * t + 0.1) continue;    // unreasonable

        const double ratio = (t > 0.0) ? span / t : 0.0;

        json recovered = {
            { "position_x_mm",   axis.Location().X() },
            { "position_y_mm",   axis.Location().Y() },
            { "diameter_mm",     2.0 * radius },
            { "emboss_depth_mm", span },
        };
        json matched = {
            { "cylindrical_face_id",      fIdx },
            { "sheet_thickness_mm",       t },
            { "depth_to_thickness_ratio", ratio },
            { "main_top_face_id",         mainTop },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.75, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::deep_emboss
