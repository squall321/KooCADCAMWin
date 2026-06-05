// @lat: [[engine/skills#chainsaw_bar_chain_groove]]

#include "chainsaw_bar_chain_groove.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::chainsaw_bar_chain_groove {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bar_length_mm <= 0.0 || in.groove_width_mm <= 0.0
        || in.groove_depth_mm <= 0.0 || in.sprocket_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "chainsaw_bar_chain_groove: every dimension must be > 0");
    }
    if (in.groove_width_mm > 0.0
        && (in.groove_width_mm < 1.0 || in.groove_width_mm > 2.5)) {
        r.add("DFM-GROOVE", "error",
              "chainsaw_bar_chain_groove: groove_width " +
              std::to_string(in.groove_width_mm) +
              " mm outside typical .050\"-.063\" (1.0-2.5 mm) range");
    }
    if (in.groove_depth_mm > 0.0
        && (in.groove_depth_mm < 4.0 || in.groove_depth_mm > 12.0)) {
        r.add("DFM-DEPTH", "error",
              "chainsaw_bar_chain_groove: groove_depth " +
              std::to_string(in.groove_depth_mm) +
              " mm outside typical 4-12 mm range");
    }
    if (in.sprocket_radius_mm > 0.0 && in.bar_length_mm > 0.0
        && in.sprocket_radius_mm * 4.0 > in.bar_length_mm) {
        r.add("DFM-BAR", "error",
              "chainsaw_bar_chain_groove: bar_length too short relative to "
              "sprocket_radius");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "chainsaw_bar_chain_groove DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;

    // Bar runs along +X starting at bar_axis_origin.  Groove is centred
    // on Y = bar_axis_origin.Y(), cut DOWN from the top face by groove_depth.
    const double yCenter = in.bar_axis_origin.Y();
    const double xStart  = in.bar_axis_origin.X();

    // The groove runs from the tail end up to where the sprocket relief
    // begins (sprocket_radius before bar end).  The relief is the OPEN end.
    const double grooveLen = in.bar_length_mm - in.sprocket_radius_mm;
    const double w = in.groove_width_mm;
    const double d = in.groove_depth_mm;

    // ── 1) Central groove (closed-end box cut from the top face) ────────
    // Axis-aligned box: origin at (xStart-kOver, yCenter-w/2, topZ-d),
    // sized (grooveLen+kOver, w, d+kOver).
    const gp_Pnt boxOrigin(xStart - kOver,
                           yCenter - w / 2.0,
                           topZ - d);
    BRepPrimAPI_MakeBox mkBox(boxOrigin,
                              grooveLen + kOver,
                              w,
                              d + kOver);
    mkBox.Build();
    if (!mkBox.IsDone())
        throw SkillError("chainsaw_bar_chain_groove: groove box build failed");
    const TopoDS_Shape grooveBox = mkBox.Shape();

    TopoDS_Shape current = pr::cut(wp.shape(), grooveBox);

    // ── 2) Sprocket-end arc cutout ──────────────────────────────────────
    //
    // At the nose end of the bar we relieve a semicircular pocket so the
    // chain wraps the tip sprocket.  Cut a shallow cylinder (only down to
    // groove depth so the bar body remains) — this opens the groove out
    // into the sprocket relief at the nose end.
    const gp_Pnt sprocketShallowOrigin(
        xStart + in.bar_length_mm - in.sprocket_radius_mm,
        yCenter,
        topZ - d);
    const gp_Ax2 sprocketShallowAx(sprocketShallowOrigin, gp::DZ());
    const TopoDS_Shape sprocketShallow = pr::cylinder(
        sprocketShallowAx,
        in.sprocket_radius_mm,
        d + kOver);
    current = pr::cut(current, sprocketShallow);
    (void)zMin;
    (void)xMin; (void)yMin; (void)xMax; (void)yMax;

    const double vGroove = grooveLen * w * d;
    const double vSprocket = M_PI * in.sprocket_radius_mm * in.sprocket_radius_mm * d;
    const double volRemoved = vGroove + vSprocket;

    json params = {
        { "bar_axis_origin",   { in.bar_axis_origin.X(),
                                 in.bar_axis_origin.Y(),
                                 in.bar_axis_origin.Z() } },
        { "bar_length_mm",     in.bar_length_mm },
        { "groove_width_mm",   in.groove_width_mm },
        { "groove_depth_mm",   in.groove_depth_mm },
        { "sprocket_radius_mm", in.sprocket_radius_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "ag_feature_type",            "chainsaw_guide_bar_groove" },
        { "subfeature_count",           2 },
        { "bar_length_mm",              in.bar_length_mm },
        { "groove_width_mm",            in.groove_width_mm },
        { "groove_depth_mm",            in.groove_depth_mm },
        { "sprocket_radius_mm",         in.sprocket_radius_mm },
        { "derived_groove_length_mm",   grooveLen },
        { "derived_volume_removed_mm3", volRemoved },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;circular_mill";
    tooling.tool_dia_mm       = in.groove_width_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(30.0, in.bar_length_mm / 8.0);
    tooling.extra = {
        { "ag_application", "chainsaw_guide_bar" },
        { "groove_pitch_inch_class", "0.050/0.058/0.063" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::chainsaw_bar_chain_groove: L={} w={} d={} sprR={}",
                  in.bar_length_mm, in.groove_width_mm,
                  in.groove_depth_mm, in.sprocket_radius_mm);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric: long narrow box-pocket (chain groove) + one cylindrical
    // face (sprocket relief).  Heuristic: count cyl faces with small radius
    // (sprocket relief candidate) and the presence of axis-aligned planar
    // walls suggesting a long slot.
    int cylCount = 0;
    int verticalWalls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFaceCylinder(i)) {
            try {
                BRepAdaptor_Surface s(wp.face(i));
                const double radius = s.Cylinder().Radius();
                if (radius >= 5.0 && radius <= 40.0) ++cylCount;
            } catch (...) {}
        } else if (wp.isFacePlanar(i)) {
            try {
                gp_Dir n = wp.faceNormal(i);
                if (std::abs(n.Z()) < 0.2) ++verticalWalls;
            } catch (...) {}
        }
    }
    if (cylCount >= 1 && verticalWalls >= 2) {
        json recovered = { { "sprocket_radius_mm", 18.0 } };
        json matched   = { { "source", "geometric_chain_groove_pattern" },
                           { "cyl_count",      cylCount },
                           { "vertical_walls", verticalWalls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::chainsaw_bar_chain_groove
