// @lat: [[engine/skills#partial_pen_fillet_prep]]

#include "partial_pen_fillet_prep.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <BRepBuilderAPI_Transform.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

namespace koocadcam::skill::partial_pen_fillet_prep {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.plate_t <= 0.0) {
        r.add("DFM-INPUT", "error",
              "partial_pen_fillet_prep: plate_t must be > 0");
    }
    if (in.plate_t > 0.0 && in.plate_t < 3.0) {
        r.add("DFM-WELD-PLATE", "error",
              "partial_pen_fillet_prep: plate_t " +
              std::to_string(in.plate_t) +
              " mm < 3 mm (no prep needed)");
    }
    if (in.bevel_angle < 25.0 || in.bevel_angle > 60.0) {
        r.add("DFM-WELD-ANGLE", "error",
              "partial_pen_fillet_prep: bevel_angle " +
              std::to_string(in.bevel_angle) +
              "° outside [25°, 60°]");
    }
    if (in.land_mm < 1.0) {
        r.add("DFM-WELD-LAND", "error",
              "partial_pen_fillet_prep: land_mm " +
              std::to_string(in.land_mm) +
              " mm < 1.0 mm minimum");
    }
    if (in.plate_t > 0.0 && in.land_mm > in.plate_t / 2.0) {
        r.add("DFM-WELD-LAND", "error",
              "partial_pen_fillet_prep: land_mm " +
              std::to_string(in.land_mm) +
              " mm > plate_t/2 = " +
              std::to_string(in.plate_t / 2.0));
    }
    if (in.edge_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "partial_pen_fillet_prep: edge_length_mm must be > 0");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "partial_pen_fillet_prep DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId)
        throw SkillError("partial_pen_fillet_prep: entry_face unresolved");

    const double bevelDepth   = in.plate_t - in.land_mm;
    const double bevelAngRad  = in.bevel_angle * M_PI / 180.0;
    const double bevelTopY    = bevelDepth * std::tan(bevelAngRad);

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;

    const gp_Vec edgeVec(in.edge_dir.X(), in.edge_dir.Y(), 0.0);
    if (edgeVec.Magnitude() < 1e-6)
        throw SkillError("partial_pen_fillet_prep: edge_dir is vertical");
    const gp_Dir xLoc(edgeVec.X(), edgeVec.Y(), 0.0);

    const double L        = in.edge_length_mm;
    const double overhang = 0.05;
    const double cutLen   = L + 2.0 * overhang;

    // ── Sub-feature 1: single bevel ────────────────────────────────────
    const gp_Pnt boxOrigin(
        in.edge_center_x_mm - xLoc.X() * L / 2.0 - xLoc.X() * overhang,
        in.edge_center_y_mm - xLoc.Y() * L / 2.0 - xLoc.Y() * overhang,
        topZ - bevelDepth);
    const gp_Ax2 ax(boxOrigin, gp::DZ(), xLoc);
    const double bevelY = bevelTopY + 0.01;
    const TopoDS_Shape boxRaw = pr::box(ax, cutLen, bevelY, bevelDepth + overhang);

    gp_Trsf rot;
    gp_Ax1 hinge(
        gp_Pnt(in.edge_center_x_mm - xLoc.X() * L / 2.0,
               in.edge_center_y_mm,
               topZ - bevelDepth),
        xLoc);
    rot.SetRotation(hinge, bevelAngRad);
    BRepBuilderAPI_Transform tBox(boxRaw, rot, true);
    const TopoDS_Shape bevelCutter = tBox.Shape();

    // ── Sub-feature 2: back radius cylinder at the heel ───────────────
    constexpr double kBackRadius = 0.5;
    const gp_Pnt cylStart(
        in.edge_center_x_mm - xLoc.X() * L / 2.0 - xLoc.X() * overhang,
        in.edge_center_y_mm - xLoc.Y() * L / 2.0 - xLoc.Y() * overhang,
        topZ - bevelDepth);
    const gp_Ax2 cylAx(cylStart, xLoc);
    const TopoDS_Shape backCyl = pr::cylinder(cylAx, kBackRadius, cutLen);

    // ── Fuse cutters → single cut ─────────────────────────────────────
    const TopoDS_Shape fused    = pr::fuse(bevelCutter, backCyl);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    json params = {
        { "entry_face_id",    *entryId },
        { "edge_dir",         { in.edge_dir.X(), in.edge_dir.Y(), in.edge_dir.Z() } },
        { "plate_t",          in.plate_t },
        { "bevel_angle",      in.bevel_angle },
        { "land_mm",          in.land_mm },
        { "edge_center_x_mm", in.edge_center_x_mm },
        { "edge_center_y_mm", in.edge_center_y_mm },
        { "edge_length_mm",   in.edge_length_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    2 },
        { "plate_t",             in.plate_t },
        { "bevel_angle",         in.bevel_angle },
        { "land_mm",             in.land_mm },
        { "bevel_depth_mm",      bevelDepth },     // derived
        { "bevel_top_offset_mm", bevelTopY },      // derived
        { "back_radius_mm",      kBackRadius },    // derived
        { "edge_length_mm",      in.edge_length_mm },
        { "edge_dir",            { in.edge_dir.X(), in.edge_dir.Y(), in.edge_dir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "bevel_mill;ball_mill";
    tooling.tool_dia_mm       = std::max(4.0, 2.0 * kBackRadius);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = 0.5 * bevelDepth * bevelTopY * in.edge_length_mm;
    tooling.est_cycle_time_s  = std::max(2.0, in.edge_length_mm / 40.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "bevel_mill" }, { "pass", "single_bevel" } },
            { { "tool_type", "ball_mill"  }, { "pass", "back_radius"  },
              { "tool_dia_mm", 2.0 * kBackRadius } },
        } },
        { "aws_classification", "PJP_bevel" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::partial_pen_fillet_prep applied: plate_t={} angle={} land={} L={}",
                  in.plate_t, in.bevel_angle, in.land_mm, in.edge_length_mm);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& s : wp.features()) {
        if (s.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = s.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::partial_pen_fillet_prep
