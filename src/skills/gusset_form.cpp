// @lat: [[engine/skills#gusset_form]]

#include "gusset_form.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepPrimAPI_MakeSphere.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::gusset_form {

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

    const double t = sheetThicknessOf(wp);
    if (t < 0.5 || t > 6.0) {
        r.add("DFM-G-SHEET", "error",
              "gusset_form: stock thickness " + std::to_string(t) +
              " mm outside 0.5–6.0 mm sheet-metal range");
    }
    if (t > 0.0 && in.gusset_width_mm < 3.0 * t) {
        r.add("DFM-G-WIDTH", "error",
              "gusset_form: gusset_width_mm " +
              std::to_string(in.gusset_width_mm) +
              " mm < 3 × thickness — insufficient material");
    }
    if (t > 0.0 && in.gusset_depth_mm < 2.0 * t) {
        r.add("DFM-G-DEPTH", "error",
              "gusset_form: gusset_depth_mm " +
              std::to_string(in.gusset_depth_mm) +
              " mm < 2 × thickness — too shallow to stiffen");
    }
    if (t > 0.0 && in.gusset_depth_mm > 8.0 * t) {
        r.add("DFM-G-DEPTH", "error",
              "gusset_form: gusset_depth_mm " +
              std::to_string(in.gusset_depth_mm) +
              " mm > 8 × thickness — risk of tearing during form");
    }
    if (t > 0.0 && in.gusset_radius_mm < t) {
        r.add("DFM-G-RADIUS", "error",
              "gusset_form: gusset_radius_mm " +
              std::to_string(in.gusset_radius_mm) +
              " mm < thickness — sharp form will crack");
    }
    // Wall normals must be (approximately) unit and non-parallel.
    auto mag = [](const std::array<double, 3>& v) {
        return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    };
    if (mag(in.wall_a_normal) < 1e-6 || mag(in.wall_b_normal) < 1e-6) {
        r.add("DFM-INPUT", "error",
              "gusset_form: wall normals must be non-zero");
    } else {
        const double na[3] = {
            in.wall_a_normal[0] / mag(in.wall_a_normal),
            in.wall_a_normal[1] / mag(in.wall_a_normal),
            in.wall_a_normal[2] / mag(in.wall_a_normal) };
        const double nb[3] = {
            in.wall_b_normal[0] / mag(in.wall_b_normal),
            in.wall_b_normal[1] / mag(in.wall_b_normal),
            in.wall_b_normal[2] / mag(in.wall_b_normal) };
        const double dot = na[0]*nb[0] + na[1]*nb[1] + na[2]*nb[2];
        if (std::abs(dot) > 0.95) {
            r.add("DFM-G-PARALLEL", "error",
                  "gusset_form: wall normals are nearly parallel — no inside corner");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// 1. Cut a triangular relief slit on the INSIDE corner (sphere intersect
//    along the corner bisector pointing INTO the corner).
// 2. Fuse a hemispherical dome on the OUTSIDE corner — the formed gusset.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gusset_form DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt corner(in.corner_xyz[0], in.corner_xyz[1], in.corner_xyz[2]);
    gp_Vec nA(in.wall_a_normal[0], in.wall_a_normal[1], in.wall_a_normal[2]);
    gp_Vec nB(in.wall_b_normal[0], in.wall_b_normal[1], in.wall_b_normal[2]);
    nA.Normalize();
    nB.Normalize();

    // Inside-corner direction = -(nA + nB).normalized (vector pointing into corner)
    gp_Vec inside(-(nA.X() + nB.X()),
                  -(nA.Y() + nB.Y()),
                  -(nA.Z() + nB.Z()));
    if (inside.Magnitude() < 1e-6) {
        // Orthogonal walls: inside is undefined → fall back to -nA.
        inside = -nA;
    }
    inside.Normalize();

    // Outside-corner direction = -inside
    const gp_Vec outside = -inside;

    // 1. Cut: small triangular relief slit at inside corner.  Approximated
    //    as a sphere of radius gusset_radius_mm at corner + small inward offset.
    const gp_Pnt insideCenter(
        corner.X() + inside.X() * (in.gusset_radius_mm * 0.5),
        corner.Y() + inside.Y() * (in.gusset_radius_mm * 0.5),
        corner.Z() + inside.Z() * (in.gusset_radius_mm * 0.5));
    BRepPrimAPI_MakeSphere reliefSphere(insideCenter, in.gusset_radius_mm);
    reliefSphere.Build();
    if (!reliefSphere.IsDone())
        throw SkillError("gusset_form: relief sphere build failed");

    TopoDS_Shape afterRelief;
    try {
        afterRelief = pr::cut(wp.shape(), reliefSphere.Shape());
    } catch (const Standard_Failure& ex) {
        spdlog::warn("gusset_form: relief cut failed ({}); skipping relief", ex.what());
        afterRelief = wp.shape();
    }

    // 2. Fuse formed dome on the outside corner.  Dome = sphere of radius
    //    determined by (gusset_width_mm + gusset_depth_mm) such that it
    //    pokes out by gusset_depth_mm.
    const double domeR =
        (in.gusset_width_mm * in.gusset_width_mm * 0.25 +
         in.gusset_depth_mm * in.gusset_depth_mm) /
        (2.0 * std::max(in.gusset_depth_mm, 1e-3));
    // Center placed so the spherical cap protrudes by `gusset_depth_mm`
    // beyond the corner along the outside bisector.
    const gp_Pnt domeCenter(
        corner.X() + outside.X() * (in.gusset_depth_mm - domeR),
        corner.Y() + outside.Y() * (in.gusset_depth_mm - domeR),
        corner.Z() + outside.Z() * (in.gusset_depth_mm - domeR));
    BRepPrimAPI_MakeSphere domeSphere(domeCenter, domeR);
    domeSphere.Build();
    if (!domeSphere.IsDone())
        throw SkillError("gusset_form: dome sphere build failed");

    TopoDS_Shape result;
    try {
        result = pr::fuse(afterRelief, domeSphere.Shape());
    } catch (const Standard_Failure& ex) {
        spdlog::warn("gusset_form: dome fuse failed: {}; keeping post-relief", ex.what());
        result = afterRelief;
    }

    // Signature.
    json params = {
        { "corner_xyz",       { in.corner_xyz[0], in.corner_xyz[1], in.corner_xyz[2] } },
        { "wall_a_normal",    { nA.X(), nA.Y(), nA.Z() } },
        { "wall_b_normal",    { nB.X(), nB.Y(), nB.Z() } },
        { "gusset_width_mm",  in.gusset_width_mm },
        { "gusset_depth_mm",  in.gusset_depth_mm },
        { "gusset_radius_mm", in.gusset_radius_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "sheet_feature_type",    "stamped_gusset" },
        { "gusset_width_mm",       in.gusset_width_mm },
        { "gusset_depth_mm",       in.gusset_depth_mm },
        { "gusset_radius_mm",      in.gusset_radius_mm },
        { "dome_radius_mm",        domeR },
        { "inside_bisector",       { inside.X(), inside.Y(), inside.Z() } },
        { "sheet_thickness_mm",    sheetThicknessOf(wp) },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "stamping_die";
    tooling.tool_dia_mm       = in.gusset_width_mm;
    tooling.tool_length_mm    = in.gusset_depth_mm;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 1.0;
    tooling.extra["machining_constraint"] =
        "Formed gusset — single-hit stamping with male+female die set; "
        "min radius ≥ thickness; max depth ≤ 8 × thickness to avoid tear";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::gusset_form applied: w={} d={} r={} faces {}→{}",
                  in.gusset_width_mm, in.gusset_depth_mm, in.gusset_radius_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// A formed gusset shows up as ANY spherical (or near-spherical) face on the
// workpiece — distinguishing it from purely planar / cylindrical sheet ops.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const double t = sheetThicknessOf(wp);
    if (t <= 0.0 || t > 6.0) return out;

    // Heuristic: count NON-planar, NON-cylindrical faces (spheres / domes
    // typically register as such).  If any are found close to a corner of
    // the bbox, emit a candidate.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    (void)xMin; (void)yMin; (void)zMin;

    int formedFaceCount = 0;
    gp_Pnt cornerEst(xMax, yMax, zMax);
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFacePlanar(i))   continue;
        if (wp.isFaceCylinder(i)) continue;
        ++formedFaceCount;
        cornerEst = wp.faceCenter(i);
    }
    if (formedFaceCount == 0) return out;

    json recovered = {
        { "corner_xyz",       { cornerEst.X(), cornerEst.Y(), cornerEst.Z() } },
        { "wall_a_normal",    { 1.0, 0.0, 0.0 } },
        { "wall_b_normal",    { 0.0, 0.0, 1.0 } },
        { "gusset_width_mm",  3.0 * t },
        { "gusset_depth_mm",  2.0 * t },
        { "gusset_radius_mm", t },
    };
    json matched = {
        { "formed_face_count", formedFaceCount },
        { "sheet_thickness_mm", t },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    return out;
}

}  // namespace koocadcam::skill::gusset_form
