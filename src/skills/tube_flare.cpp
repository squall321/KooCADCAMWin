// @lat: [[engine/skills#tube_flare]]

#include "tube_flare.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::tube_flare {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.target_od_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "tube_flare target_od_mm (" + std::to_string(in.target_od_mm) +
              ") must be > 0");
    }
    if (in.end_z_mm <= in.start_z_mm) {
        r.add("DFM-INPUT", "error",
              "tube_flare end_z_mm (" + std::to_string(in.end_z_mm) +
              ") must be > start_z_mm (" + std::to_string(in.start_z_mm) + ")");
    }

    if (!wp.shape().IsNull()) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        const double currentDia = 2.0 * bb.outerRadiusXY();
        if (in.target_od_mm > 0.0 && in.target_od_mm <= currentDia) {
            r.add("DFM-TUBE", "error",
                  "tube_flare target_od_mm (" + std::to_string(in.target_od_mm) +
                  ") must be > current OD (" + std::to_string(currentDia) +
                  ") — flaring enlarges diameter");
        }
        if (in.start_z_mm > bb.zMax + 1e-3 || in.end_z_mm < bb.zMin - 1e-3) {
            r.add("DFM-TUBE", "error",
                  "tube_flare zone [" + std::to_string(in.start_z_mm) + "," +
                  std::to_string(in.end_z_mm) + "] does not overlap stock Z [" +
                  std::to_string(bb.zMin) + "," + std::to_string(bb.zMax) + "]");
        }

        // Single-pass flare practical limit: 50% radius expansion.
        if (in.target_od_mm > 0.0 && bb.outerRadiusXY() > 0.0) {
            const double rTarget = in.target_od_mm / 2.0;
            const double rInit = bb.outerRadiusXY();
            const double expansion = (rTarget - rInit) / rInit;
            if (expansion > 0.5) {
                r.add("DFM-FLARE", "warning",
                      "tube_flare expansion ratio " +
                      std::to_string(expansion) +
                      " > 0.5 — wall-thinning / fracture risk on single pass");
            }
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Rebuild strategy: fuse a larger-OD cylinder over the flare axial range.
// (No cut needed — the new cylinder simply envelops the smaller existing
// section, and OCCT's fuse trims the seam.)

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tube_flare DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double rInit   = bb.outerRadiusXY();
    const double rTarget = in.target_od_mm / 2.0;

    // Build the flared section as a cylinder of rTarget over [start_z, end_z].
    const gp_Ax2 axNew(gp_Pnt(0.0, 0.0, in.start_z_mm), gp::DZ());
    TopoDS_Shape newSeg;
    try {
        newSeg = pr::cylinder(axNew, rTarget, in.end_z_mm - in.start_z_mm);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("tube_flare: new segment build failed: ") + ex.what());
    }

    TopoDS_Shape result;
    try {
        result = pr::fuse(wp.shape(), newSeg);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("tube_flare: fuse failed: ") + ex.what());
    }

    // Signature
    json params = {
        { "start_z_mm",   in.start_z_mm },
        { "end_z_mm",     in.end_z_mm },
        { "target_od_mm", in.target_od_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "cylindrical_face_count", 1 },
        { "axis",                 { 0.0, 0.0, 1.0 } },
        { "start_z_mm",           in.start_z_mm },
        { "end_z_mm",             in.end_z_mm },
        { "target_od_mm",         in.target_od_mm },
        { "initial_od_mm",        2.0 * rInit },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "flare_mandrel";
    tooling.tool_dia_mm       = in.target_od_mm;
    tooling.tool_length_mm    = (in.end_z_mm - in.start_z_mm) + 5.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // flaring is cold-forming — no chips
    tooling.est_cycle_time_s  =
        std::max(2.0, (in.end_z_mm - in.start_z_mm) * 0.3);
    {
        const double expansion = (rInit > 0.0)
            ? (rTarget - rInit) / rInit : 0.0;
        tooling.extra = json{
            { "expansion_ratio",  expansion },
            { "process_category", "cold_forming" },
        };
    }

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::tube_flare applied: OD {} → {} over Z [{}, {}] faces {}→{}",
                  2.0 * rInit, in.target_od_mm,
                  in.start_z_mm, in.end_z_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: a cylindrical face with axis = +Z, radius EQUAL to the workpiece
// bbox outer radius (since flaring is now the largest-radius segment) AND
// whose Z range is short (less than the total Z extent) — distinguishing
// from a plain cylindrical stock.  Adjacent +Z/-Z planar face at the
// non-end side confirms the flare shoulder.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const TopoDS_Shape& shape = wp.shape();
    if (shape.IsNull()) return out;

    const pr::Bbox3d bb = pr::optimalBbox(shape);
    const double outerR = bb.outerRadiusXY();
    const double zRange = bb.zMax - bb.zMin;
    if (outerR <= 1e-6 || zRange <= 1e-6) return out;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();

        // The flared section must be the LARGEST radius — i.e. match bbox outer.
        if (std::abs(radius - outerR) > 0.2) continue;

        const gp_Dir adir = cyl.Axis().Direction();
        if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-3) continue;

        // Collect circular edges' Z values on this face.
        std::vector<double> zVals;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Z()) - 1.0) > 1e-3) continue;
            if (std::abs(c.Radius() - radius) > 1e-3) continue;
            zVals.push_back(c.Location().Z());
        }
        if (zVals.size() < 2) continue;

        const double zLow  = *std::min_element(zVals.begin(), zVals.end());
        const double zHigh = *std::max_element(zVals.begin(), zVals.end());
        const double segHeight = zHigh - zLow;

        // The flared section is SHORTER than the full stock Z range —
        // otherwise this is just plain stock, not a flare.
        if (segHeight > 0.8 * zRange) continue;
        if (segHeight < 1e-3) continue;

        // Touches one bbox end (flaring is an end operation).
        const bool touchesMin = std::abs(zLow  - bb.zMin) < 0.2;
        const bool touchesMax = std::abs(zHigh - bb.zMax) < 0.2;
        if (!touchesMin && !touchesMax) continue;

        json recovered = {
            { "start_z_mm",   zLow },
            { "end_z_mm",     zHigh },
            { "target_od_mm", 2.0 * radius },
        };
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "flared_radius_mm",    radius },
            { "segment_height_mm",   segHeight },
            { "touches_zmin",        touchesMin },
            { "touches_zmax",        touchesMax },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.78, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::tube_flare
