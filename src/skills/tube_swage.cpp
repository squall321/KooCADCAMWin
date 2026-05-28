// @lat: [[engine/skills#tube_swage]]

#include "tube_swage.hpp"

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

namespace koocadcam::skill::tube_swage {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.target_od_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "tube_swage target_od_mm (" + std::to_string(in.target_od_mm) +
              ") must be > 0");
    }
    if (in.end_z_mm <= in.start_z_mm) {
        r.add("DFM-INPUT", "error",
              "tube_swage end_z_mm (" + std::to_string(in.end_z_mm) +
              ") must be > start_z_mm (" + std::to_string(in.start_z_mm) + ")");
    }

    if (!wp.shape().IsNull()) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        const double currentDia = 2.0 * bb.outerRadiusXY();
        if (in.target_od_mm > 0.0 && in.target_od_mm >= currentDia) {
            r.add("DFM-TUBE", "error",
                  "tube_swage target_od_mm (" + std::to_string(in.target_od_mm) +
                  ") must be < current OD (" + std::to_string(currentDia) +
                  ") — swaging reduces diameter");
        }
        if (in.start_z_mm < bb.zMin - 1e-3 || in.end_z_mm > bb.zMax + 1e-3) {
            r.add("DFM-TUBE", "error",
                  "tube_swage zone [" + std::to_string(in.start_z_mm) + "," +
                  std::to_string(in.end_z_mm) + "] outside stock Z [" +
                  std::to_string(bb.zMin) + "," + std::to_string(bb.zMax) + "]");
        }

        // Single-pass swage practical limit: 60% radius reduction.
        if (in.target_od_mm > 0.0 && bb.outerRadiusXY() > 0.0) {
            const double rTarget = in.target_od_mm / 2.0;
            const double rInit = bb.outerRadiusXY();
            const double reduction = (rInit - rTarget) / rInit;
            if (reduction > 0.6) {
                r.add("DFM-SWAGE", "warning",
                      "tube_swage reduction ratio " +
                      std::to_string(reduction) +
                      " > 0.6 — likely needs multi-pass swaging");
            }
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Rebuild strategy: cut OUT the existing axial slab in the swage Z range
// (all the way to the axis), then fuse in a NEW cylinder of the target
// outer radius spanning the same Z range.  This produces clean B-rep and
// correctly handles any internal bore — there is none to preserve in a
// solid-stock case, and if a future variant adds a bore, that bore can be
// re-cut after the fuse.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tube_swage DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double rInit   = bb.outerRadiusXY();
    const double rTarget = in.target_od_mm / 2.0;

    // 1) Cutter: an oversized cylinder spanning the swage Z range, radius
    //    slightly larger than current outer (so it overlaps the OD).  We
    //    extend the cutter slightly past start/end so no thin sliver of
    //    Ø_initial remains at the boundary (the new cylinder fused in below
    //    re-fills that overhang).  Overhang is clamped to the stock bbox
    //    so we don't grow material beyond the original Z range.
    constexpr double kOverhangR = 0.5;
    constexpr double kOverhangZ = 0.05;
    const double slabStartZ = std::max(bb.zMin, in.start_z_mm - kOverhangZ);
    const double slabEndZ   = std::min(bb.zMax, in.end_z_mm   + kOverhangZ);
    const double slabHeight = slabEndZ - slabStartZ;
    if (slabHeight <= 0.0) {
        throw SkillError("tube_swage: degenerate swage Z range after clamp");
    }
    const gp_Ax2 axCut(gp_Pnt(0.0, 0.0, slabStartZ), gp::DZ());
    TopoDS_Shape cutter;
    try {
        cutter = pr::cylinder(axCut, rInit + kOverhangR, slabHeight);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("tube_swage: cutter build failed: ") + ex.what());
    }

    TopoDS_Shape stage1;
    try {
        stage1 = pr::cut(wp.shape(), cutter);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("tube_swage: slab cut failed: ") + ex.what());
    }

    // 2) New swaged cylinder spanning the swage Z range at rTarget.
    //    Match the cutter overhang so the slab gap left by the cut is fully
    //    refilled (no thin Ø_initial sliver, no Ø=0 gap).
    const gp_Ax2 axNew(gp_Pnt(0.0, 0.0, slabStartZ), gp::DZ());
    TopoDS_Shape newSeg;
    try {
        newSeg = pr::cylinder(axNew, rTarget, slabHeight);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("tube_swage: new segment build failed: ") + ex.what());
    }

    TopoDS_Shape result;
    try {
        result = pr::fuse(stage1, newSeg);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("tube_swage: fuse failed: ") + ex.what());
    }

    // 3) Signature
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
    tooling.tool_type         = "rotary_swage_die";
    tooling.tool_dia_mm       = in.target_od_mm;
    tooling.tool_length_mm    = (in.end_z_mm - in.start_z_mm) + 5.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // swaging is cold-forming — no chips
    tooling.est_cycle_time_s  =
        std::max(2.0, (in.end_z_mm - in.start_z_mm) * 0.3);
    {
        const double reduction = (rInit > 0.0)
            ? (rInit - rTarget) / rInit : 0.0;
        tooling.extra = json{
            { "reduction_ratio",   reduction },
            { "process_category",  "cold_forming" },
        };
    }

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::tube_swage applied: OD {} → {} over Z [{}, {}] faces {}→{}",
                  2.0 * rInit, in.target_od_mm,
                  in.start_z_mm, in.end_z_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: a cylindrical face with axis = +Z, radius strictly less than the
// workpiece bbox outer radius, AND whose Z range touches one bbox end
// (zMin or zMax).  Distinguishes from turn_external (which has shoulders
// on BOTH sides) — swage's reduced section runs all the way to one end.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const TopoDS_Shape& shape = wp.shape();
    if (shape.IsNull()) return out;

    const pr::Bbox3d bb = pr::optimalBbox(shape);
    const double outerR = bb.outerRadiusXY();
    if (outerR <= 1e-6) return out;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();

        // Must be strictly smaller than the stock outer radius.
        if (radius >= outerR - 1e-3) continue;
        if (std::abs(radius - outerR) < 0.05) continue;

        // Axis along +Z.
        const gp_Dir adir = cyl.Axis().Direction();
        if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-3) continue;

        // Find circular edge Z extent on this face.
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

        // Swage is an END operation: at least one Z extent coincides with
        // bbox zMin or zMax.  This distinguishes from turn_external.
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
            { "swaged_radius_mm",    radius },
            { "touches_zmin",        touchesMin },
            { "touches_zmax",        touchesMax },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.85, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::tube_swage
