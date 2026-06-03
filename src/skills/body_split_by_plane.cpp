// @lat: [[engine/skills#body_split_by_plane]]

#include "body_split_by_plane.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::body_split_by_plane {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Build a solid half-space on the side of the plane that contains `refPoint`.
TopoDS_Shape buildHalfSpace(const gp_Pln& pln, const gp_Pnt& refPoint)
{
    // BRepBuilderAPI_MakeFace from the plane, then MakeHalfSpace using the
    // reference point to choose which side of the plane is the SOLID side.
    BRepBuilderAPI_MakeFace mf(pln);
    if (!mf.IsDone())
        throw SkillError("body_split_by_plane: failed to build plane face");
    BRepPrimAPI_MakeHalfSpace mh(mf.Face(), refPoint);
    return mh.Solid();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    const double n2 = in.plane_normal[0] * in.plane_normal[0] +
                      in.plane_normal[1] * in.plane_normal[1] +
                      in.plane_normal[2] * in.plane_normal[2];
    if (n2 < 1e-18) {
        r.add("DFM-INPUT", "error",
              "body_split_by_plane: plane_normal is zero-length (degenerate plane)");
        return r;
    }
    if (wp.shape().IsNull()) {
        r.add("DFM-INPUT", "error",
              "body_split_by_plane: workpiece shape is null");
        return r;
    }
    // Check the plane intersects the bbox.
    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double nx = in.plane_normal[0] / std::sqrt(n2);
    const double ny = in.plane_normal[1] / std::sqrt(n2);
    const double nz = in.plane_normal[2] / std::sqrt(n2);
    // Signed distance from each bbox corner to the plane.
    bool sawPos = false, sawNeg = false;
    const double ox = in.plane_origin[0];
    const double oy = in.plane_origin[1];
    const double oz = in.plane_origin[2];
    for (int i = 0; i < 8; ++i) {
        const double x = (i & 1) ? bb.xMax : bb.xMin;
        const double y = (i & 2) ? bb.yMax : bb.yMin;
        const double z = (i & 4) ? bb.zMax : bb.zMin;
        const double d = (x - ox) * nx + (y - oy) * ny + (z - oz) * nz;
        if (d >  1e-6) sawPos = true;
        if (d < -1e-6) sawNeg = true;
    }
    if (!(sawPos && sawNeg)) {
        r.add("DFM-INPUT", "error",
              "body_split_by_plane: plane does not intersect workpiece bbox "
              "— no split possible");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "body_split_by_plane DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double n2 = in.plane_normal[0] * in.plane_normal[0] +
                      in.plane_normal[1] * in.plane_normal[1] +
                      in.plane_normal[2] * in.plane_normal[2];
    const double nlen = std::sqrt(n2);
    const gp_Dir normal(in.plane_normal[0] / nlen,
                        in.plane_normal[1] / nlen,
                        in.plane_normal[2] / nlen);
    const gp_Pnt origin(in.plane_origin[0], in.plane_origin[1], in.plane_origin[2]);
    const gp_Pln pln(origin, normal);

    // Pick two reference points on opposite sides of the plane to seed the
    // half-spaces.  Walk a large step along ±normal from origin.
    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double diag = std::sqrt(bb.dx() * bb.dx() + bb.dy() * bb.dy() + bb.dz() * bb.dz());
    const double step = std::max(10.0, diag * 2.0);
    const gp_Pnt posSidePt(origin.X() + normal.X() * step,
                           origin.Y() + normal.Y() * step,
                           origin.Z() + normal.Z() * step);
    const gp_Pnt negSidePt(origin.X() - normal.X() * step,
                           origin.Y() - normal.Y() * step,
                           origin.Z() - normal.Z() * step);

    const TopoDS_Shape halfPos = buildHalfSpace(pln, posSidePt);
    const TopoDS_Shape halfNeg = buildHalfSpace(pln, negSidePt);

    // solidA = wp - halfNeg (keep +normal side); solidB = wp - halfPos
    const TopoDS_Shape solidA = pr::cut(wp.shape(), halfNeg);
    const TopoDS_Shape solidB = pr::cut(wp.shape(), halfPos);

    // Pack into a compound.  Add every TopoDS_Solid sub-shape from each
    // half-result (each may have produced 1+ solids if the workpiece was
    // already a compound).
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    int solidsAdded = 0;
    for (TopExp_Explorer e(solidA, TopAbs_SOLID); e.More(); e.Next()) {
        builder.Add(compound, e.Current());
        ++solidsAdded;
    }
    for (TopExp_Explorer e(solidB, TopAbs_SOLID); e.More(); e.Next()) {
        builder.Add(compound, e.Current());
        ++solidsAdded;
    }
    if (solidsAdded < 2) {
        throw SkillError("body_split_by_plane: produced fewer than 2 solids");
    }

    json params = {
        { "plane_origin", in.plane_origin },
        { "plane_normal", in.plane_normal },
    };
    json pattern = {
        { "kind",          kSkillId },
        { "is_compound",   true },
        { "plane_origin",  { origin.X(), origin.Y(), origin.Z() } },
        { "plane_normal",  { normal.X(), normal.Y(), normal.Z() } },
        { "solid_count",   solidsAdded },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "wire_edm";
    tooling.tool_material     = "brass";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = std::max(5.0, diag / 8.0);

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(compound, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::body_split_by_plane: produced {} solids", solidsAdded);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: a TopoDS_Compound containing ≥ 2 solid sub-shapes.  This is the
// generic "split body" geometric signal.  We confirm and recover the plane
// from the FeatureSignature history when available, otherwise emit a
// low-confidence candidate from geometry alone.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const TopoDS_Shape& shape = wp.shape();
    if (shape.IsNull()) return out;

    int solidCount = 0;
    for (TopExp_Explorer e(shape, TopAbs_SOLID); e.More(); e.Next()) ++solidCount;
    if (solidCount < 2) return out;
    if (shape.ShapeType() != TopAbs_COMPOUND) return out;

    // History replay (high confidence).
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.92;
        rf.matched_geometry = { { "solid_count", solidCount } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // Geometric-only fallback: report we observe a split-compound at low
    // confidence; can't recover the plane without more analysis.
    RecognizedFeature rf;
    rf.skill_id = kSkillId;
    rf.recovered_params = {
        { "plane_origin", { 0.0, 0.0, 0.0 } },
        { "plane_normal", { 0.0, 0.0, 1.0 } },
    };
    rf.confidence       = 0.40;
    rf.matched_geometry = { { "solid_count", solidCount } };
    out.push_back(rf);
    return out;
}

}  // namespace koocadcam::skill::body_split_by_plane
