// @lat: [[engine/skills#surface_trim_by_plane]]

#include "surface_trim_by_plane.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::surface_trim_by_plane {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (wp.shape().IsNull()) {
        r.add("DFM-INPUT", "error",
              "surface_trim_by_plane: workpiece shape is null");
        return r;
    }
    const double n2 = in.plane_normal[0] * in.plane_normal[0] +
                      in.plane_normal[1] * in.plane_normal[1] +
                      in.plane_normal[2] * in.plane_normal[2];
    if (n2 < 1e-18) {
        r.add("DFM-NORMAL", "error",
              "surface_trim_by_plane: plane_normal is zero-length");
        return r;
    }
    if (in.keep_side != "above" && in.keep_side != "below") {
        r.add("DFM-KEEP-SIDE", "error",
              "surface_trim_by_plane: keep_side must be 'above' or 'below' "
              "(got '" + in.keep_side + "')");
    }
    // Plane must intersect the bbox, else trim is a no-op or wipes everything.
    const auto bb = pr::optimalBbox(wp.shape());
    const double nlen = std::sqrt(n2);
    const double nx = in.plane_normal[0] / nlen;
    const double ny = in.plane_normal[1] / nlen;
    const double nz = in.plane_normal[2] / nlen;
    const double ox = in.plane_origin[0];
    const double oy = in.plane_origin[1];
    const double oz = in.plane_origin[2];
    bool sawPos = false, sawNeg = false;
    for (int i = 0; i < 8; ++i) {
        const double x = (i & 1) ? bb.xMax : bb.xMin;
        const double y = (i & 2) ? bb.yMax : bb.yMin;
        const double z = (i & 4) ? bb.zMax : bb.zMin;
        const double d = (x - ox) * nx + (y - oy) * ny + (z - oz) * nz;
        if (d >  1e-6) sawPos = true;
        if (d < -1e-6) sawNeg = true;
    }
    if (!(sawPos && sawNeg)) {
        r.add("DFM-MISS", "error",
              "surface_trim_by_plane: plane does not intersect workpiece bbox");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "surface_trim_by_plane DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Normalize the plane normal.
    const double n2 = in.plane_normal[0] * in.plane_normal[0] +
                      in.plane_normal[1] * in.plane_normal[1] +
                      in.plane_normal[2] * in.plane_normal[2];
    const double nlen = std::sqrt(n2);
    const gp_Dir normal(in.plane_normal[0] / nlen,
                        in.plane_normal[1] / nlen,
                        in.plane_normal[2] / nlen);
    const gp_Pnt origin(in.plane_origin[0], in.plane_origin[1], in.plane_origin[2]);
    const gp_Pln pln(origin, normal);

    // Reference point on the DISCARD side, well outside the bbox.
    const auto bb = pr::optimalBbox(wp.shape());
    const double diag = std::sqrt(bb.dx() * bb.dx() +
                                  bb.dy() * bb.dy() +
                                  bb.dz() * bb.dz());
    const double step = std::max(10.0, diag * 2.0);
    // "above" keeps +normal → discard -normal side.
    const double sign = (in.keep_side == "above") ? -1.0 : +1.0;
    const gp_Pnt discardSidePt(origin.X() + sign * normal.X() * step,
                               origin.Y() + sign * normal.Y() * step,
                               origin.Z() + sign * normal.Z() * step);

    TopoDS_Shape newShape = wp.shape();
    bool trimOk = false;
    try {
        BRepBuilderAPI_MakeFace mf(pln);
        if (!mf.IsDone())
            throw SkillError("surface_trim_by_plane: failed to build plane face");
        BRepPrimAPI_MakeHalfSpace mh(mf.Face(), discardSidePt);
        const TopoDS_Shape halfDiscard = mh.Solid();
        newShape = pr::cut(wp.shape(), halfDiscard);
        trimOk = !newShape.IsNull();
    } catch (const Standard_Failure& ex) {
        spdlog::debug("surface_trim_by_plane: trim threw — {}", ex.what());
        newShape = wp.shape();
    }

    // Derived volume (after trim).
    double derivedVol = 0.0;
    if (!newShape.IsNull()) {
        GProp_GProps gp;
        BRepGProp::VolumeProperties(newShape, gp);
        derivedVol = gp.Mass();
    }

    json params = {
        { "plane_origin", in.plane_origin },
        { "plane_normal", in.plane_normal },
        { "keep_side",    in.keep_side },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "source_face_count",   1 },
        { "thickness_mm",        0.0 },
        { "plane_origin",        { origin.X(), origin.Y(), origin.Z() } },
        { "plane_normal",        { normal.X(), normal.Y(), normal.Z() } },
        { "keep_side",           in.keep_side },
        { "derived_volume",      derivedVol },
        { "trim_done",           trimOk },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "surface_feature";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = std::max(2.0, diag / 8.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::surface_trim_by_plane: keep={} vol={} ok={}",
                  in.keep_side, derivedVol, trimOk);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic: a trimmed half-space leaves a single planar face
// that bounds the result.  History replay yields confidence 1.0.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "plane_origin", f.params.value("plane_origin",
                                            std::array<double,3>{0,0,0}) },
            { "plane_normal", f.params.value("plane_normal",
                                            std::array<double,3>{0,0,1}) },
            { "keep_side",    f.params.value("keep_side",
                                            std::string("above")) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback: count planar faces.  If at least one large planar
    // face exists, report a low-confidence trim candidate.
    if (wp.shape().IsNull()) return out;
    int planarFaces = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFacePlanar(i)) ++planarFaces;
    }
    if (planarFaces > 0) {
        RecognizedFeature rf;
        rf.skill_id = kSkillId;
        rf.recovered_params = {
            { "plane_origin", std::array<double,3>{ 0.0, 0.0, 0.0 } },
            { "plane_normal", std::array<double,3>{ 0.0, 0.0, 1.0 } },
            { "keep_side",    "above" },
        };
        rf.confidence       = 0.25;
        rf.matched_geometry = json{
            { "source", "geometry" },
            { "planar_face_count", planarFaces },
        };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::surface_trim_by_plane
