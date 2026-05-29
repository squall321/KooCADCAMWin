// @lat: [[engine/skills#nurbs_patch]]

#include "nurbs_patch.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Builder.hxx>
#include <Geom_BSplineSurface.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <NCollection_Array2.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::nurbs_patch {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// -- Validation ------------------------------------------------------------

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    const int nu = static_cast<int>(in.control_grid.size());
    if (nu < 4) {
        r.add("DFM-INPUT", "error",
              "nurbs_patch needs >= 4 control rows (got " +
              std::to_string(nu) + ")");
        return r;
    }
    const int nv = static_cast<int>(in.control_grid[0].size());
    if (nv < 4) {
        r.add("DFM-INPUT", "error",
              "nurbs_patch needs >= 4 control cols (got " +
              std::to_string(nv) + ")");
        return r;
    }
    for (int u = 0; u < nu; ++u) {
        if (static_cast<int>(in.control_grid[u].size()) != nv) {
            r.add("DFM-INPUT", "error",
                  "nurbs_patch control_grid is not rectangular at row " +
                  std::to_string(u));
            return r;
        }
    }

    if (in.degree_u < 2 || in.degree_v < 2) {
        r.add("DFM-NURBS-DEG", "error",
              "nurbs_patch effective surface degree must be >= 2 (got u=" +
              std::to_string(in.degree_u) + ", v=" +
              std::to_string(in.degree_v) + ")");
    }
    if (in.degree_u >= nu) {
        r.add("DFM-NURBS-DEG", "error",
              "nurbs_patch degree_u (" + std::to_string(in.degree_u) +
              ") must be < nu (" + std::to_string(nu) + ")");
    }
    if (in.degree_v >= nv) {
        r.add("DFM-NURBS-DEG", "error",
              "nurbs_patch degree_v (" + std::to_string(in.degree_v) +
              ") must be < nv (" + std::to_string(nv) + ")");
    }

    return r;
}

// -- Synthesis -------------------------------------------------------------

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "nurbs_patch DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const int nu = static_cast<int>(in.control_grid.size());
    const int nv = static_cast<int>(in.control_grid[0].size());

    // Load into an NCollection_Array2 (1-based indexing). OCCT 8.0 deprecates
    // the TColgp_Array2OfPnt typedef; use NCollection_Array2<gp_Pnt> directly.
    NCollection_Array2<gp_Pnt> arr(1, nu, 1, nv);
    for (int u = 0; u < nu; ++u) {
        for (int v = 0; v < nv; ++v) {
            arr.SetValue(u + 1, v + 1, in.control_grid[u][v]);
        }
    }

    Handle(Geom_BSplineSurface) surf;
    try {
        // OCCT's GeomAPI_PointsToBSplineSurface drives an
        // AppDef_BSplineCompute fit whose ConstraintOrder is derived from
        // the requested continuity.  For GeomAbs_C2 the fit requires
        // DegMax >= 2*Order + 1 = 5; a lower DegMax raises
        // "WorkDegree too small for given ConstraintOrder".  Lift DegMax
        // to 8 (>5) and floor DegMin at 3 (cubic min that supports C2).
        const int degMinU = std::max(3, in.degree_u);
        const int degMinV = std::max(3, in.degree_v);
        const int degMaxU = std::max(degMinU + 1, 8);
        const int degMaxV = std::max(degMinV + 1, 8);
        GeomAPI_PointsToBSplineSurface fit(
            arr, degMinU, degMaxU,
            degMinV, degMaxV,
            GeomAbs_C2, 1.0e-3);
        if (!fit.IsDone()) {
            throw SkillError("nurbs_patch: surface fit did not converge");
        }
        surf = fit.Surface();
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("nurbs_patch: OCCT failure - ") +
                         ex.what());
    }
    if (surf.IsNull()) {
        throw SkillError("nurbs_patch: produced null surface");
    }

    // Materialize as a single face using natural surface bounds.
    BRepBuilderAPI_MakeFace faceMaker(surf, 1.0e-3);
    if (!faceMaker.IsDone()) {
        throw SkillError("nurbs_patch: BRepBuilderAPI_MakeFace failed");
    }
    const TopoDS_Face patchFace = faceMaker.Face();

    // Fuse onto the workpiece.  The fuse with a free-standing face produces
    // a compound containing the original solid + the new face; this is the
    // additive surface result the skill promises.
    TopoDS_Shape newShape;
    try {
        newShape = pr::fuse(wp.shape(), patchFace);
    } catch (const Standard_Failure&) {
        // If the boolean fails (e.g. the patch is disjoint from the solid),
        // we still want to surface the face - wrap shape() in a compound.
        BRep_Builder builder;
        TopoDS_Compound comp;
        builder.MakeCompound(comp);
        if (!wp.shape().IsNull()) builder.Add(comp, wp.shape());
        builder.Add(comp, patchFace);
        newShape = comp;
    }

    json params = {
        { "nu",       nu },
        { "nv",       nv },
        { "degree_u", in.degree_u },
        { "degree_v", in.degree_v },
    };
    json pattern = {
        { "kind",     kSkillId },
        { "nu",       nu },
        { "nv",       nv },
        { "degree_u", in.degree_u },
        { "degree_v", in.degree_v },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "surface_feature";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::nurbs_patch applied: {}x{} grid, degree {}x{} faces {}->{}",
                  nu, nv, in.degree_u, in.degree_v,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// -- Recognition -----------------------------------------------------------
//
// History replay is the only reliable path for free-form surfaces: a
// recovered BSpline face's parameter grid is not deterministic across
// STEP round-trips.  We thus rely on feature history.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "nu",       f.params.value("nu",       0) },
            { "nv",       f.params.value("nv",       0) },
            { "degree_u", f.params.value("degree_u", 0) },
            { "degree_v", f.params.value("degree_v", 0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Topology fallback: count BSpline-surface faces.
    int bsplineFaces = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        BRepAdaptor_Surface s(wp.face(i));
        if (s.GetType() == GeomAbs_BSplineSurface ||
            s.GetType() == GeomAbs_BezierSurface) {
            ++bsplineFaces;
        }
    }
    if (bsplineFaces > 0) {
        json rec = {
            { "nu",       0 },
            { "nv",       0 },
            { "degree_u", 3 },
            { "degree_v", 3 },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.55,
            json{ { "bspline_face_count", bsplineFaces } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::nurbs_patch
