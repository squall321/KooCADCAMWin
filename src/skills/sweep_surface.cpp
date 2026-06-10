// @lat: [[engine/skills#sweep_surface]]

#include "sweep_surface.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRep_Builder.hxx>
#include <Geom_Circle.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::sweep_surface {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.path_waypoints.size() < 2) {
        r.add("DFM-INPUT", "error",
              "sweep_surface needs ≥ 2 waypoints (got " +
              std::to_string(in.path_waypoints.size()) + ")");
    }
    if (in.profile_radius_mm <= 0.0) {
        r.add("DFM-SWEEP-R", "error",
              "sweep_surface profile_radius_mm must be > 0 (got " +
              std::to_string(in.profile_radius_mm) + ")");
    }
    for (size_t i = 1; i < in.path_waypoints.size(); ++i) {
        if (in.path_waypoints[i].Distance(in.path_waypoints[i - 1]) < 1e-6) {
            r.add("DFM-SWEEP-COLL", "error",
                  "sweep_surface waypoint " + std::to_string(i) +
                  " coincides with previous point (degenerate path edge)");
            break;
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Build a polyline wire from a sequence of waypoints.
TopoDS_Wire makePathWire(const std::vector<gp_Pnt>& pts)
{
    BRepBuilderAPI_MakeWire wireMaker;
    for (size_t i = 1; i < pts.size(); ++i) {
        if (pts[i].Distance(pts[i - 1]) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(pts[i - 1], pts[i]);
        if (!em.IsDone()) throw Standard_Failure("sweep_surface: edge build failed");
        wireMaker.Add(em.Edge());
    }
    if (!wireMaker.IsDone()) throw Standard_Failure("sweep_surface: wire build failed");
    return wireMaker.Wire();
}

// Build a circular profile face perpendicular to the path's first segment,
// centered on the first waypoint.
TopoDS_Face makeCircularProfile(const gp_Pnt& center,
                                const gp_Dir& tangent,
                                double radius)
{
    const gp_Ax2 axis(center, tangent);
    Handle(Geom_Circle) circ = new Geom_Circle(axis, radius);
    BRepBuilderAPI_MakeEdge em(circ);
    if (!em.IsDone()) throw Standard_Failure("sweep_surface: profile edge failed");
    BRepBuilderAPI_MakeWire wm(em.Edge());
    if (!wm.IsDone()) throw Standard_Failure("sweep_surface: profile wire failed");
    BRepBuilderAPI_MakeFace fm(wm.Wire(), true);
    if (!fm.IsDone()) throw Standard_Failure("sweep_surface: profile face failed");
    return fm.Face();
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sweep_surface DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // 1) Build the path wire.
    const std::vector<gp_Pnt>& pts = in.path_waypoints;
    TopoDS_Wire pathWire;
    try {
        pathWire = makePathWire(pts);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("sweep_surface: ") + ex.what());
    }

    // 2) Build the circular profile at the first waypoint.
    gp_Vec firstSeg(pts[0], pts[1]);
    if (firstSeg.Magnitude() < 1e-9) {
        throw SkillError("sweep_surface: zero-length first segment");
    }
    const gp_Dir firstDir(firstSeg);
    TopoDS_Face profile;
    try {
        profile = makeCircularProfile(pts[0], firstDir, in.profile_radius_mm);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("sweep_surface: ") + ex.what());
    }

    // 3) Pipe-sweep.
    TopoDS_Shape sweptShape;
    bool sweepOk = false;
    try {
        BRepOffsetAPI_MakePipe pipe(pathWire, profile);
        pipe.Build();
        if (pipe.IsDone()) {
            sweptShape = pipe.Shape();
            sweepOk = !sweptShape.IsNull();
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("sweep_surface: pipe threw — {}", ex.what());
    }

    // 4) Fuse onto the workpiece.
    TopoDS_Shape newShape;
    if (sweepOk) {
        try {
            newShape = pr::fuse(wp.shape(), sweptShape);
        } catch (const Standard_Failure&) {
            BRep_Builder builder;
            TopoDS_Compound comp;
            builder.MakeCompound(comp);
            if (!wp.shape().IsNull()) builder.Add(comp, wp.shape());
            builder.Add(comp, sweptShape);
            newShape = comp;
        }
    } else {
        newShape = wp.shape();
    }

    // Compute approximate added volume = π r² × path length (rough; ignores
    // bends/intersections).
    double pathLen = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
        pathLen += pts[i].Distance(pts[i - 1]);
    }
    const double approxVolume =
        3.14159265358979323846 * in.profile_radius_mm * in.profile_radius_mm * pathLen;

    json paramsWaypoints = json::array();
    for (const auto& p : pts) {
        paramsWaypoints.push_back({ p.X(), p.Y(), p.Z() });
    }

    json params = {
        { "path_waypoints",    paramsWaypoints },
        { "profile_radius_mm", in.profile_radius_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "path_waypoint_count", static_cast<int>(pts.size()) },
        { "profile_radius_mm",   in.profile_radius_mm },
        { "path_length_mm",      pathLen },
        { "sweep_done",          sweepOk },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "surface_feature";
    tooling.stock_removed_mm3 = -approxVolume;     // material added
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::sweep_surface applied: wpts={} r={} len={} done={}",
                  pts.size(), in.profile_radius_mm, pathLen, sweepOk);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "path_waypoints",
              f.params.value("path_waypoints", nlohmann::json::array()) },
            { "profile_radius_mm",
              f.params.value("profile_radius_mm", 0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::sweep_surface
