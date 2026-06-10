// @lat: [[engine/skills#sweep_cut_along_curve]]

#include "sweep_cut_along_curve.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::sweep_cut_along_curve {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

double polygonArea(const std::vector<std::pair<double,double>>& poly)
{
    if (poly.size() < 3) return 0.0;
    double a = 0.0;
    const std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; ++i) {
        const auto& [x1, y1] = poly[i];
        const auto& [x2, y2] = poly[(i + 1) % n];
        a += x1 * y2 - x2 * y1;
    }
    return std::abs(a) * 0.5;
}

double pathLength(const std::vector<gp_Pnt>& pts)
{
    double L = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        L += pts[i].Distance(pts[i - 1]);
    }
    return L;
}

int distinctVertexCount(const std::vector<std::pair<double,double>>& poly)
{
    if (poly.empty()) return 0;
    int n = 1;
    for (std::size_t i = 1; i < poly.size(); ++i) {
        const double dx = poly[i].first  - poly[i - 1].first;
        const double dy = poly[i].second - poly[i - 1].second;
        if (std::sqrt(dx*dx + dy*dy) > 1e-6) ++n;
    }
    const double dxc = poly.front().first  - poly.back().first;
    const double dyc = poly.front().second - poly.back().second;
    if (n >= 2 && std::sqrt(dxc*dxc + dyc*dyc) < 1e-6) --n;
    return n;
}

TopoDS_Wire makePathWire(const std::vector<gp_Pnt>& pts)
{
    BRepBuilderAPI_MakeWire wireMaker;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        if (pts[i].Distance(pts[i - 1]) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(pts[i - 1], pts[i]);
        if (!em.IsDone()) throw Standard_Failure("path edge failed");
        wireMaker.Add(em.Edge());
    }
    if (!wireMaker.IsDone()) throw Standard_Failure("path wire failed");
    return wireMaker.Wire();
}

TopoDS_Face makeProfileFace(const std::vector<std::pair<double,double>>& poly,
                            const gp_Pnt& origin,
                            const gp_Dir& normal)
{
    std::size_t end = poly.size();
    if (end >= 2 &&
        std::hypot(poly.front().first  - poly.back().first,
                   poly.front().second - poly.back().second) < 1e-6) {
        --end;
    }
    if (end < 3) throw Standard_Failure("profile < 3 distinct vertices");

    const gp_Ax3 axis(origin, normal);
    const gp_Pln plane(axis);

    BRepBuilderAPI_MakeWire wireMaker;
    for (std::size_t i = 0; i < end; ++i) {
        const auto& p1 = poly[i];
        const auto& p2 = poly[(i + 1) % end];
        const gp_Vec ux(axis.XDirection());
        const gp_Vec uy(axis.YDirection());
        const gp_Pnt P1 = origin.Translated(ux * p1.first + uy * p1.second);
        const gp_Pnt P2 = origin.Translated(ux * p2.first + uy * p2.second);
        if (P1.Distance(P2) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(P1, P2);
        if (!em.IsDone()) throw Standard_Failure("profile edge failed");
        wireMaker.Add(em.Edge());
    }
    if (!wireMaker.IsDone()) throw Standard_Failure("profile wire failed");

    BRepBuilderAPI_MakeFace fm(plane, wireMaker.Wire(), true);
    if (!fm.IsDone()) throw Standard_Failure("profile face failed");
    return fm.Face();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.path_polyline_xyz.size() < 2) {
        r.add("DFM-INPUT", "error",
              "sweep_cut_along_curve: path needs ≥ 2 waypoints (got " +
              std::to_string(in.path_polyline_xyz.size()) + ")");
    }
    if (distinctVertexCount(in.profile_polyline) < 3) {
        r.add("DFM-SWEEP-PROF", "error",
              "sweep_cut_along_curve: profile needs ≥ 3 distinct vertices");
    }
    if (in.path_polyline_xyz.size() >= 2) {
        const double L = pathLength(in.path_polyline_xyz);
        if (L <= 1e-9) {
            r.add("DFM-SWEEP-PATH", "error",
                  "sweep_cut_along_curve: path length must be > 0 (got " +
                  std::to_string(L) + ")");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sweep_cut_along_curve DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto& pts = in.path_polyline_xyz;
    const double L  = pathLength(pts);
    const double A  = polygonArea(in.profile_polyline);

    TopoDS_Wire pathWire;
    try {
        pathWire = makePathWire(pts);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("sweep_cut_along_curve: ") + ex.what());
    }

    gp_Vec firstSeg(pts[0], pts[1]);
    if (firstSeg.Magnitude() < 1e-9) {
        throw SkillError("sweep_cut_along_curve: zero-length first segment");
    }
    const gp_Dir firstDir(firstSeg);

    TopoDS_Face profileFace;
    try {
        profileFace = makeProfileFace(in.profile_polyline, pts[0], firstDir);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("sweep_cut_along_curve: ") + ex.what());
    }

    TopoDS_Shape sweptShape;
    bool sweepOk = false;
    try {
        BRepOffsetAPI_MakePipe pipe(pathWire, profileFace);
        pipe.Build();
        if (pipe.IsDone()) {
            sweptShape = pipe.Shape();
            sweepOk    = !sweptShape.IsNull();
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("sweep_cut_along_curve: pipe threw — {}", ex.what());
    }

    const double volBefore = volumeOf(wp.shape());

    TopoDS_Shape newShape;
    if (sweepOk) {
        try {
            newShape = pr::cut(wp.shape(), sweptShape);
        } catch (const Standard_Failure& ex) {
            spdlog::debug("sweep_cut_along_curve: cut threw — {}", ex.what());
            newShape = wp.shape();
        }
    } else {
        newShape = wp.shape();
    }

    const double volAfter   = volumeOf(newShape);
    const double removedVol = volBefore - volAfter;
    const double expectedV  = A * L;

    json paramsPath = json::array();
    for (const auto& p : pts) paramsPath.push_back({ p.X(), p.Y(), p.Z() });
    json paramsProf = json::array();
    for (const auto& xy : in.profile_polyline)
        paramsProf.push_back({ xy.first, xy.second });

    json params = {
        { "profile_polyline",   paramsProf },
        { "path_polyline_xyz",  paramsPath },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "section_count",         1 },
        { "path_length_mm",        L },
        { "profile_area_mm2",      A },
        { "derived_volume_mm3",    removedVol },
        { "expected_volume_mm3",   expectedV },
        { "sweep_done",            sweepOk },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "swept_cut";
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(1.0, L / 30.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::sweep_cut_along_curve: A={} L={} removed={} done={}",
                  A, L, removedVol, sweepOk);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.95;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    int ruledLike = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i) && !wp.isFaceCylinder(i)) ++ruledLike;
    }
    if (ruledLike >= 1) {
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = json::object();
        rf.confidence       = 0.4;
        rf.matched_geometry = json{ { "ruled_face_count", ruledLike } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::sweep_cut_along_curve
