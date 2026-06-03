// @lat: [[engine/skills#structural_member_along_path]]

#include "structural_member_along_path.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Circle.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
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

namespace koocadcam::skill::structural_member_along_path {

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

double pathLength(const std::vector<std::array<double, 3>>& pts)
{
    double L = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const double dx = pts[i][0] - pts[i - 1][0];
        const double dy = pts[i][1] - pts[i - 1][1];
        const double dz = pts[i][2] - pts[i - 1][2];
        L += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return L;
}

TopoDS_Wire makePathWire(const std::vector<std::array<double, 3>>& pts)
{
    BRepBuilderAPI_MakeWire wireMaker;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const gp_Pnt p1(pts[i - 1][0], pts[i - 1][1], pts[i - 1][2]);
        const gp_Pnt p2(pts[i][0],     pts[i][1],     pts[i][2]);
        if (p1.Distance(p2) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(p1, p2);
        if (!em.IsDone()) throw Standard_Failure("path edge failed");
        wireMaker.Add(em.Edge());
    }
    if (!wireMaker.IsDone()) throw Standard_Failure("path wire failed");
    return wireMaker.Wire();
}

// Build a closed planar wire from a 2D polygon, on the plane (origin, normal).
TopoDS_Wire makePolyWire(const std::vector<std::pair<double, double>>& poly,
                         const gp_Pnt& origin,
                         const gp_Dir& normal)
{
    const gp_Ax3 ax(origin, normal);
    const gp_Vec ux(ax.XDirection());
    const gp_Vec uy(ax.YDirection());

    BRepBuilderAPI_MakeWire wm;
    const std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; ++i) {
        const auto& a = poly[i];
        const auto& b = poly[(i + 1) % n];
        const gp_Pnt P1 = origin.Translated(ux * a.first + uy * a.second);
        const gp_Pnt P2 = origin.Translated(ux * b.first + uy * b.second);
        if (P1.Distance(P2) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(P1, P2);
        if (!em.IsDone()) throw Standard_Failure("poly edge failed");
        wm.Add(em.Edge());
    }
    if (!wm.IsDone()) throw Standard_Failure("poly wire failed");
    return wm.Wire();
}

// 2D profile polygons (centered on origin) for each profile_kind.
std::vector<std::pair<double, double>> outerPoly(const std::string& kind, double D)
{
    std::vector<std::pair<double, double>> p;
    const double h = D * 0.5;
    if (kind == "square_tube" || kind == "square_solid") {
        p = { { -h, -h }, { h, -h }, { h, h }, { -h, h } };
    } else if (kind == "angle") {
        // L-shape outer outline: two legs of length D, width D (we use D as
        // outer dim and the actual leg thickness is provided by wall_thick).
        p = { { -h, -h }, { h, -h }, { h, -h + D * 0.25 },
              { -h + D * 0.25, -h + D * 0.25 }, { -h + D * 0.25, h }, { -h, h } };
    } else if (kind == "i_beam") {
        // I-section outline:  top flange  H, web Tw, bottom flange H.
        // Simplified: use D as total height & width.
        const double tf = D * 0.15;  // flange thickness
        const double tw = D * 0.10;  // web thickness (half)
        p = {
            { -h, -h }, { h, -h }, { h, -h + tf },
            { tw,  -h + tf }, { tw,  h - tf },
            { h,  h - tf }, { h, h }, { -h, h },
            { -h, h - tf }, { -tw, h - tf },
            { -tw, -h + tf }, { -h, -h + tf },
        };
    } else {
        // Fallback square.
        p = { { -h, -h }, { h, -h }, { h, h }, { -h, h } };
    }
    return p;
}

// Build the profile face (with inner hole for tubes if wall_thick > 0).
TopoDS_Face makeProfileFace(const std::string& kind,
                            double D, double wall,
                            const gp_Pnt& origin,
                            const gp_Dir& normal)
{
    const gp_Ax3 ax(origin, normal);
    const gp_Pln plane(ax);

    if (kind == "round_tube") {
        const gp_Ax2 axisC(origin, normal);
        Handle(Geom_Circle) outerC = new Geom_Circle(axisC, D * 0.5);
        BRepBuilderAPI_MakeEdge eOuter(outerC);
        if (!eOuter.IsDone()) throw Standard_Failure("round outer edge");
        BRepBuilderAPI_MakeWire wOuter(eOuter.Edge());
        if (!wOuter.IsDone()) throw Standard_Failure("round outer wire");

        BRepBuilderAPI_MakeFace fm(plane, wOuter.Wire(), true);
        if (!fm.IsDone()) throw Standard_Failure("round face outer");

        if (wall > 0.0 && wall * 2.0 < D) {
            Handle(Geom_Circle) innerC = new Geom_Circle(axisC, D * 0.5 - wall);
            BRepBuilderAPI_MakeEdge eInner(innerC);
            if (!eInner.IsDone()) throw Standard_Failure("round inner edge");
            // Inner wire reversed → hole.
            BRepBuilderAPI_MakeWire wInner(eInner.Edge());
            if (!wInner.IsDone()) throw Standard_Failure("round inner wire");
            fm.Add(TopoDS::Wire(wInner.Wire().Reversed()));
            if (!fm.IsDone()) throw Standard_Failure("round face with hole");
        }
        return fm.Face();
    }

    // Polygonal profiles.
    const auto outer = outerPoly(kind, D);
    TopoDS_Wire wOuter = makePolyWire(outer, origin, normal);
    BRepBuilderAPI_MakeFace fm(plane, wOuter, true);
    if (!fm.IsDone()) throw Standard_Failure("profile face outer");

    if (kind == "square_tube" && wall > 0.0 && wall * 2.0 < D) {
        const double hi = D * 0.5 - wall;
        std::vector<std::pair<double, double>> inner = {
            { -hi, -hi }, { hi, -hi }, { hi, hi }, { -hi, hi }
        };
        TopoDS_Wire wInner = makePolyWire(inner, origin, normal);
        fm.Add(TopoDS::Wire(wInner.Reversed()));
        if (!fm.IsDone()) throw Standard_Failure("square_tube hole");
    }
    return fm.Face();
}

// Approximate cross-section area for predicted volume.
double profileArea(const std::string& kind, double D, double wall)
{
    if (kind == "square_tube") {
        const double a = D * D;
        if (wall > 0.0 && wall * 2.0 < D) {
            const double inner = (D - 2.0 * wall) * (D - 2.0 * wall);
            return a - inner;
        }
        return a;
    }
    if (kind == "square_solid") {
        return D * D;
    }
    if (kind == "round_tube") {
        const double a = M_PI * D * D * 0.25;
        if (wall > 0.0 && wall * 2.0 < D) {
            const double ri = D * 0.5 - wall;
            return a - M_PI * ri * ri;
        }
        return a;
    }
    if (kind == "i_beam") {
        const double tf = D * 0.15;
        const double tw = D * 0.20;
        const double webH = D - 2.0 * tf;
        return 2.0 * D * tf + webH * tw;
    }
    if (kind == "angle") {
        const double t = D * 0.25;
        return 2.0 * D * t - t * t;
    }
    return D * D;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.path_polyline_xyz.size() < 2) {
        r.add("DFM-INPUT", "error",
              "structural_member_along_path: needs ≥ 2 path waypoints (got " +
              std::to_string(in.path_polyline_xyz.size()) + ")");
    }
    if (in.profile_dim_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "structural_member_along_path: profile_dim_mm must be > 0");
    }
    if (in.wall_thick_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "structural_member_along_path: wall_thick_mm must be ≥ 0");
    }
    if (in.wall_thick_mm > 0.0 &&
        in.profile_dim_mm <= 2.0 * in.wall_thick_mm) {
        r.add("DFM-WALL", "error",
              "structural_member_along_path: profile_dim (" +
              std::to_string(in.profile_dim_mm) +
              ") must be > 2 × wall_thick (" +
              std::to_string(2.0 * in.wall_thick_mm) + ")");
    }
    if (in.path_polyline_xyz.size() >= 2) {
        const double L = pathLength(in.path_polyline_xyz);
        if (L <= in.profile_dim_mm) {
            r.add("DFM-PATH", "error",
                  "structural_member_along_path: path length (" +
                  std::to_string(L) + ") must be > profile_dim (" +
                  std::to_string(in.profile_dim_mm) + ")");
        }
        for (std::size_t i = 1; i < in.path_polyline_xyz.size(); ++i) {
            const auto& a = in.path_polyline_xyz[i - 1];
            const auto& b = in.path_polyline_xyz[i];
            const double dx = b[0] - a[0], dy = b[1] - a[1], dz = b[2] - a[2];
            if (std::sqrt(dx * dx + dy * dy + dz * dz) < 1e-6) {
                r.add("DFM-COLLINEAR", "error",
                      "structural_member_along_path: coincident waypoints");
                break;
            }
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "structural_member_along_path DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto& pts = in.path_polyline_xyz;
    const double L  = pathLength(pts);
    const double A  = profileArea(in.profile_kind, in.profile_dim_mm,
                                  in.wall_thick_mm);

    // 1) path wire
    TopoDS_Wire pathWire;
    try {
        pathWire = makePathWire(pts);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("structural_member_along_path: ") + ex.what());
    }

    // 2) profile at first waypoint normal to first segment
    const gp_Pnt p0(pts[0][0], pts[0][1], pts[0][2]);
    const gp_Pnt p1(pts[1][0], pts[1][1], pts[1][2]);
    gp_Vec firstSeg(p0, p1);
    if (firstSeg.Magnitude() < 1e-9) {
        throw SkillError("structural_member_along_path: zero first segment");
    }
    const gp_Dir firstDir(firstSeg);

    TopoDS_Face profileFace;
    try {
        profileFace = makeProfileFace(in.profile_kind, in.profile_dim_mm,
                                      in.wall_thick_mm, p0, firstDir);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("structural_member_along_path: ") + ex.what());
    }

    // 3) pipe sweep
    TopoDS_Shape swept;
    bool sweepOk = false;
    try {
        BRepOffsetAPI_MakePipe pipe(pathWire, profileFace);
        pipe.Build();
        if (pipe.IsDone()) {
            swept   = pipe.Shape();
            sweepOk = !swept.IsNull();
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("structural_member_along_path: pipe threw — {}", ex.what());
    }

    // 4) fuse onto workpiece (or use swept as result if workpiece is null/empty).
    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape;
    if (sweepOk) {
        if (wp.shape().IsNull()) {
            newShape = swept;
        } else {
            try {
                newShape = pr::fuse(wp.shape(), swept);
            } catch (const Standard_Failure&) {
                BRep_Builder builder;
                TopoDS_Compound comp;
                builder.MakeCompound(comp);
                builder.Add(comp, wp.shape());
                builder.Add(comp, swept);
                newShape = comp;
            }
        }
    } else {
        newShape = wp.shape();
    }

    const double volAfter = volumeOf(newShape);
    const double added    = volAfter - volBefore;
    const double expected = A * L;

    json paramsPath = json::array();
    for (const auto& p : pts) paramsPath.push_back({ p[0], p[1], p[2] });

    json params = {
        { "profile_kind",      in.profile_kind   },
        { "profile_dim_mm",    in.profile_dim_mm },
        { "wall_thick_mm",     in.wall_thick_mm  },
        { "path_polyline_xyz", paramsPath        },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "profile_kind",        in.profile_kind   },
        { "profile_dim_mm",      in.profile_dim_mm },
        { "wall_thick_mm",       in.wall_thick_mm  },
        { "path_length",         L },
        { "section_area_mm2",    A },
        { "derived_volume_mm3",  added },
        { "expected_volume_mm3", expected },
        { "sweep_done",          sweepOk },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "weldment_member";
    tooling.tool_length_mm    = L;
    tooling.stock_removed_mm3 = -expected;  // material added
    tooling.est_cycle_time_s  = std::max(5.0, L / 100.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& f : wp.features()) wpNew->addFeature(f);
    wpNew->addFeature(sig);

    spdlog::debug("skill::structural_member_along_path: kind={} D={} L={} A={} added={}",
                  in.profile_kind, in.profile_dim_mm, L, A, added);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // (1) feature-history fast path
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

    // (2) geometric: a prismatic swept member usually shows a high count of
    // non-planar non-cylindrical "ruled" faces (the side walls) plus 1-2
    // planar end-caps.  Emit a low-confidence guess if ≥ 2 ruled faces.
    int ruled = 0;
    int planar = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFacePlanar(i)) ++planar;
        else if (!wp.isFaceCylinder(i)) ++ruled;
    }
    if (ruled >= 2 || (planar >= 4 && wp.faceCount() >= 6)) {
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = nlohmann::json::object();
        rf.confidence       = 0.40;
        rf.matched_geometry = nlohmann::json{
            { "ruled_face_count",  ruled },
            { "planar_face_count", planar },
        };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::structural_member_along_path
