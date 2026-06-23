// @lat: [[engine/skills#annular_groove]]

#include "annular_groove.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <GeomAbs_CurveType.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::annular_groove {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.outer_dia_mm <= in.inner_dia_mm + 0.4)
        r.add("DFM-INPUT", "error",
              "annular_groove: outer_dia must exceed inner_dia by >= 0.4 mm (groove width)");
    if (in.inner_dia_mm < 0.0)
        r.add("DFM-INPUT", "error", "annular_groove: inner_dia must be >= 0");
    if (in.depth_mm <= 0.0)
        r.add("DFM-INPUT", "error", "annular_groove: depth_mm must be > 0");
    if (in.taper_deg < 0.0 || in.taper_deg > 60.0)
        r.add("DFM-INPUT", "error", "annular_groove: taper_deg out of [0, 60]");
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "annular_groove DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceIdOpt = wp.resolve(in.entry_face);
    if (!faceIdOpt) throw SkillError("annular_groove: entry_face datum unresolved");
    const int faceId = *faceIdOpt;
    const gp_Pnt origin = wp.faceCenter(faceId);
    const gp_Dir normal = wp.faceNormal(faceId);

    // In-plane X axis for placing the ring centre at face-local (x, y).
    gp_Vec vx(gp::DX());
    if (std::abs(vx.Dot(gp_Vec(normal))) > 0.99) vx = gp_Vec(gp::DY());
    vx = vx - gp_Vec(normal) * vx.Dot(gp_Vec(normal));
    if (vx.Magnitude() < 1e-9) throw SkillError("annular_groove: cannot build face X-axis");
    vx.Normalize();
    const gp_Vec vy = gp_Vec(normal).Crossed(vx);

    // Ring centre lifted to world, and the cut axis = INTO the face (-normal).
    const gp_Pnt centre(
        origin.X() + in.center_x_mm * vx.X() + in.center_y_mm * vy.X(),
        origin.Y() + in.center_x_mm * vx.Y() + in.center_y_mm * vy.Y(),
        origin.Z() + in.center_x_mm * vx.Z() + in.center_y_mm * vy.Z());
    const gp_Dir cutDir(-normal.X(), -normal.Y(), -normal.Z());
    const gp_Ax2 ax(centre, cutDir, gp_Dir(vx));

    const double outerR = in.outer_dia_mm * 0.5;
    const double innerR = in.inner_dia_mm * 0.5;
    TopoDS_Shape tool;
    if (in.taper_deg > 1e-6) {
        // Tapered outer wall: bottom radius shrinks by depth*tan(taper).
        const double bottomR = outerR - in.depth_mm * std::tan(in.taper_deg * M_PI / 180.0);
        if (bottomR > innerR)
            tool = pr::annularConeRing(ax, bottomR, outerR, innerR, in.depth_mm);
        else
            tool = pr::annularRing(ax, outerR, innerR, in.depth_mm);
    } else {
        tool = pr::annularRing(ax, outerR, innerR, in.depth_mm);
    }
    const TopoDS_Shape newShape = pr::cut(wp.shape(), tool);

    const double vol = M_PI * (outerR * outerR - innerR * innerR) * in.depth_mm;

    json params = {
        { "entry_face_id", faceId },
        { "center_x_mm",   in.center_x_mm },
        { "center_y_mm",   in.center_y_mm },
        { "outer_dia_mm",  in.outer_dia_mm },
        { "inner_dia_mm",  in.inner_dia_mm },
        { "depth_mm",      in.depth_mm },
        { "taper_deg",     in.taper_deg },
        { "face_normal",   { normal.X(), normal.Y(), normal.Z() } },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "is_compound",        false },
        { "outer_dia_mm",       in.outer_dia_mm },
        { "inner_dia_mm",       in.inner_dia_mm },
        { "depth_mm",           in.depth_mm },
        { "derived_volume_mm3", vol },
        { "face_normal",        { normal.X(), normal.Y(), normal.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "endmill;ring_groove";
    tooling.tool_dia_mm       = std::max(0.5, (outerR - innerR));
    tooling.tool_length_mm    = in.depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = vol;
    tooling.est_cycle_time_s  = std::max(1.0, in.depth_mm / 30.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(validateOrHeal(newShape, "annular_groove"),
                                             wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::annular_groove applied: OD={} ID={} depth={} vol={}",
                  in.outer_dia_mm, in.inner_dia_mm, in.depth_mm, vol);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

namespace {

struct CylInfo {
    int    faceIdx;
    gp_Ax1 axis;
    double radius;
    double axialMin, axialMax;   // along axis, relative to axis location
    gp_Pnt axisLoc;
};

std::vector<CylInfo> collectCyls(const Workpiece& wp)
{
    std::vector<CylInfo> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cyl = surf.Cylinder();
        CylInfo info;
        info.faceIdx = i;
        info.axis    = cyl.Axis();
        info.radius  = cyl.Radius();
        info.axisLoc = cyl.Axis().Location();
        const gp_Dir adir = info.axis.Direction();
        // Axial extent measured from the WORLD origin along the axis direction,
        // so two coaxial cylinders are directly comparable (their own axis
        // Locations may sit at different points on the shared line).
        double aMin = std::numeric_limits<double>::max();
        double aMax = std::numeric_limits<double>::lowest();
        bool any = false;
        for (TopExp_Explorer exp(wp.face(i), TopAbs_EDGE); exp.More(); exp.Next()) {
            BRepAdaptor_Curve crv(TopoDS::Edge(exp.Current()));
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            const gp_Pnt p = c.Location();
            const double proj = p.X() * adir.X() + p.Y() * adir.Y() + p.Z() * adir.Z();
            aMin = std::min(aMin, proj);
            aMax = std::max(aMax, proj);
            any  = true;
        }
        if (!any) continue;
        info.axialMin = aMin;
        info.axialMax = aMax;
        out.push_back(info);
    }
    return out;
}

// An ANNULAR FLOOR: a flat (planar) face bounded by TWO concentric circular
// edges (an inner hole + an outer rim) about one axis.  This is the load-bearing
// signature of a ring channel — a watch bezel, a deco ring, a milled face groove
// — and is present whether or not the outer wall is a distinct face (on a round
// watch case the bezel's outer wall IS the case exterior, so there is no short
// outer cylinder; the floor still carries both radii).  An O-ring seal U-groove
// has a ROUNDED floor (no flat annular plane), so this excludes it.
struct AnnularFloor {
    int    faceIdx = -1;
    gp_Dir axis;
    gp_Pnt center;       // floor plane centre (on the axis)
    double innerR = 0.0;
    double outerR = 0.0;
    bool   ok = false;
};

AnnularFloor findAnnularFloor(const Workpiece& wp)
{
    AnnularFloor best;
    double bestWidth = -1.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir fn;
        gp_Pnt fc;
        try { fn = wp.faceNormal(i); fc = wp.faceCenter(i); } catch (...) { continue; }
        // Gather concentric circular edges (same centre on the face normal).
        std::vector<std::pair<double, gp_Pnt>> circs;  // (radius, centre)
        for (TopExp_Explorer exp(wp.face(i), TopAbs_EDGE); exp.More(); exp.Next()) {
            BRepAdaptor_Curve crv(TopoDS::Edge(exp.Current()));
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            // The circle axis must align with the face normal (coplanar ring).
            if (std::abs(std::abs(c.Axis().Direction().Dot(fn)) - 1.0) > 1e-2) continue;
            circs.emplace_back(c.Radius(), c.Location());
        }
        if (circs.size() < 2) continue;
        // Find the innermost + outermost concentric pair.
        std::sort(circs.begin(), circs.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        const double innerR = circs.front().first;
        const double outerR = circs.back().first;
        if (innerR < 0.3) continue;                         // need a real inner hole
        if (outerR - innerR < 0.4) continue;                // need real groove width
        // Centres must be coaxial (within tol).
        const gp_Pnt cIn = circs.front().second, cOut = circs.back().second;
        gp_Vec d(cIn, cOut);
        const gp_Vec axV(fn);
        if ((d - axV * d.Dot(axV)).Magnitude() > 0.1) continue;
        const double width = outerR - innerR;
        if (width > bestWidth) {
            bestWidth = width;
            best.faceIdx = i; best.axis = fn; best.center = fc;
            best.innerR = innerR; best.outerR = outerR; best.ok = true;
        }
    }
    return best;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // metadata replay (cap-exempt full confidence).
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric path B — FLOOR-FIRST.  The discriminating signature is a flat
    // ANNULAR FLOOR (an axis-normal plane bounded by concentric inner+outer
    // circles).  It carries the inner AND outer radius, so the outer wall need
    // NOT be a distinct face — on a round watch case the bezel's outer wall IS
    // the case exterior, with no short outer cylinder; the floor still has both
    // radii.  To confirm a real GROOVE (and reject a counterbore/stepped bore,
    // whose seat plane is also annular), require an INNER-WALL cylinder at the
    // floor's inner radius whose deep end sits AT the floor — a counterbore's
    // pilot cylinder runs PAST the seat floor, so its inner wall is deeper than
    // the floor and is rejected.
    const AnnularFloor floor = findAnnularFloor(wp);
    if (!floor.ok) return out;

    const gp_Dir adir = floor.axis;
    // Project any point onto a single fixed axis line (so all faces compare on
    // one signed number, independent of each cylinder's own axis-dir sign).
    auto axialOf = [&](const gp_Pnt& p) {
        return p.X() * adir.X() + p.Y() * adir.Y() + p.Z() * adir.Z();
    };
    // Per-cylinder [lo, hi] axial range on that same fixed line.
    auto cylRange = [&](const CylInfo& c, double& lo, double& hi) {
        const double s = (c.axis.Direction().Dot(adir) >= 0.0) ? 1.0 : -1.0;
        const double a = c.axialMin * s, b = c.axialMax * s;
        lo = std::min(a, b); hi = std::max(a, b);
    };
    auto coaxial = [&](const CylInfo& c) {
        if (std::abs(std::abs(c.axis.Direction().Dot(adir)) - 1.0) > 1e-2) return false;
        gp_Vec rel(floor.center, c.axisLoc);
        return (rel - gp_Vec(adir) * rel.Dot(gp_Vec(adir))).Magnitude() < 0.2;
    };
    const double floorAxial = axialOf(floor.center);

    const auto cyls = collectCyls(wp);

    // The inner WALL: a coaxial cylinder at radius ~ floor.innerR running entry
    // -> floor (the floor must be one END of its axial range).  depth = its span.
    double depth = -1.0;
    int innerFace = -1, outerFace = -1;
    for (const auto& cyl : cyls) {
        if (std::abs(cyl.radius - floor.outerR) < 0.2) { outerFace = cyl.faceIdx; continue; }
        if (std::abs(cyl.radius - floor.innerR) > 0.2) continue;
        if (!coaxial(cyl)) continue;
        double lo, hi; cylRange(cyl, lo, hi);
        if (std::min(std::abs(lo - floorAxial), std::abs(hi - floorAxial)) > 0.25) continue;
        const double d = hi - lo;
        if (d > depth) { depth = d; innerFace = cyl.faceIdx; }
    }
    if (depth < 0.1) return out;

    // SPECIFICITY — the floor must be SOLID below: a ring channel is closed at
    // the floor (nothing deeper inside the inner radius).  A COUNTERBORE /
    // STEPPED BORE keeps its pilot bore going PAST the seat floor — there is a
    // coaxial cylinder of radius <= innerR reaching below the floor, AND a
    // separate DEEPER axis-normal floor plane (the pilot bottom).  Reject if
    // EITHER exists, so even a counterbore whose pilot is only slightly deeper
    // than the seat is rejected.
    for (const auto& cyl : cyls) {
        if (cyl.radius > floor.innerR + 0.2) continue;      // not the pilot/inner hole
        if (!coaxial(cyl)) continue;
        double lo, hi; cylRange(cyl, lo, hi);
        if (lo < floorAxial - 0.15) return out;             // a hole continues past the floor
    }

    json recovered = {
        { "center_x_mm",  0.0 },                        // foreign: on its own face
        { "center_y_mm",  0.0 },
        { "outer_dia_mm", 2.0 * floor.outerR },
        { "inner_dia_mm", 2.0 * floor.innerR },
        { "depth_mm",     depth },
        { "taper_deg",    0.0 },
        { "world_center", { floor.center.X(), floor.center.Y() } },
    };
    json matched = {
        { "source",          "geometry" },
        { "floor_face",      floor.faceIdx },
        { "inner_cyl_face",  innerFace },
        { "outer_cyl_face",  outerFace },           // -1 when outer wall is the case exterior
        { "groove_width_mm", floor.outerR - floor.innerR },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.9, matched });
    return out;
}

}  // namespace koocadcam::skill::annular_groove
