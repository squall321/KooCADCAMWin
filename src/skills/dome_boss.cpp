// @lat: [[engine/skills#dome_boss]]

#include "dome_boss.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::dome_boss {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Volume of a spherical cap of base radius r and rise h.  V = pi*h*(3r^2 + h^2)/6.
double sphericalCapVolume(double r, double h)
{
    return M_PI * h * (3.0 * r * r + h * h) / 6.0;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.base_radius_mm <= 0.0)
        r.add("DFM-INPUT", "error", "dome_boss: base_radius_mm must be > 0");
    if (in.height_mm <= 0.0)
        r.add("DFM-INPUT", "error", "dome_boss: height_mm must be > 0");
    if (in.sections < 3)
        r.add("DFM-INPUT", "error", "dome_boss: sections must be >= 3");
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "dome_boss DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceIdOpt = wp.resolve(in.entry_face);
    if (!faceIdOpt) throw SkillError("dome_boss: entry_face datum unresolved");
    const int faceId = *faceIdOpt;
    const gp_Pnt origin = wp.faceCenter(faceId);
    const gp_Dir normal = wp.faceNormal(faceId);

    // In-plane X axis (anything orthogonal to the normal), to place the dome
    // centre at the requested face-local (x,y).
    gp_Vec vx(gp::DX());
    if (std::abs(vx.Dot(gp_Vec(normal))) > 0.99) vx = gp_Vec(gp::DY());
    vx = vx - gp_Vec(normal) * vx.Dot(gp_Vec(normal));
    if (vx.Magnitude() < 1e-9) throw SkillError("dome_boss: cannot build face X-axis");
    vx.Normalize();
    const gp_Vec vy = gp_Vec(normal).Crossed(vx);

    const gp_Pnt base(
        origin.X() + in.center_x_mm * vx.X() + in.center_y_mm * vy.X(),
        origin.Y() + in.center_x_mm * vx.Y() + in.center_y_mm * vy.Y(),
        origin.Z() + in.center_x_mm * vx.Z() + in.center_y_mm * vy.Z());

    // Dome axis = outward face normal; X = the in-plane X we built.
    const gp_Ax2 ax(base, normal, gp_Dir(vx));
    const TopoDS_Shape dome =
        pr::domeSolid(ax, in.base_radius_mm, in.height_mm, in.sections);
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), dome);

    const double vol = sphericalCapVolume(in.base_radius_mm, in.height_mm);

    json params = {
        { "entry_face_id",  faceId },
        { "center_x_mm",    in.center_x_mm },
        { "center_y_mm",    in.center_y_mm },
        { "base_radius_mm", in.base_radius_mm },
        { "height_mm",      in.height_mm },
        { "sections",       in.sections },
        { "face_normal",    { normal.X(), normal.Y(), normal.Z() } },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "is_compound",        true },
        { "base_radius_mm",     in.base_radius_mm },
        { "height_mm",          in.height_mm },
        { "derived_volume_mm3", vol },
        { "face_normal",        { normal.X(), normal.Y(), normal.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "form_die;freeform_add";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.height_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = -vol;   // material ADDED
    tooling.est_cycle_time_s  = std::max(2.0, in.height_mm * 0.2 + in.base_radius_mm * 0.05);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(validateOrHeal(newShape, "dome_boss"),
                                             wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::dome_boss applied: r={} h={} vol={} faces {}->{}",
                  in.base_radius_mm, in.height_mm, vol,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // metadata replay (same-session: cap-exempt full confidence).
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

    // Geometric path B (foreign geometry): a dome built by pr::domeSolid presents
    // one or more BSpline/Bezier lofted cap faces (the curved side wall) sitting
    // on a flat base.  Collect the freeform faces, fit a base circle + rise from
    // their bounding geometry, and gate on the spherical-cap volume.
    std::vector<int> bsplineFaces;
    for (int i = 0; i < wp.faceCount(); ++i)
        if (wp.isFaceBSpline(i)) bsplineFaces.push_back(i);
    if (bsplineFaces.empty()) return out;

    // ThruSections makes the cap a BSpline surface whose base ring is ALSO a
    // BSpline edge (no GeomAbs_Circle to read), and the surface poles bulge
    // outside the true base radius — so neither edges nor a bounding box measure
    // the dome cleanly.  Instead SAMPLE the cap surface on a (u,v) grid into a
    // point cloud and fit (axis, base centre+radius, rise) from it:
    //   - axis = the cloud's smallest-spread principal direction (the base-plane
    //     normal: the cap is wide across the base, thin along the rise),
    //   - base ring = the points at the minimum axial coordinate (lowest slice),
    //   - rise = axial span; base_radius = mean in-plane distance of the base ring.
    std::vector<gp_Pnt> cloud;
    double capArea = 0.0;
    for (int fid : bsplineFaces) {
        capArea += wp.faceArea(fid);
        BRepAdaptor_Surface s(wp.face(fid));
        const double u0 = s.FirstUParameter(), u1 = s.LastUParameter();
        const double v0 = s.FirstVParameter(), v1 = s.LastVParameter();
        const int N = 12;
        for (int iu = 0; iu <= N; ++iu)
            for (int iv = 0; iv <= N; ++iv) {
                const double u = u0 + (u1 - u0) * iu / N;
                const double v = v0 + (v1 - v0) * iv / N;
                cloud.push_back(s.Value(u, v));
            }
    }
    if (cloud.size() < 8) return out;

    gp_Pnt centroid(0, 0, 0);
    for (const auto& p : cloud) centroid.SetXYZ(centroid.XYZ() + p.XYZ());
    centroid.SetXYZ(centroid.XYZ() / static_cast<double>(cloud.size()));

    // Per-axis spread → the dome axis is the world axis of LEAST spread (the cap
    // is a shallow shell: wide across the base, short along the rise).
    double sx = 0, sy = 0, sz = 0;
    for (const auto& p : cloud) {
        sx += (p.X() - centroid.X()) * (p.X() - centroid.X());
        sy += (p.Y() - centroid.Y()) * (p.Y() - centroid.Y());
        sz += (p.Z() - centroid.Z()) * (p.Z() - centroid.Z());
    }
    gp_Dir domeAxis = (sz <= sx && sz <= sy) ? gp_Dir(0, 0, 1)
                    : (sx <= sy)             ? gp_Dir(1, 0, 0)
                                             : gp_Dir(0, 1, 0);

    // Axial coordinate of each point (relative to centroid).
    double aMin = 1e30, aMax = -1e30;
    for (const auto& p : cloud) {
        const double a = gp_Vec(centroid, p).Dot(gp_Vec(domeAxis));
        aMin = std::min(aMin, a);
        aMax = std::max(aMax, a);
    }
    const double rise = aMax - aMin;
    if (rise < 0.05) return out;

    // Base ring = points within 10% of the rise from the lowest slice; their
    // mean radial distance from the axis (through centroid) is the base radius.
    const double band = aMin + 0.10 * rise;
    double rSum = 0.0; int rN = 0;
    gp_Pnt baseCenter(0, 0, 0); int bN = 0;
    for (const auto& p : cloud) {
        const double a = gp_Vec(centroid, p).Dot(gp_Vec(domeAxis));
        if (a > band) continue;
        gp_Vec rel(centroid, p);
        const gp_Vec radial = rel - gp_Vec(domeAxis) * a;
        rSum += radial.Magnitude(); ++rN;
        baseCenter.SetXYZ(baseCenter.XYZ() + p.XYZ()); ++bN;
    }
    if (rN < 3) return out;
    const double baseR = rSum / rN;
    if (baseR < 0.1) return out;
    if (bN > 0) baseCenter.SetXYZ(baseCenter.XYZ() / static_cast<double>(bN));

    // Sphere radius of the fitted cap: Rs = (r^2 + h^2) / 2h.
    const double Rs = (baseR * baseR + rise * rise) / (2.0 * rise);

    // SPECIFICITY gate 1 (sphere RESIDUAL — the discriminating test): every
    // sampled point of a TRUE spherical cap lies on a sphere of radius Rs whose
    // centre sits on the dome axis a distance Rs below the apex.  Measure the
    // RMS deviation of |p - centre| from Rs; a real cap has a tiny residual, a
    // generic loft / NURBS bump / fillet blend (which also surfaces as BSpline)
    // does NOT.  This is what justifies the precise tier — an area-magnitude
    // band alone (gate 2 below) is too loose to reject foreign freeform faces.
    const double apexA = aMax;                       // apex axial coord (rel. centroid)
    const gp_Vec axV(domeAxis);
    const gp_Pnt sphereCentre(
        centroid.X() + axV.X() * (apexA - Rs),
        centroid.Y() + axV.Y() * (apexA - Rs),
        centroid.Z() + axV.Z() * (apexA - Rs));
    double resSq = 0.0;
    for (const auto& p : cloud) {
        const double dev = p.Distance(sphereCentre) - Rs;
        resSq += dev * dev;
    }
    const double rms = std::sqrt(resSq / static_cast<double>(cloud.size()));
    if (rms > 0.08 * Rs)
        return out;   // points don't lie on a sphere → not a spherical-cap dome

    // SPECIFICITY gate 2 (area magnitude, belt-and-braces): the cap surface area
    // must be in the ballpark of a spherical cap's lateral area 2*pi*Rs*h.
    const double expectArea = 2.0 * M_PI * Rs * rise;
    if (capArea < 0.5 * expectArea || capArea > 1.6 * expectArea)
        return out;   // not a spherical cap (extent mismatch)

    json recovered = {
        { "center_x_mm",    0.0 },              // foreign: centred on its own face
        { "center_y_mm",    0.0 },
        { "base_radius_mm", baseR },
        { "height_mm",      rise },
        { "sections",       6 },
        { "world_center",   { baseCenter.X(), baseCenter.Y() } },
    };
    json matched = {
        { "source",          "geometry" },
        { "bspline_faces",   static_cast<int>(bsplineFaces.size()) },
        { "cap_area_mm2",    capArea },
        { "expected_area",   expectArea },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.9, matched });
    return out;
}

}  // namespace koocadcam::skill::dome_boss
