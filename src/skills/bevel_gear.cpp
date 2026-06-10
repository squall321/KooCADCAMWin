// @lat: [[engine/skills#bevel_gear]]

#include "bevel_gear.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::bevel_gear {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr int kFlankSamples = 16;

// Build one tapered tooth-gap cutter sized for the OUTER (large) end of
// the cone.  The cutter is a simple prism (slab) — we approximate the
// taper by using the LARGE-end profile and clipping with the cone in
// post-process is too expensive; instead we use a single prism that's
// generous enough at outer-end yet still cleanly cuts at inner-end.
//
// For a true bevel we'd build z cutters with varying scaled profiles per
// axial slab; here we use 2 slabs (outer + inner) so the cut tapers.
TopoDS_Shape buildBevelGulletSlab(double moduleM, int z,
                                  double pressureAngleRad,
                                  double scale,        // taper factor at this slab
                                  double baseZ,
                                  double sliceZ,
                                  double overhang)
{
    const double mScaled = moduleM * scale;
    const double rPitch  = mScaled * static_cast<double>(z) / 2.0;
    const double rBase   = rPitch * std::cos(pressureAngleRad);
    const double rTip    = rPitch + mScaled;
    const double rRoot   = std::max(rBase * 0.95, rPitch - 1.25 * mScaled);

    const double tMax = std::sqrt(std::max(0.0,
        (rTip / rBase) * (rTip / rBase) - 1.0));
    const double invAlpha = std::tan(pressureAngleRad) - pressureAngleRad;
    const double halfToothAtBase =
        M_PI / (2.0 * static_cast<double>(z)) + invAlpha;

    const double zPlane = baseZ - overhang;

    std::vector<gp_Pnt> rightFlank;
    rightFlank.reserve(kFlankSamples);
    for (int k = 0; k < kFlankSamples; ++k) {
        const double t = tMax * static_cast<double>(k) / static_cast<double>(kFlankSamples - 1);
        const double xi = rBase * (std::cos(t) + t * std::sin(t));
        const double yi = rBase * (std::sin(t) - t * std::cos(t));
        const double c = std::cos(halfToothAtBase);
        const double s = std::sin(halfToothAtBase);
        rightFlank.emplace_back(xi * c - yi * s, xi * s + yi * c, zPlane);
    }
    std::vector<gp_Pnt> leftFlank;
    leftFlank.reserve(kFlankSamples);
    for (const auto& p : rightFlank)
        leftFlank.emplace_back(p.X(), -p.Y(), zPlane);

    const double rBaseRight_x = rBase * std::cos(halfToothAtBase);
    const double rBaseRight_y = rBase * std::sin(halfToothAtBase);
    const double rRootRight_x = rRoot * std::cos(halfToothAtBase);
    const double rRootRight_y = rRoot * std::sin(halfToothAtBase);
    const double rRootLeft_x  = rRoot * std::cos(-halfToothAtBase);
    const double rRootLeft_y  = rRoot * std::sin(-halfToothAtBase);
    const double rBaseLeft_x  = rBase * std::cos(-halfToothAtBase);
    const double rBaseLeft_y  = rBase * std::sin(-halfToothAtBase);

    const double rCeil = rTip + 2.0;
    const gp_Pnt rightLast = rightFlank[kFlankSamples - 1];
    const gp_Pnt leftLast  = leftFlank[kFlankSamples - 1];
    const double angR = std::atan2(rightLast.Y(), rightLast.X());
    const double angL = std::atan2(leftLast.Y(),  leftLast.X());
    const gp_Pnt rightCeil(rCeil * std::cos(angR), rCeil * std::sin(angR), zPlane);
    const gp_Pnt leftCeil (rCeil * std::cos(angL), rCeil * std::sin(angL), zPlane);
    const gp_Pnt topCeil  (rCeil, 0.0, zPlane);

    BRepBuilderAPI_MakeWire wire;
    wire.Add(BRepBuilderAPI_MakeEdge(
        gp_Pnt(rRootLeft_x, rRootLeft_y, zPlane),
        gp_Pnt(rRoot,        0.0,        zPlane)).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(
        gp_Pnt(rRoot, 0.0, zPlane),
        gp_Pnt(rRootRight_x, rRootRight_y, zPlane)).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(
        gp_Pnt(rRootRight_x, rRootRight_y, zPlane),
        gp_Pnt(rBaseRight_x, rBaseRight_y, zPlane)).Edge());
    for (int k = 0; k < kFlankSamples - 1; ++k)
        wire.Add(BRepBuilderAPI_MakeEdge(rightFlank[k], rightFlank[k + 1]).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(rightLast, rightCeil).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(rightCeil, topCeil).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(topCeil,   leftCeil).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(leftCeil,  leftLast).Edge());
    for (int k = kFlankSamples - 1; k > 0; --k)
        wire.Add(BRepBuilderAPI_MakeEdge(leftFlank[k], leftFlank[k - 1]).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(
        gp_Pnt(rBaseLeft_x, rBaseLeft_y, zPlane),
        gp_Pnt(rRootLeft_x, rRootLeft_y, zPlane)).Edge());

    if (!wire.IsDone())
        throw Standard_Failure("bevel_gear: slab wire build failed");

    BRepBuilderAPI_MakeFace face(wire.Wire(), true);
    if (!face.IsDone())
        throw Standard_Failure("bevel_gear: slab face build failed");

    BRepPrimAPI_MakePrism prism(face.Face(),
        gp_Vec(0.0, 0.0, sliceZ + 2.0 * overhang));
    prism.Build();
    if (!prism.IsDone())
        throw Standard_Failure("bevel_gear: slab prism build failed");

    return prism.Shape();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.module_mm <= 0.3 || in.module_mm > 50.0) {
        r.add("DFM-GEAR-MOD", "error",
              "bevel_gear: module_mm out of (0.3, 50] mm");
    }
    if (in.teeth_count < 8 || in.teeth_count > 200) {
        r.add("DFM-GEAR-CT", "error",
              "bevel_gear: teeth_count out of [8, 200]");
    }
    if (in.face_width_mm <= 0.0) {
        r.add("DFM-GEAR-FW", "error",
              "bevel_gear: face_width_mm must be > 0");
    }
    if (in.pressure_angle_deg < 14.5 || in.pressure_angle_deg > 30.0) {
        r.add("DFM-GEAR-PA", "error",
              "bevel_gear: pressure_angle out of [14.5, 30] deg");
    }
    if (in.cone_angle_deg < 10.0 || in.cone_angle_deg > 80.0) {
        r.add("DFM-GEAR-CONE", "error",
              "bevel_gear: cone_angle " +
              std::to_string(in.cone_angle_deg) + " out of [10, 80] deg");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bevel_gear DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double m       = in.module_mm;
    const int    z       = in.teeth_count;
    const double rPitch  = m * z / 2.0;
    const double rTip    = rPitch + m;
    const double Z       = in.face_width_mm;
    const double alpha   = in.pressure_angle_deg * M_PI / 180.0;
    const double cone    = in.cone_angle_deg * M_PI / 180.0;
    const double kOver   = 0.05;

    // Taper: outer end at z = 0 uses scale = 1.0; inner end at z = Z uses
    // scale = 1 − (Z·tan(coneHalf)) / rPitch, clamped to [0.2, 1.0].
    // (cone_angle is the pitch cone half-angle in degrees.)
    const double coneShrink = Z * std::tan(cone) / rPitch;
    const double scaleInner = std::clamp(1.0 - coneShrink, 0.2, 1.0);
    const double scaleOuter = 1.0;

    // Two slabs: outer (z = 0..Z/2 with scale ≈ scaleOuter) and
    // inner (z = Z/2..Z with scale ≈ midpoint between outer/inner).
    const int nSlice = 2;
    const double dZ  = Z / static_cast<double>(nSlice);
    const gp_Ax1 zAxis(gp_Pnt(0, 0, 0), gp::DZ());
    const double dTheta = 2.0 * M_PI / static_cast<double>(z);

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(static_cast<size_t>(z * nSlice));
    for (int s = 0; s < nSlice; ++s) {
        const double baseZ = static_cast<double>(s) * dZ;
        const double tMid  = (static_cast<double>(s) + 0.5) / static_cast<double>(nSlice);
        const double scale = scaleOuter * (1.0 - tMid) + scaleInner * tMid;
        const TopoDS_Shape seed =
            buildBevelGulletSlab(m, z, alpha, scale, baseZ, dZ, kOver);
        for (int i = 0; i < z; ++i) {
            gp_Trsf rot;
            rot.SetRotation(zAxis, static_cast<double>(i) * dTheta);
            BRepBuilderAPI_Transform xf(seed, rot, true);
            if (!xf.IsDone())
                throw SkillError("bevel_gear: cutter rotation failed");
            cutters.push_back(xf.Shape());
        }
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    const double avgScale = 0.5 * (scaleOuter + scaleInner);
    const double rTipAvg   = rTip   * avgScale;
    const double rRootAvg  = (rPitch - 1.25 * m) * avgScale;
    const double gulletVol =
        0.5 * (rTipAvg * rTipAvg - rRootAvg * rRootAvg) * (dTheta * 0.5) * Z;
    const double volRemoved = z * gulletVol;

    json params = {
        { "module_mm",          m },
        { "teeth_count",        z },
        { "face_width_mm",      Z },
        { "cone_angle_deg",     in.cone_angle_deg },
        { "pressure_angle_deg", in.pressure_angle_deg },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_gear",               true },
        { "is_compound",           true },
        { "subfeature_count",      z * nSlice },
        { "module_mm",             m },
        { "teeth_count",           z },
        { "cone_angle_deg",        in.cone_angle_deg },
        { "pressure_angle_deg",    in.pressure_angle_deg },
        { "pitch_dia_outer_mm",    2.0 * rPitch },
        { "pitch_dia_inner_mm",    2.0 * rPitch * scaleInner },
        { "face_width_mm",         Z },
        { "derived_volume_removed", volRemoved },
        { "standard",              "ISO 23509" },
        { "profile",               "involute_sampled_bevel" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "bevel_hob";
    tooling.tool_dia_mm       = m * 5.0;
    tooling.tool_length_mm    = Z * 2.0 + 10.0;
    tooling.tool_material     = "hss";
    tooling.flute_count       = 12;
    tooling.cutting_speed_sfm = 80.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(80.0, z * 2.5 + 50.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::bevel_gear applied: m={} z={} cone={}° faces {}→{}",
                  m, z, in.cone_angle_deg, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double xMid = 0.5 * (xMin + xMax);
    const double yMid = 0.5 * (yMin + yMax);

    std::vector<double> flankAngles;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFacePlanar(i)) {
            try {
                gp_Dir n = wp.faceNormal(i);
                if (std::abs(n.Z()) >= 0.5) continue;  // bevel flanks often slope
                const gp_Pnt c = wp.faceCenter(i);
                const double dx = c.X() - xMid;
                const double dy = c.Y() - yMid;
                if (std::hypot(dx, dy) < 1.0) continue;
                flankAngles.push_back(std::atan2(dy, dx));
            } catch (...) {}
        }
    }

    if (flankAngles.size() >= 16) {
        std::sort(flankAngles.begin(), flankAngles.end());
        int clusterCount = 0;
        const double tol = 5.0 * M_PI / 180.0;
        for (size_t k = 0; k < flankAngles.size(); ++k) {
            if (k == 0 || std::abs(flankAngles[k] - flankAngles[k - 1]) > tol)
                ++clusterCount;
        }
        const int zEst = std::max(8, clusterCount / 2);
        if (zEst >= 8 && zEst <= 200) {
            const double od = std::max({ xMax - xMin, yMax - yMin });
            const double moduleEst = od / (static_cast<double>(zEst) + 2.0);
            json recovered = {
                { "module_mm",          moduleEst },
                { "teeth_count",        zEst },
                { "face_width_mm",      zMax - zMin },
                { "cone_angle_deg",     45.0 },
                { "pressure_angle_deg", 20.0 },
            };
            json matched = {
                { "source",                "geometric_primary" },
                { "flank_face_count",      static_cast<int>(flankAngles.size()) },
                { "angular_cluster_count", clusterCount },
                { "estimated_teeth_count", zEst },
            };
            out.push_back(RecognizedFeature{
                kSkillId, recovered, 0.6, matched
            });
        }
    }

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            { { "source",          "feature_history_replay" },
              { "teeth_count",     f.pattern.value("teeth_count", 0) },
              { "cone_angle_deg",  f.pattern.value("cone_angle_deg", 0.0) } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::bevel_gear
