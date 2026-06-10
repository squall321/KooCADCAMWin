// @lat: [[engine/skills#internal_gear]]

#include "internal_gear.hpp"

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

namespace koocadcam::skill::internal_gear {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr int kFlankSamples = 16;

// For an INTERNAL gear, the teeth point INWARD (toward the axis), so the
// gullet (between two teeth) extends from the smallest radius (which is
// the tip of the tooth, near the bore) OUTWARD to a "ceiling" beyond the
// addendum.  Geometry is the mirror of external: tooth-tip is at r_pitch
// − module, root (back face of tooth) is at r_pitch + 1.25·module.
//
// For an internal involute gear:
//   r_pitch unchanged
//   r_tip   = r_pitch − module       (tooth tip is the SMALLEST radius)
//   r_root  = r_pitch + 1.25·module  (root is OUTSIDE the pitch circle)
TopoDS_Shape buildInternalGulletPrism(double moduleM, int z,
                                      double pressureAngleRad,
                                      double faceWidth, double overhang)
{
    const double rPitch = moduleM * static_cast<double>(z) / 2.0;
    const double rBase  = rPitch * std::cos(pressureAngleRad);
    const double rTipIn = std::max(rBase * 0.95, rPitch - moduleM);    // inner tip
    const double rRootOut = rPitch + 1.25 * moduleM;                   // outer root

    const double tMax = std::sqrt(std::max(0.0,
        (rRootOut / rBase) * (rRootOut / rBase) - 1.0));
    const double invAlpha = std::tan(pressureAngleRad) - pressureAngleRad;
    const double halfToothAtBase =
        M_PI / (2.0 * static_cast<double>(z)) + invAlpha;
    const double zPlane = -overhang;

    // The involute is the same curve as external — flanks of internal gear
    // are also involutes (running from rBase out to rRootOut).
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

    // INNER side: the gullet pinches IN past the tooth tip toward the
    // bore.  We cap at r = rTipIn − overhang so the cut reaches the bore.
    const double rFloor = std::max(0.5, rTipIn - 2.0);

    const gp_Pnt rightBase = rightFlank.front();
    const gp_Pnt leftBase  = leftFlank.front();
    const double angRBase = std::atan2(rightBase.Y(), rightBase.X());
    const double angLBase = std::atan2(leftBase.Y(),  leftBase.X());
    const gp_Pnt rightFloor(rFloor * std::cos(angRBase),
                            rFloor * std::sin(angRBase), zPlane);
    const gp_Pnt leftFloor (rFloor * std::cos(angLBase),
                            rFloor * std::sin(angLBase), zPlane);
    const gp_Pnt centerFloor(rFloor, 0.0, zPlane);

    // OUTER side at root: rightFlank.last and leftFlank.last sit on rRootOut.
    // Close the gullet with a straight chord between them.
    const gp_Pnt rightLast = rightFlank[kFlankSamples - 1];
    const gp_Pnt leftLast  = leftFlank[kFlankSamples - 1];
    const gp_Pnt outerMid(rRootOut, 0.0, zPlane);

    BRepBuilderAPI_MakeWire wire;
    // 1. inner floor (toward axis): leftFloor → centerFloor → rightFloor
    wire.Add(BRepBuilderAPI_MakeEdge(leftFloor, centerFloor).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(centerFloor, rightFloor).Edge());
    // 2. up from rightFloor → rightBase
    wire.Add(BRepBuilderAPI_MakeEdge(rightFloor, rightBase).Edge());
    // 3. right flank (involute) outward
    for (int k = 0; k < kFlankSamples - 1; ++k)
        wire.Add(BRepBuilderAPI_MakeEdge(rightFlank[k], rightFlank[k + 1]).Edge());
    // 4. close across outer root: rightLast → outerMid → leftLast
    wire.Add(BRepBuilderAPI_MakeEdge(rightLast, outerMid).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(outerMid, leftLast).Edge());
    // 5. left flank reversed (outward → inward)
    for (int k = kFlankSamples - 1; k > 0; --k)
        wire.Add(BRepBuilderAPI_MakeEdge(leftFlank[k], leftFlank[k - 1]).Edge());
    // 6. close from leftBase → leftFloor
    wire.Add(BRepBuilderAPI_MakeEdge(leftBase, leftFloor).Edge());

    if (!wire.IsDone())
        throw Standard_Failure("internal_gear: gullet wire build failed");

    BRepBuilderAPI_MakeFace face(wire.Wire(), true);
    if (!face.IsDone())
        throw Standard_Failure("internal_gear: gullet face build failed");

    BRepPrimAPI_MakePrism prism(face.Face(),
        gp_Vec(0.0, 0.0, faceWidth + 2.0 * overhang));
    prism.Build();
    if (!prism.IsDone())
        throw Standard_Failure("internal_gear: gullet prism build failed");

    return prism.Shape();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.module_mm <= 0.3 || in.module_mm > 50.0) {
        r.add("DFM-GEAR-MOD", "error",
              "internal_gear: module_mm out of (0.3, 50] mm");
    }
    if (in.teeth_count < 8 || in.teeth_count > 200) {
        r.add("DFM-GEAR-CT", "error",
              "internal_gear: teeth_count out of [8, 200]");
    }
    if (in.face_width_mm <= 0.0) {
        r.add("DFM-GEAR-FW", "error",
              "internal_gear: face_width_mm must be > 0");
    }
    if (in.pressure_angle_deg < 14.5 || in.pressure_angle_deg > 30.0) {
        r.add("DFM-GEAR-PA", "error",
              "internal_gear: pressure_angle out of [14.5, 30] deg");
    }
    if (in.module_mm > 0.0 && in.teeth_count > 0) {
        const double rPitch  = in.module_mm * in.teeth_count / 2.0;
        const double rRootO  = rPitch + 1.25 * in.module_mm;
        const double minOD   = 2.0 * (rRootO + 2.0 * in.module_mm);  // 2·m wall
        if (in.ring_outer_dia_mm + 1e-6 < minOD) {
            r.add("DFM-GEAR-OD", "error",
                  "internal_gear: ring_outer_dia " +
                  std::to_string(in.ring_outer_dia_mm) +
                  " smaller than required " + std::to_string(minOD) +
                  " (need 2·m wall thickness)");
        }
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "internal_gear DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double m       = in.module_mm;
    const int    z       = in.teeth_count;
    const double rPitch  = m * z / 2.0;
    const double rTipIn  = rPitch - m;
    const double rRootO  = rPitch + 1.25 * m;
    const double rBore   = std::max(0.5, rTipIn - 0.5);   // bore = inside of teeth
    const double Z       = in.face_width_mm;
    const double alpha   = in.pressure_angle_deg * M_PI / 180.0;
    const double kOver   = 0.05;

    // Step 1: bore the centre hole.  Bore radius = rBore (just inside r_tip_inner).
    // We need a base shape: assume incoming wp is a solid ring/cylinder
    // matching ring_outer_dia.  First cut a central bore, then cut z gullets.
    std::vector<TopoDS_Shape> tools;
    tools.reserve(static_cast<size_t>(z + 1));

    const gp_Ax2 boreAx(gp_Pnt(0, 0, -kOver), gp::DZ());
    tools.push_back(pr::cylinder(boreAx, rBore, Z + 2.0 * kOver));

    // Step 2: z internal gullet cutters.
    const TopoDS_Shape seedGullet =
        buildInternalGulletPrism(m, z, alpha, Z, kOver);
    const gp_Ax1 zAxis(gp_Pnt(0, 0, 0), gp::DZ());
    const double dTheta = 2.0 * M_PI / static_cast<double>(z);
    for (int i = 0; i < z; ++i) {
        if (i == 0) {
            tools.push_back(seedGullet);
            continue;
        }
        gp_Trsf rot;
        rot.SetRotation(zAxis, static_cast<double>(i) * dTheta);
        BRepBuilderAPI_Transform xf(seedGullet, rot, true);
        if (!xf.IsDone())
            throw SkillError("internal_gear: cutter rotation failed");
        tools.push_back(xf.Shape());
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), tools);

    const double gulletVol =
        0.5 * (rRootO * rRootO - rTipIn * rTipIn) * (dTheta * 0.5) * Z;
    const double boreVol = M_PI * rBore * rBore * Z;
    const double volRemoved = boreVol + z * gulletVol;

    json params = {
        { "module_mm",          m },
        { "teeth_count",        z },
        { "face_width_mm",      Z },
        { "pressure_angle_deg", in.pressure_angle_deg },
        { "ring_outer_dia_mm",  in.ring_outer_dia_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_gear",               true },
        { "is_internal",           true },
        { "is_compound",           true },
        { "subfeature_count",      z + 1 },
        { "module_mm",             m },
        { "teeth_count",           z },
        { "pressure_angle_deg",    in.pressure_angle_deg },
        { "pitch_dia_mm",          2.0 * rPitch },
        { "tip_dia_mm",            2.0 * rTipIn },
        { "root_dia_mm",           2.0 * rRootO },
        { "ring_outer_dia_mm",     in.ring_outer_dia_mm },
        { "face_width_mm",         Z },
        { "derived_volume_removed", volRemoved },
        { "standard",              "ISO 53" },
        { "profile",               "involute_sampled_internal" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "internal_gear_shaper";
    tooling.tool_dia_mm       = std::max(2.0 * rBore - 2.0 * m, 1.0);
    tooling.tool_length_mm    = Z * 2.0 + 10.0;
    tooling.tool_material     = "hss";
    tooling.flute_count       = z;     // shaper cutter has same tooth count
    tooling.cutting_speed_sfm = 60.0;
    tooling.feed_per_tooth_mm = 0.025;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(120.0, z * 3.5 + 60.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::internal_gear applied: m={} z={} OD={} faces {}→{}",
                  m, z, in.ring_outer_dia_mm, wp.faceCount(), wpNew->faceCount());

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
    int zCylCount = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFaceCylinder(i)) {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder cy = s.Cylinder();
            const gp_Dir d = cy.Axis().Direction();
            if (std::abs(std::abs(d.Z()) - 1.0) < 1e-2) ++zCylCount;
        } else if (wp.isFacePlanar(i)) {
            try {
                gp_Dir n = wp.faceNormal(i);
                if (std::abs(n.Z()) >= 0.3) continue;
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
            const double moduleEst = od / (static_cast<double>(zEst) * 1.5);
            json recovered = {
                { "module_mm",          moduleEst },
                { "teeth_count",        zEst },
                { "face_width_mm",      zMax - zMin },
                { "pressure_angle_deg", 20.0 },
                { "ring_outer_dia_mm",  od },
            };
            json matched = {
                { "source",                "geometric_primary" },
                { "z_cyl_face_count",      zCylCount },
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
            { { "source",      "feature_history_replay" },
              { "teeth_count", f.pattern.value("teeth_count", 0) },
              { "is_internal", true } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::internal_gear
