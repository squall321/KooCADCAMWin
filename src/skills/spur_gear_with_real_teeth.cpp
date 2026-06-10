// @lat: [[engine/skills#spur_gear_with_real_teeth]]

#include "spur_gear_with_real_teeth.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::spur_gear_with_real_teeth {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Derive tooth count N from module + pitch diameter (ISO 53).
int deriveToothCount(double pitch_dia_mm, double module_mm)
{
    if (module_mm <= 0.0) return 0;
    return static_cast<int>(std::round(pitch_dia_mm / module_mm));
}

// Build a single gullet (space between adjacent teeth) wedge cutter.  We
// approximate the gullet as a wedge bounded by two 5-point involute-flank
// polylines (one per tooth flank) extruded along the gear axis.
//
// The wedge spans:
//   - inner radius: root circle  r_root = pitch_r - 1.25 * m
//   - outer radius: addendum     r_outs = pitch_r + 1.00 * m
//   - angular sweep: half_tooth_angle either side of the gullet centerline
//
// For each tooth (centred at angle theta_t), the gullet to its RIGHT spans
// theta_t + half_tooth_angle .. theta_t + tooth_pitch_angle/2 + ...
// We simplify: gullet centerline = (i + 0.5) * (2*pi/N).
TopoDS_Shape buildGulletCutter(int i, int N, double moduleM, double pitchR,
                               double thickness, double overhang)
{
    const double toothAngle = 2.0 * M_PI / static_cast<double>(N);
    // Gullet centerline angle
    const double th0 = (static_cast<double>(i) + 0.5) * toothAngle;

    // Tooth thickness at the pitch circle = pi*m/2; convert to angular span
    // arc / r = pi*m / (2*pitchR).  Gullet angular span = toothAngle - that.
    const double toothArc   = M_PI * moduleM / 2.0;
    const double toothAng   = toothArc / pitchR;
    const double gulletAng  = std::max(toothAngle - toothAng, toothAngle * 0.3);

    const double rRoot = std::max(0.1, pitchR - 1.25 * moduleM);
    const double rOut  = pitchR + 1.00 * moduleM;     // addendum

    const double a0 = th0 - gulletAng / 2.0;
    const double a1 = th0 + gulletAng / 2.0;

    // 5-point polyline approximating involute flank (inner -> outer) on the
    // a0 (right tooth's left flank) side. Build inner-edge at root,
    // outer-edge at addendum, and two flank polylines.
    const int kFlankPts = 5;
    std::vector<gp_Pnt> leftFlank(kFlankPts);   // along a0
    std::vector<gp_Pnt> rightFlank(kFlankPts);  // along a1
    for (int k = 0; k < kFlankPts; ++k) {
        const double t  = static_cast<double>(k) / static_cast<double>(kFlankPts - 1);
        const double r  = rRoot + t * (rOut - rRoot);
        // Involute correction: as r grows, the flank angle shifts slightly
        // toward the tooth (gullet narrows toward root).  Approximate by
        // a sin-shaped half-correction up to 0.25 * gulletAng total.
        const double bias = 0.25 * gulletAng * std::sin(t * M_PI);
        const double aL   = a0 + bias;
        const double aR   = a1 - bias;
        leftFlank[k]  = gp_Pnt(r * std::cos(aL), r * std::sin(aL), -overhang);
        rightFlank[k] = gp_Pnt(r * std::cos(aR), r * std::sin(aR), -overhang);
    }

    // Build closed wire: leftFlank (root->out) + outer arc (right side) ->
    // rightFlank reversed (out->root) + inner arc closing.
    BRepBuilderAPI_MakeWire wire;
    for (int k = 0; k < kFlankPts - 1; ++k)
        wire.Add(BRepBuilderAPI_MakeEdge(leftFlank[k], leftFlank[k + 1]).Edge());
    // outer edge: leftFlank.last -> rightFlank.last
    wire.Add(BRepBuilderAPI_MakeEdge(leftFlank[kFlankPts - 1], rightFlank[kFlankPts - 1]).Edge());
    // right flank reversed
    for (int k = kFlankPts - 1; k > 0; --k)
        wire.Add(BRepBuilderAPI_MakeEdge(rightFlank[k], rightFlank[k - 1]).Edge());
    // closing inner edge: rightFlank[0] -> leftFlank[0]
    wire.Add(BRepBuilderAPI_MakeEdge(rightFlank[0], leftFlank[0]).Edge());

    if (!wire.IsDone())
        throw Standard_Failure("spur_gear_with_real_teeth: wire build failed");

    BRepBuilderAPI_MakeFace face(wire.Wire(), true);
    if (!face.IsDone())
        throw Standard_Failure("spur_gear_with_real_teeth: face build failed");

    BRepPrimAPI_MakePrism prism(face.Face(),
        gp_Vec(0.0, 0.0, thickness + 2.0 * overhang));
    prism.Build();
    if (!prism.IsDone())
        throw Standard_Failure("spur_gear_with_real_teeth: prism build failed");

    return prism.Shape();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.blank_dia_mm <= 0.0 || in.blank_thick_mm <= 0.0 ||
        in.module_mm <= 0.0 || in.pitch_dia_mm <= 0.0 || in.bore_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "spur_gear_with_real_teeth: positive blank/module/pitch/bore required");
        return r;
    }

    // ISO 53: module typically 0.5 .. 10 mm in general engineering.
    if (in.module_mm < 0.5 || in.module_mm > 10.0) {
        r.add("DFM-GEAR-MODULE", "error",
              "spur_gear_with_real_teeth: module " +
              std::to_string(in.module_mm) +
              " mm outside [0.5, 10] (ISO 53)");
    }

    const int N = deriveToothCount(in.pitch_dia_mm, in.module_mm);
    if (N < 6 || N > 200) {
        r.add("DFM-GEAR-COUNT", "error",
              "spur_gear_with_real_teeth: derived tooth count " +
              std::to_string(N) + " outside [6, 200]");
    }

    if (std::abs(in.pressure_angle_deg - 14.5) > 1e-3 &&
        std::abs(in.pressure_angle_deg - 20.0) > 1e-3 &&
        std::abs(in.pressure_angle_deg - 25.0) > 1e-3) {
        r.add("DFM-GEAR-PA", "error",
              "spur_gear_with_real_teeth: pressure_angle must be 14.5, 20, or 25 deg");
    }

    // OD = (N+2) * m must be <= blank_dia.
    const double od = (static_cast<double>(N) + 2.0) * in.module_mm;
    if (od > in.blank_dia_mm + 1e-3) {
        r.add("DFM-GEAR-OD", "error",
              "spur_gear_with_real_teeth: derived OD " + std::to_string(od) +
              " mm exceeds blank_dia " + std::to_string(in.blank_dia_mm));
    }

    if (in.bore_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "spur_gear_with_real_teeth: bore_dia < 0.8 mm");
    }
    if (in.bore_dia_mm >= in.pitch_dia_mm - 2.0 * in.module_mm) {
        r.add("DFM-GEAR-BORE", "error",
              "spur_gear_with_real_teeth: bore_dia " +
              std::to_string(in.bore_dia_mm) +
              " collides with tooth root circle");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "spur_gear_with_real_teeth DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const int    N         = deriveToothCount(in.pitch_dia_mm, in.module_mm);
    const double pitchR    = in.pitch_dia_mm / 2.0;
    const double boreR     = in.bore_dia_mm  / 2.0;
    const double Z         = in.blank_thick_mm;
    const double m         = in.module_mm;
    const double kOverhang = 0.05;

    // ── Tool 1: central through bore.
    const gp_Ax2 axBore(gp_Pnt(0, 0, -kOverhang), gp::DZ());
    const TopoDS_Shape boreTool =
        pr::cylinder(axBore, boreR, Z + 2.0 * kOverhang);

    // ── Tools 2..N+1: N gullet cutters distributed around pitch circle.
    std::vector<TopoDS_Shape> tools;
    tools.reserve(static_cast<size_t>(N) + 1);
    tools.push_back(boreTool);
    for (int i = 0; i < N; ++i) {
        tools.push_back(buildGulletCutter(i, N, m, pitchR, Z, kOverhang));
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), tools);

    // ── Signature
    const double od = (static_cast<double>(N) + 2.0) * m;
    const int subCount = 1 + N;     // bore + N gullets

    json params = {
        { "blank_dia_mm",       in.blank_dia_mm },
        { "blank_thick_mm",     in.blank_thick_mm },
        { "module_mm",          in.module_mm },
        { "pitch_dia_mm",       in.pitch_dia_mm },
        { "pressure_angle_deg", in.pressure_angle_deg },
        { "bore_dia_mm",        in.bore_dia_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     subCount },
        { "blank_dia_mm",         in.blank_dia_mm },
        { "blank_thick_mm",       in.blank_thick_mm },
        { "module_mm",            in.module_mm },
        { "pitch_dia_mm",         in.pitch_dia_mm },
        { "outer_dia_mm",         od },
        { "root_dia_mm",          in.pitch_dia_mm - 2.5 * m },
        { "num_teeth",            N },
        { "bore_dia_mm",          in.bore_dia_mm },
        { "pressure_angle_deg",   in.pressure_angle_deg },
        { "standard",             "ISO 53" },
    };

    ToolingMeta tooling;
    tooling.tool_type    = "gear_hob;drill";
    tooling.tool_dia_mm  = in.bore_dia_mm;
    tooling.tool_length_mm = in.blank_thick_mm + 10.0;
    tooling.tool_material = "hss";
    tooling.flute_count   = 0;        // hob is N/A
    tooling.cutting_speed_sfm = 100.0;
    tooling.feed_per_tooth_mm = 0.05;
    // Stock removed ≈ bore vol + N * gullet wedge vol (approx).
    const double rRoot = in.pitch_dia_mm / 2.0 - 1.25 * m;
    const double rOut  = od / 2.0;
    const double gulletWedgeVol = (rOut * rOut - rRoot * rRoot) * 0.5 *
                                  (2.0 * M_PI / N) * 0.6 * Z;  // 0.6 = gullet fraction
    tooling.stock_removed_mm3 =
        M_PI * boreR * boreR * Z + N * gulletWedgeVol;
    tooling.est_cycle_time_s = std::max(30.0, N * 2.0 + 60.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },    { "tool_dia_mm", in.bore_dia_mm } },
            { { "tool_type", "gear_hob" }, { "module_mm",   m },
              { "num_teeth",   N }, { "pressure_angle_deg", in.pressure_angle_deg } },
        } }
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::spur_gear_with_real_teeth applied: m={} N={} pitch={} OD={}",
                  m, N, in.pitch_dia_mm, od);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Strategy:
//   PRIMARY (geometric): count cylindrical faces near the axis (bore + blank
//   side) and count NON-cylindrical / non-planar tooth flank faces. The
//   number of tooth gullets ≈ count of small planar faces around the rim.
//   We approximate by counting planar side faces whose centers lie roughly
//   on the pitch circle.
//   FALLBACK (metadata): replay stored Input from feature history.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // ── Primary: geometric heuristic
    //
    // Strategy:
    //   1. Find all Z-axis cylindrical faces — the bore is the SMALLEST
    //      such radius, centered near the workpiece XY centroid.
    //   2. Walk every planar face and collect tooth-flank candidates: side
    //      faces (|n.Z| < 0.3) whose center is OFF the gear axis (offset
    //      > bore_r + module).
    //   3. Cluster those flank faces by ANGULAR sector around the gear
    //      axis.  Each cluster = one tooth gap (gullet).  N = cluster count.
    //
    // This is robust to OCCT splitting one flank into several sub-faces:
    //   sub-faces of the same flank land in the same angular bucket.
    std::vector<double> zCylRadii;  // candidate bores / blank
    std::vector<double> flankAngles;
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    const double xMid = 0.5 * (xMin + xMax);
    const double yMid = 0.5 * (yMin + yMax);

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFaceCylinder(i)) {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder cy = s.Cylinder();
            const gp_Dir d = cy.Axis().Direction();
            if (std::abs(std::abs(d.Z()) - 1.0) < 1e-2) {
                zCylRadii.push_back(cy.Radius());
            }
        } else if (wp.isFacePlanar(i)) {
            try {
                gp_Dir n = wp.faceNormal(i);
                if (std::abs(n.Z()) >= 0.3) continue;  // top/bottom, not flank
                const gp_Pnt c = wp.faceCenter(i);
                const double dx = c.X() - xMid;
                const double dy = c.Y() - yMid;
                const double r  = std::hypot(dx, dy);
                if (r < 1.0) continue;                 // on-axis — not a flank
                const double th = std::atan2(dy, dx);  // (-π, π]
                flankAngles.push_back(th);
            } catch (...) {}
        }
    }

    if (!zCylRadii.empty() && flankAngles.size() >= 12) {
        std::sort(zCylRadii.begin(), zCylRadii.end());
        const double boreR = zCylRadii.front();

        // Cluster flank angles into buckets ~ 1° wide.  Each tooth/gullet
        // contributes 2 flank faces (left + right) at angles ~ ±toothAngle/2;
        // OCCT may split each into sub-faces, so we cluster by 5° bins to
        // collapse them.  The cluster COUNT then equals 2 × num_teeth
        // (each tooth has 2 flanks) — divide by 2.
        std::sort(flankAngles.begin(), flankAngles.end());
        int clusterCount = 0;
        const double kClusterTol = 5.0 * M_PI / 180.0;   // 5°
        for (size_t k = 0; k < flankAngles.size(); ++k) {
            if (k == 0 ||
                std::abs(flankAngles[k] - flankAngles[k - 1]) > kClusterTol) {
                ++clusterCount;
            }
        }
        const int Nest = std::max(6, clusterCount / 2);
        if (Nest >= 6 && Nest <= 200) {
            const double od = std::max({ xMax - xMin, yMax - yMin });
            const double pitchDia = od * static_cast<double>(Nest) /
                                    (static_cast<double>(Nest) + 2.0);
            const double moduleEst = pitchDia / static_cast<double>(Nest);

            // Confidence: 0.65 base, +0.05 if bore is small (< 25% of OD),
            // +0.05 if cluster count is at least 12 (typical spur gear).
            double confidence = 0.65;
            if (boreR < od * 0.25) confidence += 0.05;
            if (clusterCount >= 12) confidence += 0.05;

            json recovered = {
                { "blank_dia_mm",        od },
                { "blank_thick_mm",      thickness },
                { "module_mm",           moduleEst },
                { "pitch_dia_mm",        pitchDia },
                { "pressure_angle_deg",  20.0 },
                { "bore_dia_mm",         2.0 * boreR },
            };
            json matched = {
                { "source",                "geometric_primary" },
                { "z_cyl_face_count",      static_cast<int>(zCylRadii.size()) },
                { "flank_face_count",      static_cast<int>(flankAngles.size()) },
                { "angular_cluster_count", clusterCount },
                { "estimated_num_teeth",   Nest },
            };
            out.push_back(RecognizedFeature{
                kSkillId, recovered, confidence, matched
            });
        }
    }

    // ── Fallback: metadata replay
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",           "feature_history_replay" },
            { "subfeature_count", f.pattern.value("subfeature_count", 0) },
            { "num_teeth",        f.pattern.value("num_teeth", 0) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::spur_gear_with_real_teeth
