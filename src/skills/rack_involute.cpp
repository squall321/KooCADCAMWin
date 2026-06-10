// @lat: [[engine/skills#rack_involute]]

#include "rack_involute.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
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

namespace koocadcam::skill::rack_involute {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Build one rack tooth-gap cutter, centred at X=0.
// The gap is a trapezoid in the XZ-plane (top wide, bottom narrow) with
// flank angle = pressure_angle from vertical, extruded along Y by
// face_width.
//
// Standard rack geometry (ISO 53):
//   addendum  = m
//   dedendum  = 1.25·m
//   total tooth height h = 2.25·m
//   tooth thickness at pitch line = π·m / 2 (top of gap = π·m / 2 too)
//   bottom of gap (at root) is narrower by 2·dedendum·tan(α)
//   But the GAP between teeth has the INVERSE geometry: at top (above pitch
//   line) the gap is wider; at bottom (below pitch line) narrower.
//
// Layout: rack is a bar sitting with its top face at z = (2·m), bottom at
// z = 0.  The pitch line is at z = m.  Each tooth gap is cut DOWNWARD
// from the top into the bar:
//   - top opening width (at z = 2·m): π·m / 2 + 2·(addendum)·tan(α) = π·m/2 + 2·m·tan(α)
//     (this is the WIDE end of the trapezoid — the gap mouth)
//   - bottom width (at z = m − 1.25·m = −0.25·m, below the bar): we cap at
//     z = −overhang so the cut goes through.  Use the rack-flank formula:
//     gapBottomW = π·m/2 − 2·dedendum·tan(α)  (narrower)
//
// For simplicity our bar uses depth = 2.5·m (so 2.25·m tooth height + 0.25·m
// for a base land).
TopoDS_Shape buildRackToothGap(double moduleM, double pressureAngleRad,
                               double faceWidth, double barTopZ,
                               double overhang)
{
    const double m       = moduleM;
    const double h       = 2.25 * m;        // total tooth height
    const double tanA    = std::tan(pressureAngleRad);
    const double gapTopHalf = 0.5 * (M_PI * m / 2.0 + 2.0 * m * tanA);
    const double gapBotHalf = std::max(0.05 * m,
                                       0.5 * (M_PI * m / 2.0 - 2.5 * m * tanA));

    // We build a closed wire in the XZ plane at Y = -overhang.
    const double zTop = barTopZ + overhang;
    const double zBot = barTopZ - h - overhang;

    const gp_Pnt p1(-gapTopHalf, -overhang, zTop);  // top-left
    const gp_Pnt p2( gapTopHalf, -overhang, zTop);  // top-right
    const gp_Pnt p3( gapBotHalf, -overhang, zBot);  // bot-right
    const gp_Pnt p4(-gapBotHalf, -overhang, zBot);  // bot-left

    BRepBuilderAPI_MakeWire wire;
    wire.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(p2, p3).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(p3, p4).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(p4, p1).Edge());

    if (!wire.IsDone())
        throw Standard_Failure("rack_involute: gap wire build failed");

    BRepBuilderAPI_MakeFace face(wire.Wire(), true);
    if (!face.IsDone())
        throw Standard_Failure("rack_involute: gap face build failed");

    BRepPrimAPI_MakePrism prism(face.Face(),
        gp_Vec(0.0, faceWidth + 2.0 * overhang, 0.0));
    prism.Build();
    if (!prism.IsDone())
        throw Standard_Failure("rack_involute: gap prism build failed");

    return prism.Shape();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.module_mm <= 0.3 || in.module_mm > 50.0) {
        r.add("DFM-GEAR-MOD", "error",
              "rack_involute: module_mm out of (0.3, 50] mm");
    }
    if (in.tooth_count < 2 || in.tooth_count > 500) {
        r.add("DFM-GEAR-CT", "error",
              "rack_involute: tooth_count " + std::to_string(in.tooth_count) +
              " out of [2, 500]");
    }
    if (in.face_width_mm <= 0.0) {
        r.add("DFM-GEAR-FW", "error",
              "rack_involute: face_width_mm must be > 0");
    }
    if (in.pressure_angle_deg < 14.5 || in.pressure_angle_deg > 30.0) {
        r.add("DFM-GEAR-PA", "error",
              "rack_involute: pressure_angle out of [14.5, 30] deg");
    }
    if (in.module_mm > 0.0 && in.tooth_count > 0) {
        const double pitch = M_PI * in.module_mm;
        const double needed = in.tooth_count * pitch;
        const double rackLen = in.rack_length_mm > 0.0
            ? in.rack_length_mm
            : needed + 2.0 * in.module_mm;
        if (rackLen + 1e-6 < needed) {
            r.add("DFM-GEAR-LEN", "error",
                  "rack_involute: rack_length " + std::to_string(rackLen) +
                  " shorter than required " + std::to_string(needed));
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
        std::string msg = "rack_involute DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double m     = in.module_mm;
    const double Z     = in.face_width_mm;
    const double pitch = M_PI * m;
    const int    z     = in.tooth_count;
    const double alpha = in.pressure_angle_deg * M_PI / 180.0;
    const double kOver = 0.05;

    // Determine bar top Z (assume workpiece bbox top is bar top).
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double barTopZ = zMax;

    // Place teeth centred on the bar's X-span midpoint.
    const double rackLen = in.rack_length_mm > 0.0
        ? in.rack_length_mm
        : z * pitch + 2.0 * m;
    const double xMid    = 0.5 * (xMin + xMax);
    const double xStart  = xMid - (z - 1) * pitch / 2.0;

    // Build seed gap at X = 0, then translate copies along X.
    const TopoDS_Shape seed = buildRackToothGap(m, alpha, Z, barTopZ, kOver);

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(static_cast<size_t>(z));
    for (int i = 0; i < z; ++i) {
        const double xPos = xStart + static_cast<double>(i) * pitch;
        gp_Trsf tr;
        tr.SetTranslation(gp_Vec(xPos, yMin - kOver, 0.0));
        BRepBuilderAPI_Transform xf(seed, tr, true);
        if (!xf.IsDone())
            throw SkillError("rack_involute: cutter translation failed");
        cutters.push_back(xf.Shape());
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    // Volume per gap (rough trapezoidal): avg width × height × face_width.
    const double tanA = std::tan(alpha);
    const double gapTopW = M_PI * m / 2.0 + 2.0 * m * tanA;
    const double gapBotW = std::max(0.1 * m, M_PI * m / 2.0 - 2.5 * m * tanA);
    const double h       = 2.25 * m;
    const double gapVol  = 0.5 * (gapTopW + gapBotW) * h * Z;
    const double volRemoved = z * gapVol;

    json params = {
        { "module_mm",          m },
        { "tooth_count",        z },
        { "face_width_mm",      Z },
        { "pressure_angle_deg", in.pressure_angle_deg },
        { "rack_length_mm",     rackLen },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_gear",               true },
        { "is_rack",               true },
        { "is_compound",           true },
        { "subfeature_count",      z },
        { "module_mm",             m },
        { "tooth_count",           z },
        { "pressure_angle_deg",    in.pressure_angle_deg },
        { "pitch_mm",              pitch },
        { "rack_length_mm",        rackLen },
        { "face_width_mm",         Z },
        { "derived_volume_removed", volRemoved },
        { "standard",              "ISO 53" },
        { "profile",               "trapezoidal_flank" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "rack_cutter";
    tooling.tool_dia_mm       = std::min(0.4 * m * M_PI, 10.0);
    tooling.tool_length_mm    = Z * 2.0 + 10.0;
    tooling.tool_material     = "hss";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 80.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(30.0, z * 1.2 + 15.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::rack_involute applied: m={} z={} faces {}→{}",
                  m, z, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Geometric: count planar flank faces along X axis with tilted normal
    // (|n.X| > 0.2, |n.Z| < 0.95) at periodic X positions.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    std::vector<double> flankX;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        try {
            gp_Dir n = wp.faceNormal(i);
            // Tooth flanks are tilted: significant X component, modest Z.
            if (std::abs(n.X()) < 0.15) continue;
            if (std::abs(n.Z()) > 0.85) continue;
            const gp_Pnt c = wp.faceCenter(i);
            flankX.push_back(c.X());
        } catch (...) {}
    }

    if (flankX.size() >= 4) {
        std::sort(flankX.begin(), flankX.end());
        // Cluster by 0.5 mm bins
        int clusterCount = 0;
        for (size_t k = 0; k < flankX.size(); ++k) {
            if (k == 0 || (flankX[k] - flankX[k - 1]) > 0.5)
                ++clusterCount;
        }
        // Each tooth has 2 flank faces → tooth_count ≈ clusters / 2
        const int zEst = std::max(2, clusterCount / 2);
        const double rackLen = xMax - xMin;
        // pitch = π·m, so m = (rackLen / zEst) / π   (approximate)
        const double moduleEst = (rackLen / static_cast<double>(zEst)) / M_PI;

        json recovered = {
            { "module_mm",          moduleEst },
            { "tooth_count",        zEst },
            { "face_width_mm",      yMax - yMin },
            { "pressure_angle_deg", 20.0 },
            { "rack_length_mm",     rackLen },
        };
        json matched = {
            { "source",                "geometric_primary" },
            { "flank_face_count",      static_cast<int>(flankX.size()) },
            { "x_cluster_count",       clusterCount },
            { "estimated_tooth_count", zEst },
        };
        out.push_back(RecognizedFeature{
            kSkillId, recovered, 0.65, matched
        });
    }

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            { { "source",       "feature_history_replay" },
              { "tooth_count",  f.pattern.value("tooth_count", 0) } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::rack_involute
