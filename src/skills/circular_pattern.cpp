// @lat: [[engine/skills#circular_pattern]]

#include "circular_pattern.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <TopoDS.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::circular_pattern {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.count < 1) {
        r.add("DFM-INPUT", "error", "circular_pattern: count must be >= 1");
    }
    if (in.hole_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "circular_pattern: hole_dia_mm must be > 0");
    }
    if (in.hole_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "circular_pattern: hole_depth_mm must be > 0");
    }
    if (in.radial_offset_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "circular_pattern: radial_offset_mm must be > 0");
    }
    if (in.total_angle_deg <= 0.0 || in.total_angle_deg > 360.0) {
        r.add("DFM-INPUT", "error",
              "circular_pattern: total_angle_deg " +
              std::to_string(in.total_angle_deg) + " out of (0, 360]");
    }
    if (in.hole_dia_mm > 0.0 && in.hole_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "circular_pattern: hole_dia_mm " + std::to_string(in.hole_dia_mm) +
              " < 0.8 mm (ISO 235:2016)");
    }
    const double an2 = in.axis_dir_xyz[0] * in.axis_dir_xyz[0] +
                       in.axis_dir_xyz[1] * in.axis_dir_xyz[1] +
                       in.axis_dir_xyz[2] * in.axis_dir_xyz[2];
    if (an2 < 1e-18) {
        r.add("DFM-INPUT", "error",
              "circular_pattern: axis_dir_xyz is zero-length");
    }
    // DFM-PATT-2 — chord_pitch = 2*R*sin(step/2) must exceed hole_dia.
    if (in.count > 1 && in.radial_offset_mm > 0.0 && in.total_angle_deg > 0.0) {
        const double stepDeg = in.total_angle_deg / in.count;
        const double stepRad = stepDeg * M_PI / 180.0;
        const double chord   = 2.0 * in.radial_offset_mm * std::sin(stepRad / 2.0);
        if (chord <= in.hole_dia_mm) {
            r.add("DFM-PATT-2", "error",
                  "circular_pattern: chord_pitch " + std::to_string(chord) +
                  " <= hole_dia_mm " + std::to_string(in.hole_dia_mm) +
                  " (instances would overlap)");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "circular_pattern DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double an = std::sqrt(
        in.axis_dir_xyz[0] * in.axis_dir_xyz[0] +
        in.axis_dir_xyz[1] * in.axis_dir_xyz[1] +
        in.axis_dir_xyz[2] * in.axis_dir_xyz[2]);
    const gp_Dir axisDir(in.axis_dir_xyz[0] / an,
                         in.axis_dir_xyz[1] / an,
                         in.axis_dir_xyz[2] / an);
    const gp_Pnt axisOrg(in.axis_origin_xyz[0],
                         in.axis_origin_xyz[1],
                         in.axis_origin_xyz[2]);
    const gp_Ax1 rotAxis(axisOrg, axisDir);

    // Pick an "X̂_⊥" perpendicular to axisDir.
    gp_Dir refX(1.0, 0.0, 0.0);
    if (std::abs(gp_Vec(axisDir).Dot(gp_Vec(refX))) > 0.95)
        refX = gp_Dir(0.0, 1.0, 0.0);
    gp_Vec vx(refX);
    vx = vx - gp_Vec(axisDir) * vx.Dot(gp_Vec(axisDir));
    vx.Normalize();
    const gp_Dir radialDir(vx);

    // Base seed cylinder at theta=0, on the +radialDir at radial_offset_mm.
    constexpr double kOverhang = 0.1;
    const double toolLen = in.hole_depth_mm + 2.0 * kOverhang;
    const double r       = in.hole_dia_mm * 0.5;

    // Start the cylinder slightly +axisDir-side of the axis origin so the
    // cut starts cleanly.  Cut direction = -axisDir (into the body).
    const gp_Pnt seedStart(
        axisOrg.X() + radialDir.X() * in.radial_offset_mm + axisDir.X() * kOverhang,
        axisOrg.Y() + radialDir.Y() * in.radial_offset_mm + axisDir.Y() * kOverhang,
        axisOrg.Z() + radialDir.Z() * in.radial_offset_mm + axisDir.Z() * kOverhang);

    const gp_Dir cutDir(-axisDir.X(), -axisDir.Y(), -axisDir.Z());
    const gp_Ax2 seedAx(seedStart, cutDir);
    const TopoDS_Shape seedCyl = pr::cylinder(seedAx, r, toolLen);

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(static_cast<size_t>(in.count));
    const double stepDeg = in.total_angle_deg / in.count;
    for (int i = 0; i < in.count; ++i) {
        const double angleRad = i * stepDeg * M_PI / 180.0;
        gp_Trsf rot;
        rot.SetRotation(rotAxis, angleRad);
        BRepBuilderAPI_Transform xform(seedCyl, rot, true);
        if (!xform.IsDone())
            throw SkillError("circular_pattern: transform failed");
        cutters.push_back(xform.Shape());
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    const double perVol = M_PI * r * r * in.hole_depth_mm;
    const double totalVol = perVol * in.count;

    json params = {
        { "axis_origin_xyz",  in.axis_origin_xyz },
        { "axis_dir_xyz",     { axisDir.X(), axisDir.Y(), axisDir.Z() } },
        { "hole_dia_mm",      in.hole_dia_mm },
        { "hole_depth_mm",    in.hole_depth_mm },
        { "radial_offset_mm", in.radial_offset_mm },
        { "count",            in.count },
        { "total_angle_deg",  in.total_angle_deg },
    };
    json pattern = {
        { "kind",           kSkillId },
        { "is_pattern",     true },
        { "instance_count", in.count },
        { "derived_params", {
            { "hole_dia_mm",       in.hole_dia_mm },
            { "hole_depth_mm",     in.hole_depth_mm },
            { "radial_offset_mm",  in.radial_offset_mm },
            { "total_angle_deg",   in.total_angle_deg },
            { "step_deg",          stepDeg },
            { "per_inst_vol_mm3",  perVol },
            { "total_vol_mm3",     totalVol },
        } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill";
    tooling.tool_dia_mm       = in.hole_dia_mm;
    tooling.tool_length_mm    = in.hole_depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = totalVol;
    tooling.est_cycle_time_s  = std::max(1.0, in.hole_depth_mm / 50.0) * in.count;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::circular_pattern applied: count={} R={} dia={} vol={}",
                  in.count, in.radial_offset_mm, in.hole_dia_mm, totalVol);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Heuristic: ≥3 cylindrical faces sharing radius (±1%) whose centers lie on
// a common circle (variance of distance-from-mean-center < 5%) → candidate.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 0.95;
        r.matched_geometry = f.pattern;
        out.push_back(r);
    }
    if (!out.empty()) return out;

    struct C { double x, y, z, r; };
    std::vector<C> all;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder gc = s.Cylinder();
            // AXIS gate: the recovered params (axis_dir hard-coded +Z) and
            // apply() assume holes bored along +/-Z.  A SIDE bolt-circle (axis
            // along X/Y) would be recovered with the wrong axis and its
            // hole_centers (XY only) would mismatch the drills in subsumption, so
            // recover ONLY vertical-axis bolt circles (a 3-D variant is a follow-up).
            if (std::abs(gc.Axis().Direction().Z()) < 0.99) continue;
            all.push_back(C{ gc.Location().X(), gc.Location().Y(),
                             gc.Location().Z(), gc.Radius() });
        } catch (...) {}
    }
    if (all.size() < 3) return out;

    // Only co-radial cylinders can belong to one bolt circle.  Pick the LARGEST
    // same-radius group (within 1e-2) of >= 3 — this discards the stock's own
    // outer wall (a lone, differently-sized cylinder) that would otherwise
    // pollute the centre/radius statistics and break the deviation gate.
    std::vector<C> cyls;
    {
        std::vector<bool> used(all.size(), false);
        for (size_t g = 0; g < all.size(); ++g) {
            if (used[g]) continue;
            std::vector<C> grp;
            for (size_t k = 0; k < all.size(); ++k)
                if (!used[k] && std::abs(all[k].r - all[g].r) < 1e-2) {
                    grp.push_back(all[k]); used[k] = true;
                }
            if (grp.size() > cyls.size()) cyls = grp;
        }
    }
    if (cyls.size() < 3) return out;

    // Compute mean XY center.
    double mx = 0.0, my = 0.0;
    for (const auto& c : cyls) { mx += c.x; my += c.y; }
    mx /= cyls.size();
    my /= cyls.size();
    // Distances from mean.
    std::vector<double> ds;
    for (const auto& c : cyls)
        ds.push_back(std::sqrt((c.x - mx) * (c.x - mx) + (c.y - my) * (c.y - my)));
    const double meanD = std::accumulate(ds.begin(), ds.end(), 0.0) / ds.size();
    if (meanD < 1e-3) return out;
    double maxDev = 0.0;
    for (double d : ds) maxDev = std::max(maxDev, std::abs(d - meanD));
    if (maxDev / meanD > 0.05) return out;

    // Require a roughly EVEN angular distribution too — N holes equally spaced
    // on the circle — so an irregular cluster that merely sits on a common
    // radius (e.g. 3 unrelated holes) is not mis-read as a bolt circle.
    std::vector<double> angles;
    for (const auto& c : cyls)
        angles.push_back(std::atan2(c.y - my, c.x - mx));
    std::sort(angles.begin(), angles.end());
    std::vector<double> aGaps;
    for (size_t k = 1; k < angles.size(); ++k)
        aGaps.push_back(angles[k] - angles[k - 1]);
    aGaps.push_back(2.0 * M_PI - (angles.back() - angles.front()));  // wrap gap
    const double meanA = (2.0 * M_PI) / cyls.size();
    double maxADev = 0.0;
    for (double a : aGaps) maxADev = std::max(maxADev, std::abs(a - meanA));
    // Allow up to 25 % of the nominal step (a partial arc still reads as even).
    if (meanA > 1e-6 && maxADev / meanA > 0.25) return out;

    json centers = json::array();
    for (const auto& c : cyls) centers.push_back({ c.x, c.y });
    json recovered = {
        { "axis_origin_xyz",  { mx, my, 0.0 } },
        { "axis_dir_xyz",     { 0.0, 0.0, 1.0 } },
        { "hole_dia_mm",      2.0 * cyls.front().r },
        { "hole_depth_mm",    0.0 },
        { "radial_offset_mm", meanD },
        { "count",            static_cast<int>(cyls.size()) },
        { "total_angle_deg",  360.0 },
        { "hole_centers",     centers },     // for pattern subsumption
    };
    json matched = {
        { "cyl_count",  static_cast<int>(cyls.size()) },
        { "mean_radius", meanD },
        { "max_radial_deviation", maxDev },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.65, matched });
    return out;
}

}  // namespace koocadcam::skill::circular_pattern
