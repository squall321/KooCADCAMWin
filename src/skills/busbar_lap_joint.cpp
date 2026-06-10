// @lat: [[engine/skills#busbar_lap_joint]]

#include "busbar_lap_joint.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::busbar_lap_joint {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bar_thickness_mm < 3.0) {
        r.add("DFM-IEEE605", "error",
              "bar_thickness " + std::to_string(in.bar_thickness_mm) +
              " mm < 3 mm — insufficient current-carrying cross-section");
    }
    if (in.bar_width_mm <= 0.0 || in.lap_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bar dimensions must be > 0");
    }

    static const double allowedDia[] = { 6.0, 8.0, 10.0, 12.0, 14.0, 16.0 };
    bool diaOk = false;
    for (double d : allowedDia) if (std::abs(d - in.bolt_dia_mm) < 0.1) { diaOk = true; break; }
    if (!diaOk) {
        r.add("DFM-NEMA-CC1", "error",
              "bolt_dia " + std::to_string(in.bolt_dia_mm) +
              " mm not standard (M6/M8/M10/M12/M14/M16)");
    }
    if (in.bolt_pitch_mm < 2.5 * in.bolt_dia_mm) {
        r.add("DFM-NEMA-CC1", "error",
              "bolt_pitch " + std::to_string(in.bolt_pitch_mm) +
              " mm < 2.5×bolt_dia edge-distance");
    }
    if (in.bolt_pitch_mm > in.lap_length_mm - in.bolt_dia_mm) {
        r.add("DFM-LAP-FIT", "error",
              "bolt_pitch + bolt_dia > lap_length — holes outside lap region");
    }
    if (in.notch_depth_mm >= in.bar_width_mm / 4.0) {
        r.add("DFM-LAP-NOTCH", "error",
              "notch_depth too large (must be < bar_width/4)");
    }
    if (in.dimple_dia_mm < 0.5) {
        r.add("DFM-002", "error",
              "dimple_dia " + std::to_string(in.dimple_dia_mm) + " mm < 0.5 mm");
    }
    if (in.dimple_depth_mm <= 0.0 ||
        in.dimple_depth_mm >= in.bar_thickness_mm / 2.0) {
        r.add("DFM-LAP-DIMPLE", "error",
              "dimple_depth must be in (0, bar_thickness/2)");
    }
    if (in.dimple_grid_nx < 1 || in.dimple_grid_ny < 1) {
        r.add("DFM-INPUT", "error", "dimple grid dimensions must be >= 1");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "busbar_lap_joint DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("busbar_lap_joint: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("busbar_lap_joint: only axis_dir = -Z supported");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zEntry = zMax;
    const double kOverhang = 0.05;

    // Bar local axis: bar extends along +X (from the end_x point toward
    // larger X).  Hole 1 is closest to the end, hole 2 is bolt_pitch
    // further along +X.
    const double h1x = in.end_x_mm + (in.lap_length_mm - in.bolt_pitch_mm) / 2.0;
    const double h2x = h1x + in.bolt_pitch_mm;
    const double hy  = in.end_y_mm;

    const double rBolt = in.bolt_dia_mm / 2.0;
    // 1) Hole 1.
    const gp_Pnt h1c(h1x, hy, zEntry + kOverhang);
    const gp_Ax2 h1Ax(h1c, gp_Dir(0, 0, -1));
    const TopoDS_Shape hole1 = pr::cylinder(
        h1Ax, rBolt, in.bar_thickness_mm + 2.0 * kOverhang);

    // 2) Hole 2.
    const gp_Pnt h2c(h2x, hy, zEntry + kOverhang);
    const gp_Ax2 h2Ax(h2c, gp_Dir(0, 0, -1));
    const TopoDS_Shape hole2 = pr::cylinder(
        h2Ax, rBolt, in.bar_thickness_mm + 2.0 * kOverhang);

    // 3) Alignment notch — a rectangular cutout on the +Y long edge,
    //    centered between the two holes along X, going inward by notch_depth.
    const double notchCenterX = (h1x + h2x) / 2.0;
    const double notchY = in.end_y_mm + in.bar_width_mm / 2.0;
    const gp_Pnt notchOrigin(
        notchCenterX - in.notch_width_mm / 2.0,
        notchY - in.notch_depth_mm,
        zEntry - in.bar_thickness_mm - kOverhang);
    const gp_Ax2 notchAx(notchOrigin, gp_Dir(0, 0, 1));
    const TopoDS_Shape notch = pr::box(
        notchAx, in.notch_width_mm, in.notch_depth_mm,
        in.bar_thickness_mm + 2.0 * kOverhang);

    // 4) Slip-resistance dimple grid — nx × ny shallow cylindrical pits,
    //    centered on (notchCenterX, hy), spanning ~half of the lap.
    const double gridLen = in.bolt_pitch_mm * 0.4;
    const double gridWid = in.bar_width_mm * 0.4;
    std::vector<TopoDS_Shape> dimples;
    const double rDim = in.dimple_dia_mm / 2.0;
    const int nx = in.dimple_grid_nx;
    const int ny = in.dimple_grid_ny;
    for (int ix = 0; ix < nx; ++ix) {
        for (int iy = 0; iy < ny; ++iy) {
            const double dx = (nx == 1) ? 0.0
                : (ix - (nx - 1) / 2.0) * (gridLen / (nx - 1));
            const double dy = (ny == 1) ? 0.0
                : (iy - (ny - 1) / 2.0) * (gridWid / (ny - 1));
            const gp_Pnt dc(notchCenterX + dx, hy + dy, zEntry + kOverhang);
            const gp_Ax2 dAx(dc, gp_Dir(0, 0, -1));
            dimples.push_back(pr::cylinder(dAx, rDim, in.dimple_depth_mm + kOverhang));
        }
    }

    // Fuse all cutters → single cut.
    TopoDS_Shape cutter = pr::fuse(hole1, hole2);
    cutter = pr::fuse(cutter, notch);
    for (const auto& d : dimples) cutter = pr::fuse(cutter, d);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",      *entryId },
        { "end_x_mm",           in.end_x_mm },
        { "end_y_mm",           in.end_y_mm },
        { "axis_dir",           { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "lap_length_mm",      in.lap_length_mm },
        { "bar_width_mm",       in.bar_width_mm },
        { "bar_thickness_mm",   in.bar_thickness_mm },
        { "bolt_dia_mm",        in.bolt_dia_mm },
        { "bolt_pitch_mm",      in.bolt_pitch_mm },
        { "notch_depth_mm",     in.notch_depth_mm },
        { "notch_width_mm",     in.notch_width_mm },
        { "dimple_grid_nx",     in.dimple_grid_nx },
        { "dimple_grid_ny",     in.dimple_grid_ny },
        { "dimple_dia_mm",      in.dimple_dia_mm },
        { "dimple_depth_mm",    in.dimple_depth_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "is_compound",      true },
        { "subfeature_count", 4 },          // 2 holes + notch + dimple grid
        { "bolt_hole_count",  2 },
        { "notch_count",      1 },
        { "dimple_count",     nx * ny },
        { "bolt_dia_mm",      in.bolt_dia_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;end_mill;center_punch";
    tooling.tool_dia_mm       = in.bolt_dia_mm;
    tooling.tool_length_mm    = in.bar_thickness_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 =
        2.0 * M_PI * rBolt * rBolt * in.bar_thickness_mm
      + in.notch_width_mm * in.notch_depth_mm * in.bar_thickness_mm
      + nx * ny * M_PI * rDim * rDim * in.dimple_depth_mm;
    tooling.est_cycle_time_s = std::max(5.0,
        2.0 * in.bar_thickness_mm / 30.0 + nx * ny * 0.4 + 3.0);
    tooling.extra = {
        { "standard",       "IEEE 605 / NEMA CC1" },
        { "operation_note", "Drill 2 bolt holes, mill notch, center-punch dimple grid" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::busbar_lap_joint applied: pitch={} bolt={} faces {}->{}",
                  in.bolt_pitch_mm, in.bolt_dia_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay (high confidence).
    for (const auto& sig : wp.features()) {
        if (sig.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = sig.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }

    // Geometric heuristic: ≥2 large vertical cylinders (boltholes) of same
    // dia, plus N small vertical cylinders (dimples).  Take the 2 largest
    // distinct boltholes and the dimple count as a confidence signal.
    struct Cyl { double radius; gp_Pnt loc; };
    std::vector<Cyl> verticalCyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder c = s.Cylinder();
        const gp_Dir d = c.Axis().Direction();
        if (std::abs(d.Z()) > 0.9)
            verticalCyls.push_back({ c.Radius(), c.Axis().Location() });
    }
    if (verticalCyls.size() < 2) return out;

    // Cluster radii.
    std::sort(verticalCyls.begin(), verticalCyls.end(),
              [](const Cyl& a, const Cyl& b){ return a.radius < b.radius; });

    // The bolthole class is the LARGEST radii cluster with >=2 members.
    std::vector<size_t> boltIdx;
    const double rMax = verticalCyls.back().radius;
    for (size_t i = 0; i < verticalCyls.size(); ++i) {
        if (std::abs(verticalCyls[i].radius - rMax) < 0.2)
            boltIdx.push_back(i);
    }
    if (boltIdx.size() < 2) return out;

    int dimpleCount = 0;
    const double rMin = verticalCyls.front().radius;
    for (const auto& vc : verticalCyls) {
        if (std::abs(vc.radius - rMin) < 0.2 && rMin < rMax * 0.5) ++dimpleCount;
    }

    const gp_Pnt p0 = verticalCyls[boltIdx[0]].loc;
    const gp_Pnt p1 = verticalCyls[boltIdx[1]].loc;
    const double pitch = std::hypot(p0.X() - p1.X(), p0.Y() - p1.Y());

    json recovered = {
        { "end_x_mm",        (p0.X() + p1.X()) / 2.0 - pitch / 2.0 },
        { "end_y_mm",        (p0.Y() + p1.Y()) / 2.0 },
        { "axis_dir",        { 0.0, 0.0, -1.0 } },
        { "bolt_dia_mm",     2.0 * rMax },
        { "bolt_pitch_mm",   pitch },
        { "dimple_count",    dimpleCount },
    };
    RecognizedFeature r;
    r.skill_id         = kSkillId;
    r.recovered_params = recovered;
    r.confidence       = (dimpleCount > 0) ? 0.75 : 0.55;
    r.matched_geometry = {
        { "bolt_hole_count", static_cast<int>(boltIdx.size()) },
        { "dimple_count",    dimpleCount },
    };
    out.push_back(r);
    return out;
}

}  // namespace koocadcam::skill::busbar_lap_joint
