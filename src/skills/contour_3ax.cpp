// @lat: [[engine/skills#contour_3ax]]

#include "contour_3ax.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace koocadcam::skill::contour_3ax {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.tool_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "contour_3ax: tool_dia_mm " +
              std::to_string(in.tool_dia_mm) +
              " < min 0.8 mm");
    }
    if (in.contour_polyline_3d.size() < 2) {
        r.add("DFM-3AX-PTS", "error",
              "contour_3ax: contour_polyline_3d needs >= 2 points "
              "(got " + std::to_string(in.contour_polyline_3d.size()) + ")");
    }

    // BBox check: polyline points must lie within the workpiece XY bbox.
    if (!in.contour_polyline_3d.empty()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double kBboxTol = 1e-3;
        for (size_t i = 0; i < in.contour_polyline_3d.size(); ++i) {
            const auto& p = in.contour_polyline_3d[i];
            if (p.x_mm < xMin - kBboxTol || p.x_mm > xMax + kBboxTol ||
                p.y_mm < yMin - kBboxTol || p.y_mm > yMax + kBboxTol) {
                r.add("DFM-3AX-BBOX", "error",
                      "contour_3ax: point " + std::to_string(i) +
                      " (" + std::to_string(p.x_mm) + ", " +
                      std::to_string(p.y_mm) +
                      ") outside workpiece XY bbox");
            }
            // Z must be at or below the top, above the bottom.
            if (p.z_mm > zMax + kBboxTol || p.z_mm < zMin - kBboxTol) {
                r.add("DFM-3AX-BBOX", "error",
                      "contour_3ax: point " + std::to_string(i) +
                      " z=" + std::to_string(p.z_mm) +
                      " outside workpiece Z bbox [" +
                      std::to_string(zMin) + ", " + std::to_string(zMax) + "]");
            }
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// For each consecutive pair (P_i, P_{i+1}) build a stadium-cutter:
//   - two cylinders of radius = tool_dia/2 at the segment endpoints
//   - one rectangular box between them
// All three reach down to z = min(z_i, z_{i+1}) and up to the workpiece
// top + small overhang.  Fuse the per-segment stadium pieces into a
// single tool, then cut once.

namespace {

// Build the stadium cutter for one polyline segment.  zTop is the
// workpiece top; zBottom is the segment's specified floor Z.
TopoDS_Shape buildSegmentCutter(double x1, double y1,
                                double x2, double y2,
                                double zBottom, double zTop,
                                double radius)
{
    const double kOverhang = 0.05;
    const double cutterH   = (zTop + kOverhang) - zBottom;

    const gp_Ax2 axStart(gp_Pnt(x1, y1, zBottom), gp_Dir(0.0, 0.0, 1.0));
    const gp_Ax2 axEnd  (gp_Pnt(x2, y2, zBottom), gp_Dir(0.0, 0.0, 1.0));
    const TopoDS_Shape cylStart = pr::cylinder(axStart, radius, cutterH);
    const TopoDS_Shape cylEnd   = pr::cylinder(axEnd,   radius, cutterH);

    const gp_Vec dir2D(x2 - x1, y2 - y1, 0.0);
    const double slotLen = dir2D.Magnitude();
    if (slotLen < 1e-9) {
        // Degenerate segment — just return a single cylinder so the cut
        // still succeeds.
        return cylStart;
    }
    const gp_Dir xLoc(dir2D.X() / slotLen, dir2D.Y() / slotLen, 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    const gp_Pnt boxOrigin(x1 - radius * yLoc.X(),
                           y1 - radius * yLoc.Y(),
                           zBottom);
    const gp_Ax2 boxAx(boxOrigin, gp_Dir(0.0, 0.0, 1.0), xLoc);
    const TopoDS_Shape boxConn = pr::box(boxAx, slotLen, 2.0 * radius, cutterH);

    return pr::fuseMany(cylStart, { cylEnd, boxConn });
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "contour_3ax DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("contour_3ax: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("contour_3ax: only axis_dir = -Z is supported in slice 1");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double radius = in.tool_dia_mm / 2.0;
    const double zTop   = zMax;

    // Build per-segment cutters and fuse them all.
    TopoDS_Shape combinedCutter;
    bool first = true;
    json segments = json::array();

    const auto& pts = in.contour_polyline_3d;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const auto& a = pts[i];
        const auto& b = pts[i + 1];
        const double zBottom = std::min(a.z_mm, b.z_mm);
        const double segLen  = std::hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm);

        // Skip zero-length segments at the same Z (no material to remove).
        if (segLen < 1e-9 && std::abs(a.z_mm - b.z_mm) < 1e-9) {
            segments.push_back({
                { "start", { a.x_mm, a.y_mm, a.z_mm } },
                { "end",   { b.x_mm, b.y_mm, b.z_mm } },
                { "z_floor", zBottom },
                { "length_mm", 0.0 },
                { "note", "degenerate (skipped)" },
            });
            continue;
        }

        const TopoDS_Shape segCutter = buildSegmentCutter(
            a.x_mm, a.y_mm, b.x_mm, b.y_mm,
            zBottom, zTop, radius);

        if (first) { combinedCutter = segCutter; first = false; }
        else        combinedCutter = pr::fuseMany(combinedCutter, { segCutter });

        segments.push_back({
            { "start",     { a.x_mm, a.y_mm, a.z_mm } },
            { "end",       { b.x_mm, b.y_mm, b.z_mm } },
            { "z_floor",   zBottom },
            { "length_mm", segLen },
        });
    }

    if (first) {
        // No usable segments (all degenerate).  Treat as error.
        throw SkillError("contour_3ax: contour_polyline_3d has no usable segments");
    }

    const TopoDS_Shape newShape = pr::cut(wp.shape(), combinedCutter);

    // Total swept length (for cycle-time estimate)
    double totalLen = 0.0;
    for (const auto& s : segments) {
        if (s.contains("length_mm"))
            totalLen += s.at("length_mm").get<double>();
    }

    // Build the signature
    json polylineJson = json::array();
    for (const auto& p : pts) {
        polylineJson.push_back({ p.x_mm, p.y_mm, p.z_mm });
    }

    json params = {
        { "entry_face_id",          *entryId },
        { "axis_dir",               { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "tool_dia_mm",            in.tool_dia_mm },
        { "stepover_mm",            in.stepover_mm },
        { "contour_polyline_3d",    polylineJson },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "segment_count",       segments.size() },
        { "segments",            segments },
        { "tool_dia_mm",         in.tool_dia_mm },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "topology",            "chain_of_stadium_segments_varying_z" },
    };

    ToolingMeta tooling;
    tooling.tool_type           = "ball_mill";   // 3-axis surface follow → ball-nose
    tooling.tool_dia_mm         = in.tool_dia_mm;
    tooling.tool_length_mm      = std::max(15.0,
        (zMax - zMin) * 1.5 + 5.0);
    tooling.tool_material       = "carbide";
    tooling.flute_count         = 2;
    tooling.cutting_speed_sfm   = 700.0;
    tooling.feed_per_tooth_mm   = 0.020;
    // Approximate stock removed = swept stadium area × average depth.
    // Average depth uses the mean Z drop along the polyline.
    double zSum = 0.0;
    for (const auto& p : pts) zSum += (zMax - p.z_mm);
    const double avgDrop = (pts.empty()) ? 0.0 : zSum / pts.size();
    tooling.stock_removed_mm3   = totalLen * in.tool_dia_mm * avgDrop;
    tooling.est_cycle_time_s    = std::max(1.0, totalLen * 0.1 + 5.0);
    tooling.extra["strategy"]           = "contour_3ax";
    tooling.extra["segment_count"]      = segments.size();
    tooling.extra["total_path_len_mm"]  = totalLen;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::contour_3ax applied: segments={} tool={} totalLen={}",
                  segments.size(), in.tool_dia_mm, totalLen);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: vertical-axis cylindrical end-caps at varying Z, connected by
// planar walls.  We:
//   1. Prefer feature-history when present (high confidence).
//   2. Otherwise count vertical cylinder faces in the workpiece; if there
//      are ≥ 4 (i.e. ≥ 2 segments × 2 end-caps each) we emit a low-
//      confidence marker recovering a coarse polyline from cylinder
//      centers.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // A. Feature history
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.85;
        rf.matched_geometry = {
            { "source", "feature_history" },
        };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // B. Geometric heuristic: gather vertical-axis cylinder faces of
    //    matching radius and project their centers + z-extent.  Order
    //    them by some sensible criterion (here: XY position along the
    //    line of best fit through the centers).  Slice-1 fidelity: we
    //    just report the count and a list of "potential vertices".
    struct VCyl {
        double r;
        gp_Pnt center;        // axis location (any point on axis)
        double zHi;
        double zLo;
    };
    std::vector<VCyl> vcs;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface surf(wp.face(fIdx));
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir adir = cyl.Axis().Direction();
        if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-2) continue;
        // Find the cylinder's z extent via its bounding circular edges.
        double zHi = -std::numeric_limits<double>::infinity();
        double zLo =  std::numeric_limits<double>::infinity();
        for (TopExp_Explorer exp(wp.face(fIdx), TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Pnt mid = crv.Value(
                (crv.FirstParameter() + crv.LastParameter()) / 2.0);
            zHi = std::max(zHi, mid.Z());
            zLo = std::min(zLo, mid.Z());
        }
        if (zHi == -std::numeric_limits<double>::infinity()) continue;
        vcs.push_back({ cyl.Radius(), cyl.Axis().Location(), zHi, zLo });
    }

    // Need ≥ 4 vertical cylinder faces (≥ 2 stadium segments).  Also they
    // must share radius (the contour uses one tool diameter).
    if (vcs.size() < 4) return out;
    const double r0 = vcs.front().r;
    for (const auto& v : vcs) {
        if (std::abs(v.r - r0) > 1e-3) return out;
    }

    // Look at the distribution of z-floors across cylinder centers.  If
    // they all share the same z, this is a flat-bottom feature (not 3ax);
    // skip.  Slice-1: if range > 0.5 mm we treat it as varying-Z.
    double zMin = vcs[0].zLo, zMax = vcs[0].zLo;
    for (const auto& v : vcs) {
        zMin = std::min(zMin, v.zLo);
        zMax = std::max(zMax, v.zLo);
    }
    if (std::abs(zMax - zMin) < 0.5) return out;

    // Recover a coarse polyline as the cylinder centers ordered by X.
    std::vector<VCyl> sorted = vcs;
    std::sort(sorted.begin(), sorted.end(),
              [](const VCyl& a, const VCyl& b) {
                  return a.center.X() < b.center.X();
              });
    json polylineJson = json::array();
    for (const auto& v : sorted) {
        polylineJson.push_back({ v.center.X(), v.center.Y(), v.zLo });
    }

    RecognizedFeature rf;
    rf.skill_id = kSkillId;
    rf.recovered_params = {
        { "axis_dir",            { 0.0, 0.0, -1.0 } },
        { "tool_dia_mm",         2.0 * r0 },
        { "stepover_mm",         0.0 },
        { "contour_polyline_3d", polylineJson },
    };
    rf.confidence       = 0.50;
    rf.matched_geometry = {
        { "source", "geometric_heuristic" },
        { "vertical_cyl_count",  vcs.size() },
        { "z_floor_range_mm",    zMax - zMin },
        { "note",
          "varying-Z cylinder centers ordered by X; full chain "
          "recovery requires BREP adjacency walk (slice-2)" },
    };
    out.push_back(rf);
    return out;
}

}  // namespace koocadcam::skill::contour_3ax
