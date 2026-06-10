// @lat: [[engine/skills#mill_rect_pocket]]

#include "mill_rect_pocket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::mill_rect_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.length_mm <= 0.0 || in.width_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "length_mm and width_mm must be > 0");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "depth_mm must be > 0");
    }
    if (in.corner_r_mm < 0.0) {
        r.add("DFM-INPUT", "error", "corner_r_mm must be >= 0");
    }
    if (in.corner_r_mm > 0.0 && in.corner_r_mm < 0.2) {
        r.add("DFM-004", "error",
              "corner_r " + std::to_string(in.corner_r_mm) +
              " mm < min R 0.2 mm");
    }
    const double minDim = std::min(in.length_mm, in.width_mm);
    if (in.corner_r_mm > 0.0 && in.corner_r_mm >= minDim / 2.0) {
        r.add("DFM-INPUT", "error",
              "corner_r must be < min(length, width) / 2 (geometry would collapse)");
    }
    if (in.depth_mm > 5.0) {
        r.add("DFM-006", "warning",
              "depth_mm " + std::to_string(in.depth_mm) +
              " > 5 — verify against workpiece thickness");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mill_rect_pocket DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("mill_rect_pocket: entry_face datum unresolved");

    // Bbox-driven entry Z (axis-aligned pockets — same convention as
    // mill_circular_pocket).  For Z-axis pockets only: bottomCenter Z is
    // (workpiece top - depth).  axis_dir != -Z is currently unsupported by
    // roundedRectPocketTool (which builds along +Z); we throw if so.
    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("mill_rect_pocket: only axis_dir = -Z is supported in slice 1");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double kOverhang = 0.05;
    const gp_Pnt bottomCenter(in.center_x_mm, in.center_y_mm,
                              zMax - in.depth_mm);

    const TopoDS_Shape cutter = pr::roundedRectPocketTool(
        bottomCenter, in.length_mm, in.width_mm,
        in.depth_mm + kOverhang, in.corner_r_mm);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // Build signature.
    json params = {
        { "entry_face_id", *entryId },
        { "center_x_mm",   in.center_x_mm },
        { "center_y_mm",   in.center_y_mm },
        { "axis_dir",      { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "length_mm",     in.length_mm },
        { "width_mm",      in.width_mm },
        { "depth_mm",      in.depth_mm },
        { "corner_r_mm",   in.corner_r_mm },
    };
    json pattern = {
        { "kind",                     kSkillId },
        { "planar_wall_count",        4 },
        { "corner_cylinder_count",    in.corner_r_mm > 0.0 ? 4 : 0 },
        { "bottom_planar_face",       true },
        { "length_mm",                in.length_mm },
        { "width_mm",                 in.width_mm },
        { "corner_r_mm",              in.corner_r_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = (in.corner_r_mm > 0.0) ? in.corner_r_mm * 2.0 : 6.0;
    tooling.tool_length_mm    = in.depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 600.0;
    tooling.feed_per_tooth_mm = 0.025;
    tooling.stock_removed_mm3 = in.length_mm * in.width_mm * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.0,
        (in.length_mm * in.width_mm * in.depth_mm) / 5000.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::mill_rect_pocket applied: {}x{}x{} cornerR={} faces {}→{}",
                  in.length_mm, in.width_mm, in.depth_mm, in.corner_r_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: 4 vertical-axis cylindrical corner fillets sharing the same Z
// extent + 1 flat bottom planar face at that Z extent's lower bound.
//
// Algorithm:
//   1. Collect all vertical-axis cylindrical faces with circular boundary
//      edges (potential corner fillets), recording each face's axis-XY and
//      its z-range.
//   2. Cluster sub-faces by axis-XY (same corner, post-STEP-roundtrip-split).
//   3. Group corner clusters by their common (zLo, zHi) — clusters sharing
//      the same z range belong to the same pocket.
//   4. Each pocket group with exactly 4 corner clusters → emit a candidate.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    struct SubFaceEntry {
        int    cylFaceIdx;
        double radius;
        double cx;
        double cy;
        double zTop;     // highest circular boundary edge z of THIS face
        double zBot;     // lowest circular boundary edge z of THIS face
    };
    std::vector<SubFaceEntry> sub;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cf = wp.face(fIdx);
        BRepAdaptor_Surface surf(cf);
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir adir = cyl.Axis().Direction();
        // Vertical corner fillet: axis nearly parallel to global Z.
        if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-2) continue;

        double zHi = -1e30, zLo = 1e30;
        bool hasCircularEdge = false;
        for (TopExp_Explorer exp(cf, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            hasCircularEdge = true;
            const gp_Pnt mid = crv.Value((crv.FirstParameter() + crv.LastParameter()) / 2.0);
            zHi = std::max(zHi, mid.Z());
            zLo = std::min(zLo, mid.Z());
        }
        if (!hasCircularEdge) continue;
        sub.push_back({
            fIdx, cyl.Radius(),
            cyl.Axis().Location().X(), cyl.Axis().Location().Y(),
            zHi, zLo
        });
    }

    // Cluster sub-faces by axis-XY (each cluster ≈ one corner position).
    // STEP roundtrip can split a single corner-fillet cylinder into 2-4
    // sub-faces sharing the same axis; this collapse re-unifies them and
    // accumulates the full Z extent across the sub-faces.
    struct Cluster {
        double cx = 0.0, cy = 0.0;
        double radius = 0.0;
        double zHi = -1e30, zLo = 1e30;
        std::vector<int> cylIds;
    };
    std::vector<Cluster> clusters;
    constexpr double kClusterTol = 0.05;   // mm
    for (const auto& e : sub) {
        Cluster* hit = nullptr;
        for (auto& c : clusters) {
            if (std::hypot(c.cx - e.cx, c.cy - e.cy) < kClusterTol &&
                std::abs(c.radius - e.radius) < kClusterTol) {
                hit = &c; break;
            }
        }
        if (!hit) {
            Cluster c;
            c.cx = e.cx; c.cy = e.cy;
            c.radius = e.radius;
            c.zHi = e.zTop; c.zLo = e.zBot;
            c.cylIds.push_back(e.cylFaceIdx);
            clusters.push_back(std::move(c));
        } else {
            hit->zHi = std::max(hit->zHi, e.zTop);
            hit->zLo = std::min(hit->zLo, e.zBot);
            hit->cylIds.push_back(e.cylFaceIdx);
        }
    }

    // Group corner clusters by (zLo, zHi) — corners that share the same Z
    // range belong to the same pocket.  Tolerance 0.05 mm.
    struct PocketGroup {
        double zLo, zHi;
        std::vector<int> clusterIdx;
    };
    std::vector<PocketGroup> pockets;
    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& c = clusters[i];
        PocketGroup* hit = nullptr;
        for (auto& pg : pockets) {
            if (std::abs(pg.zLo - c.zLo) < kClusterTol &&
                std::abs(pg.zHi - c.zHi) < kClusterTol) {
                hit = &pg; break;
            }
        }
        if (!hit) {
            pockets.push_back({ c.zLo, c.zHi, { static_cast<int>(i) } });
        } else {
            hit->clusterIdx.push_back(static_cast<int>(i));
        }
    }

    // Each pocket group with exactly 4 corner clusters → emit a candidate.
    for (const auto& pg : pockets) {
        if (pg.clusterIdx.size() != 4) continue;
        double cxSum = 0.0, cySum = 0.0, rSum = 0.0;
        double zHiSum = 0.0, zLoSum = 0.0;
        for (int idx : pg.clusterIdx) {
            const auto& c = clusters[idx];
            cxSum += c.cx;  cySum += c.cy;
            rSum  += c.radius;
            zHiSum += c.zHi;
            zLoSum += c.zLo;
        }
        const double cx = cxSum / 4.0;
        const double cy = cySum / 4.0;
        const double r  = rSum  / 4.0;
        const double zHi = zHiSum / 4.0;
        const double zLo = zLoSum / 4.0;
        const double depth = std::abs(zHi - zLo);

        double maxDX = 0.0, maxDY = 0.0;
        for (int idx : pg.clusterIdx) {
            const auto& c = clusters[idx];
            maxDX = std::max(maxDX, std::abs(c.cx - cx));
            maxDY = std::max(maxDY, std::abs(c.cy - cy));
        }
        const double length = 2.0 * (maxDX + r);
        const double width  = 2.0 * (maxDY + r);

        std::vector<int> allCylIds;
        for (int idx : pg.clusterIdx)
            allCylIds.insert(allCylIds.end(),
                             clusters[idx].cylIds.begin(),
                             clusters[idx].cylIds.end());

        // Find the bottom face geometrically (planar face at z ≈ zLo with
        // normal mostly ±Z).  Useful diagnostic, not a hard requirement.
        int bottomFaceIdx = -1;
        for (int i = 0; i < wp.faceCount(); ++i) {
            if (!wp.isFacePlanar(i)) continue;
            BRepAdaptor_Surface bs(wp.face(i));
            const gp_Pln pln = bs.Plane();
            if (std::abs(pln.Axis().Direction().Z()) < 0.95) continue;
            if (std::abs(pln.Location().Z() - zLo) > kClusterTol) continue;
            bottomFaceIdx = i; break;
        }

        json recovered = {
            { "center_x_mm",  cx },
            { "center_y_mm",  cy },
            { "axis_dir",     { 0.0, 0.0, -1.0 } },
            { "length_mm",    length },
            { "width_mm",     width },
            { "depth_mm",     depth },
            { "corner_r_mm",  r },
        };
        json matched = {
            { "bottom_face_id",       bottomFaceIdx },
            { "corner_cylinder_ids",  json(allCylIds) },
            { "corner_cluster_count", pg.clusterIdx.size() },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.85, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::mill_rect_pocket
