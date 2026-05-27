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
    wpNew->addFeature(sig);

    spdlog::debug("skill::mill_rect_pocket applied: {}x{}x{} cornerR={} faces {}→{}",
                  in.length_mm, in.width_mm, in.depth_mm, in.corner_r_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: 4 planar walls + (optional) 4 vertical-axis cylindrical corner
// fillets + 1 flat bottom planar face.  We detect rectangular pockets by:
//   1. Find planar faces with normal in Z-plane (vertical walls).
//   2. Cluster cylindrical faces with vertical axes (corner fillets).
//   3. Require ≥ 4 walls + 0 or 4 corner cylinders + 1 common bottom.

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

}  // namespace

// Geometric-attribute matcher: find the workpiece-face index whose plane
// matches `af`'s plane (parallel normal + coincident point) within tolerance.
// Robust against STEP round-trip, which preserves geometry but breaks the
// TShape handle identity that IsSame() relies on.
static int findPlanarFaceIndex(const Workpiece& wp, const TopoDS_Face& af)
{
    BRepAdaptor_Surface as(af);
    if (as.GetType() != GeomAbs_Plane) return -1;
    const gp_Pln plnA = as.Plane();
    const gp_Dir nA   = plnA.Axis().Direction();
    const gp_Pnt pA   = plnA.Location();

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        BRepAdaptor_Surface bs(wp.face(i));
        const gp_Pln plnB = bs.Plane();
        const gp_Dir nB   = plnB.Axis().Direction();
        // Normals parallel (allow either orientation — the underlying plane
        // is the same regardless of face orientation).
        if (std::abs(std::abs(nA.Dot(nB)) - 1.0) > 1e-4) continue;
        // The candidate point pA must lie on plane B.
        const gp_Pnt pB = plnB.Location();
        const double dx = pA.X() - pB.X();
        const double dy = pA.Y() - pB.Y();
        const double dz = pA.Z() - pB.Z();
        const double dist = std::abs(dx * nB.X() + dy * nB.Y() + dz * nB.Z());
        if (dist > 1e-3) continue;
        return i;
    }
    return -1;
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    // For each vertical-axis cylindrical face (potential corner fillet), find
    // its adjacent planar bottom face and group by that face id (the shared
    // bottom face identifies one pocket).
    struct CornerEntry {
        int    cylFaceIdx;
        double radius;
        gp_Pnt axisBase;       // axis location (any point on axis line)
        gp_Dir axisDir;
        double zTop;
        double zBot;
    };

    // Map: shared-bottom-face-index → list of corner entries
    std::map<int, std::vector<CornerEntry>> bottomGroups;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cf = wp.face(fIdx);
        BRepAdaptor_Surface surf(cf);
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir adir = cyl.Axis().Direction();
        // Vertical corner fillet: axis nearly parallel to global Z.
        if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-2) continue;

        // Find the adjacent planar face (the bottom) shared with this cyl
        // via one of its circular edges.
        int bottomFaceIdx = -1;
        double zHi = -1e30, zLo = 1e30;
        for (TopExp_Explorer exp(cf, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Pnt mid = crv.Value((crv.FirstParameter() + crv.LastParameter()) / 2.0);
            zHi = std::max(zHi, mid.Z());
            zLo = std::min(zLo, mid.Z());
            if (!edgeFaces.Contains(e)) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(cf)) continue;
                if (BRepAdaptor_Surface(af).GetType() != GeomAbs_Plane) continue;
                // Match `af` to a workpiece face by geometric attributes,
                // not by IsSame() (STEP round-trip preserves geometry but
                // creates fresh TShape handles → IsSame fails).
                const int idx = findPlanarFaceIndex(wp, af);
                if (idx >= 0) { bottomFaceIdx = idx; }
            }
        }
        if (bottomFaceIdx < 0) continue;
        bottomGroups[bottomFaceIdx].push_back({
            fIdx, cyl.Radius(), cyl.Axis().Location(), adir, zHi, zLo
        });
    }

    // For each group of 4 corner cylinders sharing a bottom face → emit one
    // recognized rect-pocket candidate.
    for (const auto& [bottomIdx, entries] : bottomGroups) {
        if (entries.size() != 4) continue;
        // Estimate center = mean of axis bases (XY).
        double cxSum = 0.0, cySum = 0.0;
        double rSum = 0.0;
        double zHiSum = 0.0, zLoSum = 0.0;
        for (const auto& e : entries) {
            cxSum += e.axisBase.X();
            cySum += e.axisBase.Y();
            rSum  += e.radius;
            zHiSum += e.zTop;
            zLoSum += e.zBot;
        }
        const double cx = cxSum / 4.0;
        const double cy = cySum / 4.0;
        const double r  = rSum  / 4.0;
        const double zHi = zHiSum / 4.0;
        const double zLo = zLoSum / 4.0;
        const double depth = std::abs(zHi - zLo);

        // length/width: max XY extent between corner axis bases × 2
        double maxDX = 0.0, maxDY = 0.0;
        for (const auto& e : entries) {
            maxDX = std::max(maxDX, std::abs(e.axisBase.X() - cx));
            maxDY = std::max(maxDY, std::abs(e.axisBase.Y() - cy));
        }
        const double length = 2.0 * (maxDX + r);
        const double width  = 2.0 * (maxDY + r);

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
            { "bottom_face_id", bottomIdx },
            { "corner_cylinder_ids", { entries[0].cylFaceIdx, entries[1].cylFaceIdx,
                                       entries[2].cylFaceIdx, entries[3].cylFaceIdx } },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.85, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::mill_rect_pocket
