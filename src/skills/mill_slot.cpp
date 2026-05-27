// @lat: [[engine/skills#mill_slot]]

#include "mill_slot.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Frames.hpp"
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
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::mill_slot {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.width_mm < 0.8) {
        r.add("DFM-002", "error",
              "mill_slot width_mm " + std::to_string(in.width_mm) +
              " < min 0.8 mm (end-mill min)");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "depth_mm must be > 0");
    }
    const double slotLen = std::hypot(in.end_x_mm - in.start_x_mm,
                                       in.end_y_mm - in.start_y_mm);
    if (slotLen <= 1e-6) {
        r.add("DFM-INPUT", "error", "start and end must differ (slot length > 0)");
    }
    if (in.width_mm > 0.0 && in.depth_mm / in.width_mm > 5.0) {
        r.add("DFM-AR", "warning",
              "depth/width ratio > 5 — tool stiffness limit");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mill_slot DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("mill_slot: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("mill_slot: only axis_dir = -Z is supported in slice 1");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;
    const double radius    = in.width_mm / 2.0;
    const double zBottom   = zMax - in.depth_mm;
    const double cutterH   = in.depth_mm + kOverhang;

    // Build stadium cutter = 2 cylinders + 1 box (fused).
    const gp_Ax2 axStart(gp_Pnt(in.start_x_mm, in.start_y_mm, zMax + kOverhang),
                         gp_Dir(0.0, 0.0, -1.0));
    const gp_Ax2 axEnd  (gp_Pnt(in.end_x_mm,   in.end_y_mm,   zMax + kOverhang),
                         gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape cylStart = pr::cylinder(axStart, radius, cutterH);
    const TopoDS_Shape cylEnd   = pr::cylinder(axEnd,   radius, cutterH);

    // Connecting box: oriented along start→end vector.
    const gp_Vec dir2D(in.end_x_mm - in.start_x_mm, in.end_y_mm - in.start_y_mm, 0.0);
    const double slotLen = dir2D.Magnitude();
    const gp_Dir xLoc(dir2D.X() / slotLen, dir2D.Y() / slotLen, 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);  // 90° CCW

    // gp_Ax2(P, V, Vx) — V = local Z, Vx = local X.  We pick V = +Z (up) and
    // Vx = xLoc so YDir = V × Vx = +Z × xLoc = (-xLoc.Y, xLoc.X, 0) = yLoc.
    // Then DX = slotLen along xLoc, DY = width along yLoc, DZ = cutterH along
    // +Z.  Origin = (start - radius*yLoc, zMax - depth)  so the box extends
    // from the slot bottom (z = zMax - depth) upward by cutterH = depth +
    // overhang, ending above the stock top.
    const gp_Pnt boxOrigin(
        in.start_x_mm - radius * yLoc.X(),
        in.start_y_mm - radius * yLoc.Y(),
        zMax - in.depth_mm);
    const gp_Ax2 boxAx(boxOrigin, gp_Dir(0.0, 0.0, 1.0), xLoc);
    const TopoDS_Shape boxConn = pr::box(boxAx, slotLen, in.width_mm,
                                          in.depth_mm + kOverhang);

    // Combine cutter parts.  Fuse rather than compound-then-cut because
    // we want a single connected cutter (avoids ambiguous Boolean result).
    const TopoDS_Shape cutter = pr::fuseMany(cylStart, { cylEnd, boxConn });

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",  *entryId },
        { "start_x_mm",     in.start_x_mm },
        { "start_y_mm",     in.start_y_mm },
        { "end_x_mm",       in.end_x_mm },
        { "end_y_mm",       in.end_y_mm },
        { "axis_dir",       { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "width_mm",       in.width_mm },
        { "depth_mm",       in.depth_mm },
    };
    json pattern = {
        { "kind",                     kSkillId },
        { "end_cylinder_count",       2 },
        { "long_wall_planar_count",   2 },
        { "bottom_planar_face",       true },
        { "width_mm",                 in.width_mm },
        { "depth_mm",                 in.depth_mm },
        { "length_mm",                slotLen },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = in.width_mm;
    tooling.tool_length_mm    = in.depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 600.0;
    tooling.feed_per_tooth_mm = 0.025;
    tooling.stock_removed_mm3 = (M_PI * radius * radius + slotLen * in.width_mm) * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.0, slotLen * 0.1 + in.depth_mm * 0.5);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::mill_slot applied: len={} width={} depth={} faces {}→{}",
                  slotLen, in.width_mm, in.depth_mm, wp.faceCount(), wpNew->faceCount());
    (void)zBottom;

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: 2 half-cylinder vertical-axis faces of matching radius, connected
// by 2 long planar walls, all sharing a common flat bottom face.

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
// matches `af`'s plane within tolerance.  Robust against STEP round-trip
// (which preserves geometry but breaks IsSame() TShape identity).
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
        if (std::abs(std::abs(nA.Dot(nB)) - 1.0) > 1e-4) continue;
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

    // Collect vertical-axis cylindrical faces grouped by shared bottom face.
    struct CylEntry {
        int     cylIdx;
        double  radius;
        gp_Pnt  axisBase;
        double  zTop;
        double  zBot;
    };
    std::map<int, std::vector<CylEntry>> bottomGroups;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cf = wp.face(fIdx);
        BRepAdaptor_Surface surf(cf);
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir adir = cyl.Axis().Direction();
        if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-2) continue;

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
                const int idx = findPlanarFaceIndex(wp, af);
                if (idx >= 0) { bottomFaceIdx = idx; }
            }
        }
        if (bottomFaceIdx < 0) continue;
        bottomGroups[bottomFaceIdx].push_back({
            fIdx, cyl.Radius(), cyl.Axis().Location(), zHi, zLo
        });
    }

    // For each group of exactly 2 vertical-cyl faces with matching radii →
    // mill_slot candidate.
    for (const auto& [bottomIdx, entries] : bottomGroups) {
        if (entries.size() != 2) continue;
        if (std::abs(entries[0].radius - entries[1].radius) > 1e-3) continue;
        const double r = entries[0].radius;
        const double depth = std::abs(entries[0].zTop - entries[0].zBot);

        json recovered = {
            { "start_x_mm",   entries[0].axisBase.X() },
            { "start_y_mm",   entries[0].axisBase.Y() },
            { "end_x_mm",     entries[1].axisBase.X() },
            { "end_y_mm",     entries[1].axisBase.Y() },
            { "axis_dir",     { 0.0, 0.0, -1.0 } },
            { "width_mm",     2.0 * r },
            { "depth_mm",     depth },
        };
        json matched = {
            { "bottom_face_id",       bottomIdx },
            { "end_cylinder_ids",     { entries[0].cylIdx, entries[1].cylIdx } },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.85, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::mill_slot
