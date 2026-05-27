// @lat: [[engine/skills#mill_keyway]]

#include "mill_keyway.hpp"

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
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Torus.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace koocadcam::skill::mill_keyway {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.width_mm < 0.8) {
        r.add("DFM-002", "error",
              "mill_keyway width_mm " + std::to_string(in.width_mm) +
              " < min 0.8 mm");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "depth_mm must be > 0");
    }
    const double keyLen = std::hypot(in.end_x_mm - in.start_x_mm,
                                     in.end_y_mm - in.start_y_mm);
    if (keyLen <= 1e-6) {
        r.add("DFM-INPUT", "error", "start and end must differ (keyway length > 0)");
    }
    if (in.width_mm > 0.0 && in.depth_mm / in.width_mm > 5.0) {
        r.add("DFM-AR", "warning",
              "depth/width ratio > 5 — sharp-corner slot is difficult to broach");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mill_keyway DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("mill_keyway: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("mill_keyway: only axis_dir = -Z is supported in slice 1");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    // Build a single oriented box from start to end.
    const gp_Vec dir2D(in.end_x_mm - in.start_x_mm, in.end_y_mm - in.start_y_mm, 0.0);
    const double slotLen = dir2D.Magnitude();
    const gp_Dir xLoc(dir2D.X() / slotLen, dir2D.Y() / slotLen, 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);  // 90° CCW

    // Origin: at start, offset by -width/2 along yLoc, at the slot bottom Z.
    // gp_Ax2(P, V=+Z, Vx=xLoc) → DX along xLoc (slotLen),
    //                            DY along yLoc (width_mm),
    //                            DZ along +Z   (depth + overhang).
    const gp_Pnt boxOrigin(
        in.start_x_mm - in.width_mm / 2.0 * yLoc.X(),
        in.start_y_mm - in.width_mm / 2.0 * yLoc.Y(),
        zMax - in.depth_mm);
    const gp_Ax2 boxAx(boxOrigin, gp_Dir(0.0, 0.0, 1.0), xLoc);
    const TopoDS_Shape cutter = pr::box(boxAx, slotLen, in.width_mm,
                                         in.depth_mm + kOverhang);

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
        { "planar_wall_count",        4 },
        { "corner_cylinder_count",    0 },   // key differentiator
        { "bottom_planar_face",       true },
        { "width_mm",                 in.width_mm },
        { "depth_mm",                 in.depth_mm },
        { "length_mm",                slotLen },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "broach";  // typical for sharp-corner keyways
    tooling.tool_dia_mm       = in.width_mm;
    tooling.tool_length_mm    = in.depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "hss";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 30.0;       // broaching is slow
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = slotLen * in.width_mm * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(2.0, slotLen * 0.2 + in.depth_mm * 1.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::mill_keyway applied: len={} width={} depth={} faces {}→{}",
                  slotLen, in.width_mm, in.depth_mm, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: 4 planar walls + 1 flat bottom face, NO cylindrical corner
// fillets.  Differentiates from mill_rect_pocket (has 4 corner cylinders)
// and mill_slot (has 2 end half-cylinders).

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

double angleBetweenDir(const gp_Dir& a, const gp_Dir& b)
{
    const double dot = std::clamp(
        a.X()*b.X() + a.Y()*b.Y() + a.Z()*b.Z(), -1.0, 1.0);
    return std::acos(dot) * 180.0 / M_PI;
}

// Geometric face matcher: locate the workpiece face whose surface coincides
// with `af`'s (plane / cylinder / torus).  STEP round-trip safe — does not
// rely on IsSame() TShape identity.
int findFaceIndexGeom(const Workpiece& wp, const TopoDS_Face& af)
{
    BRepAdaptor_Surface as(af);
    const GeomAbs_SurfaceType ta = as.GetType();

    if (ta == GeomAbs_Plane) {
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
    if (ta == GeomAbs_Cylinder) {
        const gp_Cylinder cA = as.Cylinder();
        const gp_Ax1 axA     = cA.Axis();
        const gp_Pnt pA      = axA.Location();
        const gp_Dir dA      = axA.Direction();
        const double rA      = cA.Radius();
        for (int i = 0; i < wp.faceCount(); ++i) {
            if (!wp.isFaceCylinder(i)) continue;
            BRepAdaptor_Surface bs(wp.face(i));
            const gp_Cylinder cB = bs.Cylinder();
            const gp_Ax1 axB     = cB.Axis();
            if (std::abs(cB.Radius() - rA) > 1e-3) continue;
            if (std::abs(std::abs(dA.Dot(axB.Direction())) - 1.0) > 1e-4) continue;
            // Distance from pA to line axB must be ~ 0 (concentric axes).
            const gp_Pnt pB = axB.Location();
            const gp_Vec v(pB, pA);
            const gp_Vec axDir(axB.Direction());
            const gp_Vec perp = v - axDir * v.Dot(axDir);
            if (perp.Magnitude() > 1e-3) continue;
            return i;
        }
        return -1;
    }
    if (ta == GeomAbs_Torus) {
        const gp_Torus tA = as.Torus();
        const gp_Ax1 axA  = tA.Axis();
        const gp_Pnt pA   = axA.Location();
        const gp_Dir dA   = axA.Direction();
        const double majA = tA.MajorRadius();
        const double minA = tA.MinorRadius();
        for (int i = 0; i < wp.faceCount(); ++i) {
            BRepAdaptor_Surface bs(wp.face(i));
            if (bs.GetType() != GeomAbs_Torus) continue;
            const gp_Torus tB = bs.Torus();
            if (std::abs(tB.MajorRadius() - majA) > 1e-3) continue;
            if (std::abs(tB.MinorRadius() - minA) > 1e-3) continue;
            const gp_Ax1 axB = tB.Axis();
            if (std::abs(std::abs(dA.Dot(axB.Direction())) - 1.0) > 1e-4) continue;
            const gp_Pnt pB = axB.Location();
            if (pA.Distance(pB) > 1e-3) continue;
            return i;
        }
        return -1;
    }
    // Other surface types (sphere/cone/bspline) — not currently needed by
    // any skill's recognizer.  Fall back to IsSame as a best-effort.
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.face(i).IsSame(af)) return i;
    }
    return -1;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    // Reject if any vertical-axis cylindrical face exists adjacent to a
    // candidate bottom face — that means there is a corner fillet and the
    // feature is actually mill_slot / mill_rect_pocket.

    auto faceIndex = [&](const TopoDS_Face& f) -> int {
        return findFaceIndexGeom(wp, f);
    };

    // For each candidate "bottom" (horizontal-normal planar face that is NOT
    // the workpiece top face), enumerate its bordering walls.
    for (int bIdx = 0; bIdx < wp.faceCount(); ++bIdx) {
        if (!wp.isFacePlanar(bIdx)) continue;
        gp_Dir nb;
        try { nb = wp.faceNormal(bIdx); } catch (...) { continue; }
        // The pocket bottom's outward normal points UP (+Z) — into the pocket
        // air, away from the workpiece material below.  Skip ceilings/sides.
        if (std::abs(nb.Z() - 1.0) > 1e-2) continue;

        // Exclude the workpiece top face (its center sits at zMax).
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const gp_Pnt bc = wp.faceCenter(bIdx);
        if (std::abs(bc.Z() - zMax) < 1e-3) continue;

        // Walk faces sharing an edge with this candidate bottom.  Collect
        // unique adjacent planar wall faces and reject if any cylindrical.
        std::vector<int>    walls;
        bool                hasCylinderNeighbor = false;
        for (TopExp_Explorer exp(wp.face(bIdx), TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(wp.face(bIdx))) continue;
                const int ai = faceIndex(af);
                if (ai < 0) continue;
                if (wp.isFaceCylinder(ai)) { hasCylinderNeighbor = true; break; }
                if (!wp.isFacePlanar(ai))  continue;
                // Wall normals must be horizontal (perp to entry).
                try {
                    const gp_Dir an = wp.faceNormal(ai);
                    if (std::abs(an.Z()) > 1e-2) continue;  // not a vertical wall
                } catch (...) { continue; }
                if (std::find(walls.begin(), walls.end(), ai) == walls.end())
                    walls.push_back(ai);
            }
            if (hasCylinderNeighbor) break;
        }
        if (hasCylinderNeighbor) continue;
        if (walls.size() != 4)   continue;

        // Group walls into 2 pairs of parallel walls — sharp keyway.
        std::vector<std::pair<int,int>> pairs;
        std::vector<bool> used(4, false);
        for (int i = 0; i < 4 && pairs.size() < 2; ++i) {
            if (used[i]) continue;
            for (int j = i + 1; j < 4; ++j) {
                if (used[j]) continue;
                gp_Dir ni, nj;
                try { ni = wp.faceNormal(walls[i]); nj = wp.faceNormal(walls[j]); }
                catch (...) { continue; }
                const double ang = angleBetweenDir(ni, nj);
                if (ang > 175.0 || ang < 5.0) {
                    pairs.emplace_back(walls[i], walls[j]);
                    used[i] = used[j] = true;
                    break;
                }
            }
        }
        if (pairs.size() != 2) continue;

        // Pair[0] long walls (longer), pair[1] short walls (shorter).
        auto pairLen = [&](const std::pair<int,int>& p) {
            return wp.faceArea(p.first) + wp.faceArea(p.second);
        };
        if (pairLen(pairs[0]) < pairLen(pairs[1])) std::swap(pairs[0], pairs[1]);
        const auto& longPair  = pairs[0];
        const auto& shortPair = pairs[1];

        // Recover geometry.
        const gp_Pnt cL1 = wp.faceCenter(longPair.first);
        const gp_Pnt cL2 = wp.faceCenter(longPair.second);
        const gp_Pnt cS1 = wp.faceCenter(shortPair.first);
        const gp_Pnt cS2 = wp.faceCenter(shortPair.second);

        const double width = std::hypot(cL1.X() - cL2.X(), cL1.Y() - cL2.Y());
        // depth: from bottom face Z to wall top Z (approx from wall face area).
        // Approximate as |zMax - bc.Z()| only when bottom is at the right level.
        const double depth = zMax - bc.Z();

        json recovered = {
            { "start_x_mm",   cS1.X() },
            { "start_y_mm",   cS1.Y() },
            { "end_x_mm",     cS2.X() },
            { "end_y_mm",     cS2.Y() },
            { "axis_dir",     { 0.0, 0.0, -1.0 } },
            { "width_mm",     width },
            { "depth_mm",     depth },
        };
        json matched = {
            { "bottom_face_id",   bIdx },
            { "wall_face_ids",    { walls[0], walls[1], walls[2], walls[3] } },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.80, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::mill_keyway
