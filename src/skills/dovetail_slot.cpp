// @lat: [[engine/skills#dovetail_slot]]

#include "dovetail_slot.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::dovetail_slot {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.top_width_mm < 0.8) {
        r.add("DFM-002", "error",
              "dovetail_slot top_width " + std::to_string(in.top_width_mm) +
              " mm < min 0.8 mm");
    }
    if (in.bottom_width_mm < 0.8) {
        r.add("DFM-002", "error",
              "dovetail_slot bottom_width " + std::to_string(in.bottom_width_mm) +
              " mm < min 0.8 mm");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "depth_mm must be > 0");
    }
    if (in.wall_angle_deg < 15.0 || in.wall_angle_deg > 75.0) {
        r.add("DFM-011", "error",
              "dovetail_slot wall_angle " + std::to_string(in.wall_angle_deg) +
              "° outside [15°, 75°] — approaches knife-edge or vertical");
    }
    const double slotLen = std::hypot(in.end_x_mm - in.start_x_mm,
                                      in.end_y_mm - in.start_y_mm);
    if (slotLen <= 1e-6) {
        r.add("DFM-INPUT", "error", "start and end must differ (slot length > 0)");
    }
    if (std::abs(in.top_width_mm - in.bottom_width_mm) < 1e-3) {
        r.add("DFM-DOVETAIL", "warning",
              "top_width == bottom_width — this is a plain rectangular slot, "
              "not a dovetail (use mill_keyway instead)");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Build a trapezoidal cross-section face in the (yLoc, +Z) plane at the
// origin point.  The trapezoid sits with its top edge at z = topZ (the
// entry plane) and bottom edge at z = topZ - depth.
//
//          ┌──────┐ ← top edge, width = topW, at z = topZ
//          ╲      ╱
//           ╲    ╱   ← side walls, slope = wall_angle from vertical
//            └──┘    ← bottom edge, width = botW, at z = topZ - depth
//
// Built as a closed 4-edge wire centered on `origin` (which is the slot
// center along the y-direction at z=topZ).
TopoDS_Shape buildTrapezoidalFace(const gp_Pnt& center, const gp_Dir& yLoc,
                                  double topW, double botW, double depth)
{
    // Top corners (at +/- topW/2 along yLoc, at center.Z())
    const gp_Pnt p1(
        center.X() - topW / 2.0 * yLoc.X(),
        center.Y() - topW / 2.0 * yLoc.Y(),
        center.Z());
    const gp_Pnt p2(
        center.X() + topW / 2.0 * yLoc.X(),
        center.Y() + topW / 2.0 * yLoc.Y(),
        center.Z());
    // Bottom corners (at +/- botW/2 along yLoc, at center.Z() - depth)
    const gp_Pnt p3(
        center.X() + botW / 2.0 * yLoc.X(),
        center.Y() + botW / 2.0 * yLoc.Y(),
        center.Z() - depth);
    const gp_Pnt p4(
        center.X() - botW / 2.0 * yLoc.X(),
        center.Y() - botW / 2.0 * yLoc.Y(),
        center.Z() - depth);

    BRepBuilderAPI_MakeWire wireMaker;
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p2, p3).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p3, p4).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p4, p1).Edge());
    if (!wireMaker.IsDone())
        throw Standard_Failure("dovetail_slot: wire build failed");

    BRepBuilderAPI_MakeFace faceMaker(wireMaker.Wire(), true);
    if (!faceMaker.IsDone())
        throw Standard_Failure("dovetail_slot: face build failed");
    return faceMaker.Face();
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "dovetail_slot DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("dovetail_slot: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("dovetail_slot: only axis_dir = -Z is supported in slice 1");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    // Local frame: xLoc = along slot (start→end), yLoc = perpendicular in XY.
    const gp_Vec dir2D(in.end_x_mm - in.start_x_mm, in.end_y_mm - in.start_y_mm, 0.0);
    const double slotLen = dir2D.Magnitude();
    const gp_Dir xLoc(dir2D.X() / slotLen, dir2D.Y() / slotLen, 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    // Build trapezoidal face at the START end of the slot (with a slight
    // overhang above the entry plane so the Boolean cut is clean).
    const gp_Pnt startCenter(in.start_x_mm, in.start_y_mm, zMax + kOverhang);
    const TopoDS_Shape startFace = buildTrapezoidalFace(
        startCenter, yLoc, in.top_width_mm, in.bottom_width_mm,
        in.depth_mm + 2.0 * kOverhang);

    // Extrude along xLoc (the slot length) by slotLen.
    const gp_Vec extrudeVec(xLoc.X() * slotLen, xLoc.Y() * slotLen, 0.0);
    BRepPrimAPI_MakePrism prism(startFace, extrudeVec);
    prism.Build();
    if (!prism.IsDone())
        throw SkillError("dovetail_slot: prism build failed");
    const TopoDS_Shape cutter = prism.Shape();

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",      *entryId },
        { "start_x_mm",         in.start_x_mm },
        { "start_y_mm",         in.start_y_mm },
        { "end_x_mm",           in.end_x_mm },
        { "end_y_mm",           in.end_y_mm },
        { "axis_dir",           { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "top_width_mm",       in.top_width_mm },
        { "bottom_width_mm",    in.bottom_width_mm },
        { "depth_mm",           in.depth_mm },
        { "wall_angle_deg",     in.wall_angle_deg },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "angled_wall_count",          2 },
        { "bottom_planar_face",         true },
        { "end_cap_planar_count",       2 },
        { "top_width_mm",               in.top_width_mm },
        { "bottom_width_mm",            in.bottom_width_mm },
        { "depth_mm",                   in.depth_mm },
        { "length_mm",                  slotLen },
        { "wall_angle_deg",             in.wall_angle_deg },
        { "is_reverse_dovetail",        in.top_width_mm > in.bottom_width_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "dovetail_cutter";
    tooling.tool_dia_mm       = std::max(in.top_width_mm, in.bottom_width_mm);
    tooling.tool_length_mm    = in.depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.025;
    // Trapezoid cross-section area ≈ (top + bottom) / 2 × depth.
    tooling.stock_removed_mm3 = (in.top_width_mm + in.bottom_width_mm) / 2.0
                                * in.depth_mm * slotLen;
    tooling.est_cycle_time_s  = std::max(2.0, slotLen * 0.15 + in.depth_mm * 0.6);
    tooling.extra["wall_angle_deg"] = in.wall_angle_deg;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::dovetail_slot applied: top={} bot={} depth={} len={} faces {}→{}",
                  in.top_width_mm, in.bottom_width_mm, in.depth_mm, slotLen,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: 2 angled planar walls (normal has non-trivial Z component) +
// 1 horizontal flat bottom + 2 vertical end caps.

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

// Geometric planar-face matcher.  Robust to STEP round-trip — relies on
// plane normal + point coincidence rather than IsSame TShape identity.
int findPlanarFaceIndex(const Workpiece& wp, const TopoDS_Face& af)
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

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    auto faceIndex = [&](const TopoDS_Face& f) -> int {
        return findPlanarFaceIndex(wp, f);
    };

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // For each candidate horizontal "bottom" face (normal +Z, but NOT the
    // top of the workpiece), count adjacent angled walls.
    for (int bIdx = 0; bIdx < wp.faceCount(); ++bIdx) {
        if (!wp.isFacePlanar(bIdx)) continue;
        gp_Dir nb;
        try { nb = wp.faceNormal(bIdx); } catch (...) { continue; }
        if (std::abs(nb.Z() - 1.0) > 1e-2) continue;
        const gp_Pnt bc = wp.faceCenter(bIdx);
        if (std::abs(bc.Z() - zMax) < 1e-3) continue;

        // Walk adjacent planar walls; classify as angled (nz != 0) vs vertical.
        std::vector<int> angledWalls;
        std::vector<int> verticalWalls;
        for (TopExp_Explorer exp(wp.face(bIdx), TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(wp.face(bIdx))) continue;
                const int ai = faceIndex(af);
                if (ai < 0 || !wp.isFacePlanar(ai)) continue;
                gp_Dir an;
                try { an = wp.faceNormal(ai); } catch (...) { continue; }
                // Reject anything with a strongly vertical normal — that's
                // the top/bottom of the workpiece, not a wall.
                if (std::abs(an.Z()) > 0.95) continue;
                // Wall normal Z component classifies it.
                if (std::abs(an.Z()) > 1e-2) {
                    if (std::find(angledWalls.begin(), angledWalls.end(), ai) == angledWalls.end())
                        angledWalls.push_back(ai);
                } else {
                    if (std::find(verticalWalls.begin(), verticalWalls.end(), ai) == verticalWalls.end())
                        verticalWalls.push_back(ai);
                }
            }
        }
        if (angledWalls.size() != 2)   continue;
        if (verticalWalls.size() != 2) continue;

        // Recover geometry.
        const double depth = zMax - bc.Z();
        // start/end: midpoints of the two vertical end caps.
        const gp_Pnt sCap = wp.faceCenter(verticalWalls[0]);
        const gp_Pnt eCap = wp.faceCenter(verticalWalls[1]);

        // Bottom width: dimension of bottom face along its long axis ≈
        // sqrt(area² / length).  Use simpler: distance between the two
        // angled-wall centers projected perpendicular to vertical-wall-axis.
        const gp_Pnt cw1 = wp.faceCenter(angledWalls[0]);
        const gp_Pnt cw2 = wp.faceCenter(angledWalls[1]);
        const double midGap = std::hypot(cw1.X() - cw2.X(), cw1.Y() - cw2.Y());

        // Estimate top_width and bottom_width from wall normal Z component.
        // wall normal makes angle θ from vertical, where tan(θ) = (Δhoriz/Δvert)
        // across the wall face; we approximate top vs bottom by using wall area
        // and depth.
        gp_Dir aw0; double angleDeg = 30.0;
        try { aw0 = wp.faceNormal(angledWalls[0]);
              // angle from vertical (Z axis) → wall_angle_deg
              const double cosθ = std::abs(aw0.Z());
              angleDeg = std::acos(std::clamp(cosθ, 0.0, 1.0)) * 180.0 / M_PI;
              // wall_angle_deg = angle of wall from vertical, so we want the
              // complementary angle of (wall_normal vs +Z) — which is the
              // angle θ above directly.  (A vertical wall has normal in XY,
              // i.e. aw.Z = 0, θ = 90° from +Z; so wall_from_vertical =
              // 90° - 90° = 0°.  An angled wall with aw.Z = 0.5 → θ ≈ 60° →
              // wall_from_vertical = 30°.)
              angleDeg = 90.0 - angleDeg;
              angleDeg = std::abs(angleDeg);
        } catch (...) {}

        // Approximation: top_width and bottom_width such that
        //   (top_width + bottom_width) / 2 = midGap
        // and  (bottom_width - top_width) = 2 × depth × tan(angleDeg)
        // (for a positive dovetail — bottom wider than top).
        const double widthDiff = 2.0 * depth * std::tan(angleDeg * M_PI / 180.0);
        const double topW = midGap - widthDiff / 2.0;
        const double botW = midGap + widthDiff / 2.0;

        json recovered = {
            { "start_x_mm",       sCap.X() },
            { "start_y_mm",       sCap.Y() },
            { "end_x_mm",         eCap.X() },
            { "end_y_mm",         eCap.Y() },
            { "axis_dir",         { 0.0, 0.0, -1.0 } },
            { "top_width_mm",     std::max(0.0, topW) },
            { "bottom_width_mm",  std::max(0.0, botW) },
            { "depth_mm",         depth },
            { "wall_angle_deg",   angleDeg },
        };
        json matched = {
            { "bottom_face_id",   bIdx },
            { "angled_wall_ids",  { angledWalls[0], angledWalls[1] } },
            { "end_cap_ids",      { verticalWalls[0], verticalWalls[1] } },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.75, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::dovetail_slot
