// @lat: [[engine/skills#dovetail_mount_compound]]

#include "dovetail_mount_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <Standard_Failure.hxx>
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
#include <limits>

namespace koocadcam::skill::dovetail_mount_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.slot_width_mm < 2.0) {
        r.add("DFM-002", "error",
              "dovetail_mount_compound slot_width " +
              std::to_string(in.slot_width_mm) +
              " mm < 2 mm (dovetail cutter too small)");
    }
    if (in.slot_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "dovetail_mount_compound slot_depth_mm must be > 0");
    }
    if (in.slot_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "dovetail_mount_compound slot_length_mm must be > 0");
    }
    if (in.dovetail_angle_deg < 15.0 || in.dovetail_angle_deg > 45.0) {
        r.add("DFM-DOVETAIL", "error",
              "dovetail_mount_compound dovetail_angle " +
              std::to_string(in.dovetail_angle_deg) +
              "° outside [15°, 45°] — approaches knife-edge or vertical wall");
    }
    if (in.slot_width_mm > 0.0
        && in.slot_length_mm < 4.0 * in.slot_width_mm) {
        r.add("DFM-DOVETAIL", "error",
              "dovetail_mount_compound: slot_length (" +
              std::to_string(in.slot_length_mm) +
              " mm) < 4 × slot_width — insufficient guide length");
    }

    // rail_dir & slot_dir must be horizontal (±X / ±Y) for slice 1.
    const bool alongX = std::abs(std::abs(in.slot_dir.X()) - 1.0) < 1e-3
                        && std::abs(in.slot_dir.Y()) < 1e-3;
    const bool alongY = std::abs(std::abs(in.slot_dir.Y()) - 1.0) < 1e-3
                        && std::abs(in.slot_dir.X()) < 1e-3;
    if (!alongX && !alongY) {
        r.add("DFM-INPUT", "error",
              "dovetail_mount_compound: slot_dir must be ±X or ±Y in slice 1");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Build a trapezoidal cross-section face perpendicular to slot direction at
// `center` (top-center of trapezoid at z=center.Z()).  Top width = topW,
// bottom width = botW (botW > topW for proper dovetail), extending DOWN
// by depth.  Cross-section sits in (yLoc, +Z) plane.
TopoDS_Shape buildTrapezoidalFace(const gp_Pnt& topCenter, const gp_Dir& yLoc,
                                  double topW, double botW, double depth)
{
    const gp_Pnt p1(
        topCenter.X() - topW / 2.0 * yLoc.X(),
        topCenter.Y() - topW / 2.0 * yLoc.Y(),
        topCenter.Z());
    const gp_Pnt p2(
        topCenter.X() + topW / 2.0 * yLoc.X(),
        topCenter.Y() + topW / 2.0 * yLoc.Y(),
        topCenter.Z());
    const gp_Pnt p3(
        topCenter.X() + botW / 2.0 * yLoc.X(),
        topCenter.Y() + botW / 2.0 * yLoc.Y(),
        topCenter.Z() - depth);
    const gp_Pnt p4(
        topCenter.X() - botW / 2.0 * yLoc.X(),
        topCenter.Y() - botW / 2.0 * yLoc.Y(),
        topCenter.Z() - depth);

    BRepBuilderAPI_MakeWire wm;
    wm.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
    wm.Add(BRepBuilderAPI_MakeEdge(p2, p3).Edge());
    wm.Add(BRepBuilderAPI_MakeEdge(p3, p4).Edge());
    wm.Add(BRepBuilderAPI_MakeEdge(p4, p1).Edge());
    if (!wm.IsDone()) throw Standard_Failure("dovetail_mount_compound: wire failed");
    BRepBuilderAPI_MakeFace fm(wm.Wire(), true);
    if (!fm.IsDone()) throw Standard_Failure("dovetail_mount_compound: face failed");
    return fm.Face();
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "dovetail_mount_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("dovetail_mount_compound: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("dovetail_mount_compound: only axis_dir = -Z supported");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    // ── Derived dovetail bottom width: bottom is WIDER than top by
    // 2 × depth × tan(dovetail_angle).
    const double widthDiff = 2.0 * in.slot_depth_mm
                             * std::tan(in.dovetail_angle_deg * M_PI / 180.0);
    const double slotTopW  = in.slot_width_mm;
    const double slotBotW  = in.slot_width_mm + widthDiff;

    // Local axes in plane of entry face.
    const gp_Dir xLoc(in.slot_dir.X(), in.slot_dir.Y(), 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    // ── Sub-feature 1: dovetail slot ──────────────────────────────────
    //
    // Build trapezoid face at start (with overhang above entry plane), then
    // extrude along xLoc by slot_length.
    const gp_Pnt slotStart(in.start_x_mm, in.start_y_mm, zMax + kOverhang);
    const TopoDS_Shape trapFace = buildTrapezoidalFace(
        slotStart, yLoc, slotTopW, slotBotW, in.slot_depth_mm + 2.0 * kOverhang);

    const gp_Vec extrudeVec(
        xLoc.X() * in.slot_length_mm,
        xLoc.Y() * in.slot_length_mm,
        0.0);
    BRepPrimAPI_MakePrism dovePrism(trapFace, extrudeVec);
    dovePrism.Build();
    if (!dovePrism.IsDone()) throw SkillError("dovetail_mount_compound: prism failed");
    const TopoDS_Shape doveCutter = dovePrism.Shape();

    // ── Sub-feature 2-3: 2 set-screw threaded holes from one side wall ──
    //
    // Drill from the +yLoc side of the slot, axis = -yLoc, depth = slot_width.
    // Position the holes at 1/3 and 2/3 along slot_length, at z = zMax -
    // slot_depth/2.
    const double setScrewDia   = 4.2;   // M5 tap-drill
    const double setScrewDepth = in.slot_width_mm + 2.0;

    // Distance from slot center to the +yLoc wall surface = slotBotW/2 + 2 mm
    // (so the hole enters from outside the dovetail wall side surface).
    const double sideOffset = slotBotW / 2.0 + 5.0;

    auto setScrewToolAt = [&](double frac) -> TopoDS_Shape {
        const double along = frac * in.slot_length_mm;
        const gp_Pnt holeEntry(
            in.start_x_mm + along * xLoc.X() + sideOffset * yLoc.X(),
            in.start_y_mm + along * xLoc.Y() + sideOffset * yLoc.Y(),
            zMax - in.slot_depth_mm / 2.0);
        const gp_Dir holeAxis(-yLoc.X(), -yLoc.Y(), 0.0);
        const gp_Ax2 ax(holeEntry, holeAxis);
        return pr::cylinder(ax, setScrewDia / 2.0, setScrewDepth);
    };
    const TopoDS_Shape ss1 = setScrewToolAt(1.0 / 3.0);
    const TopoDS_Shape ss2 = setScrewToolAt(2.0 / 3.0);

    // ── Sub-feature 4: tee-slot stop at end of slot ──────────────────────
    //
    // A transverse cut at slot_length end (along yLoc), 3 mm wide, full
    // slot depth.  We place it just past the slot end to act as a stop.
    const double stopWidth = 3.0;
    const double stopLen   = slotBotW + 4.0;  // wider than slot bottom
    const gp_Pnt stopOrigin(
        in.start_x_mm + (in.slot_length_mm - stopWidth) * xLoc.X()
                      - stopLen / 2.0 * yLoc.X(),
        in.start_y_mm + (in.slot_length_mm - stopWidth) * xLoc.Y()
                      - stopLen / 2.0 * yLoc.Y(),
        zMax - in.slot_depth_mm);
    const gp_Ax2 stopAx(stopOrigin, gp_Dir(0.0, 0.0, 1.0), xLoc);
    const TopoDS_Shape stopBox = pr::box(stopAx, stopWidth, stopLen,
                                          in.slot_depth_mm + kOverhang);

    // Fuse the 4 tools; cut once.
    TopoDS_Shape fused;
    try {
        fused = pr::fuse(doveCutter, stopBox);
        fused = pr::fuse(fused, ss1);
        fused = pr::fuse(fused, ss2);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("dovetail_mount_compound: fuse failed: ") + e.what());
    }

    TopoDS_Shape newShape;
    try {
        newShape = pr::cut(wp.shape(), fused);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("dovetail_mount_compound: cut failed: ") + e.what());
    }

    json params = {
        { "entry_face_id",      *entryId },
        { "start_x_mm",         in.start_x_mm },
        { "start_y_mm",         in.start_y_mm },
        { "slot_dir",           { in.slot_dir.X(), in.slot_dir.Y(), in.slot_dir.Z() } },
        { "axis_dir",           { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "slot_length_mm",     in.slot_length_mm },
        { "slot_width_mm",      in.slot_width_mm },
        { "slot_depth_mm",      in.slot_depth_mm },
        { "dovetail_angle_deg", in.dovetail_angle_deg },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    4 },
        { "slot_length_mm",      in.slot_length_mm },
        { "slot_top_width_mm",   slotTopW },
        { "slot_bot_width_mm",   slotBotW },
        { "slot_depth_mm",       in.slot_depth_mm },
        { "dovetail_angle_deg",  in.dovetail_angle_deg },
        { "set_screw_dia_mm",    setScrewDia },
        { "set_screw_count",     2 },
        { "set_screw_thread",    "M5" },
        { "stop_width_mm",       stopWidth },
        { "stop_length_mm",      stopLen },
    };
    ToolingMeta tooling;
    tooling.tool_type   = "dovetail_cutter;drill;tap;endmill";
    tooling.tool_dia_mm = std::max(slotTopW, slotBotW);
    tooling.tool_material = "carbide";
    tooling.flute_count   = 4;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.025;
    const double doveV   = (slotTopW + slotBotW) / 2.0 * in.slot_depth_mm * in.slot_length_mm;
    const double ssV     = 2.0 * M_PI * (setScrewDia/2.0) * (setScrewDia/2.0) * setScrewDepth;
    const double stopV   = stopWidth * stopLen * in.slot_depth_mm;
    tooling.stock_removed_mm3 = doveV + ssV + stopV;
    tooling.est_cycle_time_s = std::max(10.0,
        in.slot_length_mm * 0.2 + in.slot_depth_mm * 0.5 + 8.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "endmill" },        { "tool_dia_mm", slotTopW },
              { "depth_mm", in.slot_depth_mm },  { "note", "pilot slot for dovetail entry" } },
            { { "tool_type", "dovetail_cutter" },{ "tool_dia_mm", slotBotW },
              { "depth_mm", in.slot_depth_mm },  { "note", "shape trapezoidal walls" } },
            { { "tool_type", "drill" },          { "tool_dia_mm", setScrewDia },
              { "depth_mm", setScrewDepth },     { "pass_count", 2 },
              { "note", "M5 set-screw tap-drills (side entry)" } },
            { { "tool_type", "tap" },            { "thread_size", "M5" },
              { "tap_depth_mm", setScrewDepth }, { "note", "side-tap from outside wall" } },
            { { "tool_type", "endmill" },        { "tool_dia_mm", stopWidth },
              { "depth_mm", in.slot_depth_mm },  { "note", "tee-slot stop" } },
        } },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::dovetail_mount_compound applied: len={} top_w={} bot_w={} depth={} angle={} faces {}→{}",
                  in.slot_length_mm, slotTopW, slotBotW,
                  in.slot_depth_mm, in.dovetail_angle_deg,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + geometric fallback ────────────────────
//
// Geometric signature of a dovetail mount compound:
//   - ONE dovetail slot: bottom planar face (+Z normal, below top), with TWO
//     angled planar walls (|n.Z| > 0.02) and TWO vertical end-cap walls.
//   - AT LEAST 2 transverse cylindrical "set-screw" holes whose axes are
//     perpendicular to the slot axis and lie at z ≈ slot mid-depth.
// Either condition alone would match a plain dovetail_slot; the combination
// is what makes it the compound mount.

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

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

std::vector<RecognizedFeature> geometric_fallback(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto edgeFaces = buildEdgeFaceMap(wp.shape());

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // 1) Locate a dovetail-slot bottom face (mirrors dovetail_slot.cpp).
    for (int bIdx = 0; bIdx < wp.faceCount(); ++bIdx) {
        if (!wp.isFacePlanar(bIdx)) continue;
        gp_Dir nb;
        try { nb = wp.faceNormal(bIdx); } catch (...) { continue; }
        if (std::abs(nb.Z() - 1.0) > 1e-2) continue;
        const gp_Pnt bc = wp.faceCenter(bIdx);
        if (std::abs(bc.Z() - zMax) < 1e-3) continue;

        std::vector<int> angledWalls;
        std::vector<int> verticalWalls;
        for (TopExp_Explorer exp(wp.face(bIdx), TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(wp.face(bIdx))) continue;
                const int ai = findPlanarFaceIndex(wp, af);
                if (ai < 0 || !wp.isFacePlanar(ai)) continue;
                gp_Dir an;
                try { an = wp.faceNormal(ai); } catch (...) { continue; }
                if (std::abs(an.Z()) > 0.95) continue;
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

        const double slotDepth = zMax - bc.Z();
        const gp_Pnt sCap = wp.faceCenter(verticalWalls[0]);
        const gp_Pnt eCap = wp.faceCenter(verticalWalls[1]);
        const gp_Vec slotVec(eCap.X() - sCap.X(), eCap.Y() - sCap.Y(), 0.0);
        const double slotLen = slotVec.Magnitude();
        if (slotLen < 4.0) continue;
        const gp_Dir slotDir(slotVec.X(), slotVec.Y(), 0.0);

        const gp_Pnt cw1 = wp.faceCenter(angledWalls[0]);
        const gp_Pnt cw2 = wp.faceCenter(angledWalls[1]);
        const double midGap = std::hypot(cw1.X() - cw2.X(), cw1.Y() - cw2.Y());

        gp_Dir aw0;
        double angleDeg = 30.0;
        try {
            aw0 = wp.faceNormal(angledWalls[0]);
            const double cosθ = std::abs(aw0.Z());
            angleDeg = std::acos(std::clamp(cosθ, 0.0, 1.0)) * 180.0 / M_PI;
            angleDeg = std::abs(90.0 - angleDeg);
        } catch (...) {}
        if (angleDeg < 15.0 || angleDeg > 45.0) continue;

        const double widthDiff = 2.0 * slotDepth
                                * std::tan(angleDeg * M_PI / 180.0);
        const double slotWidth = std::max(2.0, midGap - widthDiff * 0.5);

        // 2) Count transverse cylindrical holes whose axes are perpendicular
        //    to slot dir and lie at z ≈ mid-slot depth.
        const double slotMidZ = bc.Z() + slotDepth * 0.5;
        int sideHoles = 0;
        for (int cIdx = 0; cIdx < wp.faceCount(); ++cIdx) {
            if (!wp.isFaceCylinder(cIdx)) continue;
            BRepAdaptor_Surface cs(wp.face(cIdx));
            const gp_Cylinder cy = cs.Cylinder();
            const gp_Dir cdir = cy.Axis().Direction();
            if (std::abs(cdir.Z()) > 0.1) continue;  // axis must lie in XY plane
            const double dotSlot = std::abs(cdir.X()*slotDir.X() + cdir.Y()*slotDir.Y());
            if (dotSlot > 0.2) continue;             // must be ⊥ to slot
            const gp_Pnt cLoc = cy.Axis().Location();
            if (std::abs(cLoc.Z() - slotMidZ) > slotDepth) continue;
            if (cy.Radius() > 8.0 || cy.Radius() < 1.0) continue;
            sideHoles++;
        }
        if (sideHoles < 2) continue;

        json recovered = {
            { "start_x_mm",         sCap.X() },
            { "start_y_mm",         sCap.Y() },
            { "slot_dir",           { slotDir.X(), slotDir.Y(), 0.0 } },
            { "axis_dir",           { 0.0, 0.0, -1.0 } },
            { "slot_length_mm",     slotLen },
            { "slot_width_mm",      slotWidth },
            { "slot_depth_mm",      slotDepth },
            { "dovetail_angle_deg", angleDeg },
        };
        json matched = {
            { "source",            "geometric_fallback" },
            { "bottom_face_id",    bIdx },
            { "angled_wall_ids",   json(angledWalls) },
            { "vertical_wall_ids", json(verticalWalls) },
            { "side_hole_count",   sideHoles },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.50, matched });
    }
    return out;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "feature_history_replay" } };
        out.push_back(r);
    }
    if (out.empty()) {
        out = geometric_fallback(wp);
    }
    return out;
}

}  // namespace koocadcam::skill::dovetail_mount_compound
