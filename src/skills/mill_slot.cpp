// @lat: [[engine/skills#mill_slot]]

#include "mill_slot.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Frames.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Standard_Failure.hxx>
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

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;
    const double radius    = in.width_mm / 2.0;
    const double cutterH   = in.depth_mm + kOverhang;

    // cutDir = the slot's machining axis (INTO the face).  Legacy callers pass
    // axis_dir = -Z; a SIDE port (USB-C / SIM tray) passes e.g. -Y.
    gp_Vec cutVec(in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z());
    if (cutVec.Magnitude() < 1e-9) throw SkillError("mill_slot: axis_dir is zero");
    cutVec.Normalize();
    const gp_Dir cutDir(cutVec);

    // Entry-plane start/end points.  BACKWARD-COMPAT: when the caller leaves
    // start_z/end_z at 0 AND axis = -Z (the original top-face contract), the
    // entry plane is the stock top (zMax) and start/end are the XY the caller
    // gave — identical to the previous behaviour.  Otherwise the caller's 3-D
    // start/end define the entry points directly.
    const bool legacyTop = std::abs(in.start_z_mm) < 1e-9 &&
                           std::abs(in.end_z_mm)   < 1e-9 &&
                           std::abs(in.axis_dir.X()) < 1e-6 &&
                           std::abs(in.axis_dir.Y()) < 1e-6 &&
                           in.axis_dir.Z() < 0.0;
    const double sz = legacyTop ? zMax : in.start_z_mm;
    const double ez = legacyTop ? zMax : in.end_z_mm;
    const gp_Pnt startPt(in.start_x_mm, in.start_y_mm, sz);
    const gp_Pnt endPt  (in.end_x_mm,   in.end_y_mm,   ez);

    // The cutter starts slightly OUTSIDE the entry plane (along -cutDir) and
    // extends cutterH inward (along +cutDir).
    const gp_Pnt cylStartBase(startPt.X() - cutDir.X() * kOverhang,
                              startPt.Y() - cutDir.Y() * kOverhang,
                              startPt.Z() - cutDir.Z() * kOverhang);
    const gp_Pnt cylEndBase  (endPt.X() - cutDir.X() * kOverhang,
                              endPt.Y() - cutDir.Y() * kOverhang,
                              endPt.Z() - cutDir.Z() * kOverhang);
    const gp_Ax2 axStart(cylStartBase, cutDir);
    const gp_Ax2 axEnd  (cylEndBase,   cutDir);
    const TopoDS_Shape cylStart = pr::cylinder(axStart, radius, cutterH);
    const TopoDS_Shape cylEnd   = pr::cylinder(axEnd,   radius, cutterH);

    // Connecting box, oriented along the start->end vector in the entry plane.
    // xLoc = slot length direction; the box's local Z (main dir) = cutDir so DZ
    // = depth extrudes inward; YDir = cutDir × xLoc = the width direction.
    gp_Vec lenVec(startPt, endPt);
    const double slotLen = lenVec.Magnitude();
    if (slotLen < 1e-9) throw SkillError("mill_slot: start == end (zero length)");
    lenVec.Normalize();
    const gp_Dir xLoc(lenVec);
    // width direction = cutDir × xLoc (perpendicular to both, in the entry plane).
    gp_Vec widthVec = gp_Vec(cutDir).Crossed(gp_Vec(xLoc));
    if (widthVec.Magnitude() < 1e-9) throw SkillError("mill_slot: axis parallel to slot");
    widthVec.Normalize();
    const gp_Dir yLoc(widthVec);

    // Box origin: start, shifted -radius along width, and -overhang along -cutDir
    // (so it starts outside the face like the cylinders).
    const gp_Pnt boxOrigin(
        startPt.X() - radius * yLoc.X() - cutDir.X() * kOverhang,
        startPt.Y() - radius * yLoc.Y() - cutDir.Y() * kOverhang,
        startPt.Z() - radius * yLoc.Z() - cutDir.Z() * kOverhang);
    // gp_Ax2(P, V=cutDir, Vx=xLoc): box DX runs along Vx (xLoc = slot length),
    // DY along YDir = cutDir × xLoc (= yLoc = width), DZ along V (cutDir = depth).
    const gp_Ax2 boxAx(boxOrigin, cutDir, xLoc);
    const TopoDS_Shape boxConn = pr::box(boxAx, slotLen, in.width_mm, cutterH);

    // Combine cutter parts.  Fuse rather than compound-then-cut because
    // we want a single connected cutter (avoids ambiguous Boolean result).
    const TopoDS_Shape cutter = pr::fuseMany(cylStart, { cylEnd, boxConn });

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // Emit the 3-D entry points + axis so a recovered side slot regenerates and
    // the CAM generator places the traverse on the real surface.
    json params = {
        { "entry_face_id",  *entryId },
        { "start_x_mm",     in.start_x_mm },
        { "start_y_mm",     in.start_y_mm },
        { "start_z_mm",     sz },
        { "end_x_mm",       in.end_x_mm },
        { "end_y_mm",       in.end_y_mm },
        { "end_z_mm",       ez },
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
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::mill_slot applied: len={} width={} depth={} faces {}→{}",
                  slotLen, in.width_mm, in.depth_mm, wp.faceCount(), wpNew->faceCount());

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

    // Collect cylindrical end-cap faces grouped by shared bottom face.  The
    // slot axis is arbitrary (a top slot bores along -Z; a SIDE port along
    // -X/-Y), so we track each end's geometry in 3-D and along the cylinder's
    // OWN axis instead of assuming Z.
    struct CylEntry {
        int     cylIdx;
        double  radius;
        gp_Pnt  axisBase;     // entry-plane centre of this end (3-D)
        gp_Dir  axisDir;      // this cylinder's axis
        double  aTop;         // max axial coordinate (entry side)
        double  aBot;         // min axial coordinate (bottom side)
    };
    std::map<int, std::vector<CylEntry>> bottomGroups;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cf = wp.face(fIdx);
        BRepAdaptor_Surface surf(cf);
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir adir = cyl.Axis().Direction();

        // CONCAVITY gate (B1.5 follow-up): a slot's end half-cylinder is
        // CONCAVE (outward normal points back toward the end axis); a
        // rounded TAB or outer corner is the convex mirror image and must
        // not seed a phantom slot.  Same principle as drill_hole's
        // machinability gate and mill_rect_pocket's corner gate.
        try {
            const gp_Pnt cc = wp.faceCenter(fIdx);
            const gp_Dir nn = wp.faceNormal(fIdx);
            const gp_Pnt aa = cyl.Axis().Location();
            const gp_Vec rel(aa, cc);
            const double along = rel.X() * adir.X() + rel.Y() * adir.Y()
                               + rel.Z() * adir.Z();
            gp_Vec radial = rel - gp_Vec(adir) * along;
            if (radial.Magnitude() < 1e-9) continue;
            radial.Normalize();
            const double ndot = nn.X() * radial.X() + nn.Y() * radial.Y()
                              + nn.Z() * radial.Z();
            if (ndot > -0.2) continue;   // convex/tangential → not a slot end
        } catch (const Standard_Failure&) {
            continue;
        }

        // Track each circular edge's coordinate ALONG the cylinder axis (not Z),
        // and record the entry-plane circle centre (the edge at the larger axial
        // coordinate, where the slot opens to the surface).
        // First pass: find the axial extent (aHi = entry side, aLo = bottom).
        const gp_Pnt axisLoc = cyl.Axis().Location();
        double aHi = -1e30, aLo = 1e30;
        gp_Pnt entryCentre = axisLoc;
        for (TopExp_Explorer exp(cf, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Pnt mid = crv.Value((crv.FirstParameter() + crv.LastParameter()) / 2.0);
            const double a = gp_Vec(axisLoc, mid).Dot(gp_Vec(adir));
            if (a > aHi) { aHi = a; entryCentre = crv.Circle().Location(); }
            aLo = std::min(aLo, a);
        }
        // Second pass: among the axis-normal planar faces adjacent to this end
        // cylinder, the slot has TWO that matter — the ENTRY surface (the face
        // the slot opens onto; the LARGEST) and the FLOOR (the pocket bottom; a
        // SMALL face).  We classify by area: the floor is the smallest axis-
        // normal adjacent plane, the entry is the largest.  Both ends then share
        // the floor.  This is orientation- AND axis-sign-independent (unlike
        // keying to aHi/aLo, where a flipped cylinder axis swaps entry/floor).
        int bottomFaceIdx = -1, entryFaceIdx = -1;
        double bottomArea = 1e30, entryArea = -1.0;
        for (TopExp_Explorer exp(cf, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            if (!edgeFaces.Contains(e)) continue;
            const auto& adj = edgeFaces.FindFromKey(e);
            for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                const TopoDS_Face& af = TopoDS::Face(it.Value());
                if (af.IsSame(cf)) continue;
                if (BRepAdaptor_Surface(af).GetType() != GeomAbs_Plane) continue;
                const int idx = findPlanarFaceIndex(wp, af);
                if (idx < 0) continue;
                const gp_Dir fn = wp.faceNormal(idx);
                if (std::abs(std::abs(fn.Dot(adir)) - 1.0) > 1e-2) continue;
                const double area = wp.faceArea(idx);
                if (area < bottomArea) { bottomArea = area; bottomFaceIdx = idx; }
                if (area > entryArea)  { entryArea  = area; entryFaceIdx  = idx; }
            }
        }
        if (bottomFaceIdx < 0) continue;
        // The end's entry circle centre = its axis projected onto the entry
        // (largest) face plane, so start/end carry the real entry-surface point.
        gp_Pnt endEntry = entryCentre;
        if (entryFaceIdx >= 0) {
            const gp_Pnt fc = wp.faceCenter(entryFaceIdx);
            const gp_Dir fn = wp.faceNormal(entryFaceIdx);
            // Project the cylinder axis location onto the entry plane along adir.
            const double t = gp_Vec(axisLoc, fc).Dot(gp_Vec(fn)) /
                             std::max(1e-9, gp_Vec(adir).Dot(gp_Vec(fn)));
            endEntry = gp_Pnt(axisLoc.X() + adir.X() * t,
                              axisLoc.Y() + adir.Y() * t,
                              axisLoc.Z() + adir.Z() * t);
        }
        bottomGroups[bottomFaceIdx].push_back({
            fIdx, cyl.Radius(), endEntry, adir, aHi, aLo
        });
    }

    // For each group of exactly 2 DISTINCT end positions with matching radii →
    // mill_slot candidate.  (OCCT may split one semicircular end into several
    // co-radial / co-axial cylindrical faces at the SAME position; collapse
    // those to one end before counting.)
    for (const auto& [bottomIdx, rawEntries] : bottomGroups) {
        std::vector<CylEntry> entries;
        for (const auto& e : rawEntries) {
            bool merged = false;
            for (const auto& kept : entries)
                if (e.axisBase.Distance(kept.axisBase) < 0.5 &&
                    std::abs(e.radius - kept.radius) < 1e-3) { merged = true; break; }
            if (!merged) entries.push_back(e);
        }
        if (entries.size() != 2) continue;
        if (std::abs(entries[0].radius - entries[1].radius) > 1e-3) continue;
        // Both ends share the slot axis; recover it (and flip toward the bottom
        // = the machining/cut direction).  depth = axial span of the end walls.
        const double r = entries[0].radius;
        const double depth = std::abs(entries[0].aTop - entries[0].aBot);

        // The cut direction is the machining axis pointing INTO the part: from
        // the entry-plane circle centre toward the slot floor.  Deriving it from
        // the floor geometry (not the cylinder axis sign, which OCCT may flip)
        // makes it unambiguous for any orientation.
        const gp_Pnt floorCentre = wp.faceCenter(bottomIdx);
        gp_Vec cutVec(entries[0].axisBase, floorCentre);
        // Project onto the bore axis to get a clean direction (ignore any in-plane
        // offset of the floor centre).
        const gp_Vec aDirVec(entries[0].axisDir);
        const double proj = cutVec.Dot(aDirVec);
        gp_Vec dirVec = aDirVec * (proj >= 0.0 ? 1.0 : -1.0);
        if (dirVec.Magnitude() < 1e-9) dirVec = aDirVec;
        dirVec.Normalize();
        const gp_Dir cutDir(dirVec);

        json recovered = {
            { "start_x_mm",   entries[0].axisBase.X() },
            { "start_y_mm",   entries[0].axisBase.Y() },
            { "start_z_mm",   entries[0].axisBase.Z() },
            { "end_x_mm",     entries[1].axisBase.X() },
            { "end_y_mm",     entries[1].axisBase.Y() },
            { "end_z_mm",     entries[1].axisBase.Z() },
            { "axis_dir",     { cutDir.X(), cutDir.Y(), cutDir.Z() } },
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
