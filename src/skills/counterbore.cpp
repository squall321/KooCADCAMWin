// @lat: [[engine/skills#counterbore]]

#include "counterbore.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopAbs.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <optional>

namespace koocadcam::skill::counterbore {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pilot_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "counterbore pilot diameter " + std::to_string(in.pilot_dia_mm) +
              " mm < min 0.8 mm");
    }
    if (in.pilot_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "counterbore pilot diameter must be > 0");
    }
    if (in.seat_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "counterbore seat diameter must be > 0");
    }
    if (in.pilot_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "counterbore pilot depth must be > 0");
    }
    if (in.seat_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "counterbore seat depth must be > 0");
    }
    if (in.seat_dia_mm <= in.pilot_dia_mm) {
        r.add("DFM-COUNTERBORE-GEOM", "error",
              "counterbore seat dia " + std::to_string(in.seat_dia_mm) +
              " must be > pilot dia " + std::to_string(in.pilot_dia_mm) +
              " (else feature is not a counterbore)");
    }
    if (in.seat_depth_mm >= in.pilot_depth_mm) {
        r.add("DFM-COUNTERBORE-GEOM", "error",
              "counterbore seat depth " + std::to_string(in.seat_depth_mm) +
              " must be < pilot depth " + std::to_string(in.pilot_depth_mm) +
              " (else feature is not a counterbore)");
    }
    const double ratio = (in.pilot_dia_mm > 0.0) ? (in.pilot_depth_mm / in.pilot_dia_mm) : 0.0;
    if (ratio > 8.0) {
        r.add("DFM-PECK", "warning",
              "pilot depth/dia ratio " + std::to_string(ratio) +
              " > 8 — peck drilling recommended (tool length / chip evacuation)");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    // 1) DFM gate
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "counterbore DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // 2) Resolve entry face datum
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("counterbore: entry_face datum unresolved");

    // 3) Compute drill geometry (same pattern as drill_hole)
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // True entry point = where the tool axis pierces the resolved entry-face
    // plane (correct for any axis, including tilted).
    const gp_Pnt entryPlanePoint =
        entryPointOnFacePlane(wp, *entryId, in.position_x_mm, in.position_y_mm,
                              adir, zMin, zMax);
    // toolStart = the cutter launch point, kEntryOverhang OUTSIDE the surface
    // (back along -adir from the true entry point).
    gp_Pnt toolStart(entryPlanePoint.X() - adir.X() * kEntryOverhang,
                     entryPlanePoint.Y() - adir.Y() * kEntryOverhang,
                     entryPlanePoint.Z() - adir.Z() * kEntryOverhang);

    // 4) Build the two cutters.  Both share the same gp_Ax2 axis.
    const gp_Ax2 toolAx(toolStart, adir);

    // Pilot — goes the full pilot depth (plus overhang for clean entry).
    const TopoDS_Shape pilotTool =
        pr::cylinder(toolAx, in.pilot_dia_mm / 2.0, in.pilot_depth_mm + kEntryOverhang);

    // Seat — goes only seat_depth (plus overhang).
    const TopoDS_Shape seatTool =
        pr::cylinder(toolAx, in.seat_dia_mm / 2.0, in.seat_depth_mm + kEntryOverhang);

    // 5) Pilot and seat are CONCENTRIC overlapping cylinders.  OCCT's
    //    BRepAlgoAPI_Cut on a compound containing overlapping solids can
    //    miss interior overlap; fuse them into a single connected cutter
    //    first, then perform a single cut.
    const TopoDS_Shape fusedCutter = pr::fuse(pilotTool, seatTool);
    const TopoDS_Shape newShape    = pr::cut(wp.shape(), fusedCutter);

    // 6) Build signature.  Stamp the entry-plane Z (position_z_mm) so downstream
    //    CAM machines from the real surface, not Z=0.  entryPlanePoint is the
    //    exact point where the axis crosses the entry surface in BOTH the ±Z
    //    shortcut and the general tilted-axis branch.
    json params = {
        { "entry_face_kind", "resolved_id" },
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "position_z_mm",   entryPlanePoint.Z() },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "pilot_dia_mm",    in.pilot_dia_mm },
        { "pilot_depth_mm",  in.pilot_depth_mm },
        { "seat_dia_mm",     in.seat_dia_mm },
        { "seat_depth_mm",   in.seat_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     2 },             // pilot + seat
        { "step_annular_face_present",  true },          // the planar annulus at seat depth
        { "bottom_planar_face_present", true },          // pilot's blind bottom
        { "pilot_dia_mm",               in.pilot_dia_mm },
        { "seat_dia_mm",                in.seat_dia_mm },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    // CAM exec: typically a dedicated counterbore tool, or pilot drill +
    // endmill / cbore for the seat.  We report it as a counterbore tool;
    // the "extra" field encodes the alternative two-tool sequence.
    tooling.tool_type        = "counterbore";
    tooling.tool_dia_mm      = in.seat_dia_mm;        // largest engagement
    tooling.tool_length_mm   = in.pilot_depth_mm * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double pilotR = in.pilot_dia_mm / 2.0;
    const double seatR  = in.seat_dia_mm  / 2.0;
    tooling.stock_removed_mm3 =
        M_PI * pilotR * pilotR * in.pilot_depth_mm +
        M_PI * (seatR * seatR - pilotR * pilotR) * in.seat_depth_mm;
    tooling.est_cycle_time_s = std::max(1.0, in.pilot_depth_mm / 50.0);
    tooling.extra = {
        { "two_tool_sequence", {
            { { "tool_type", "drill" },   { "tool_dia_mm", in.pilot_dia_mm } },
            { { "tool_type", "endmill" }, { "tool_dia_mm", in.seat_dia_mm  } },
        } }
    };

    FeatureSignature sig {
        kSkillId,
        params,
        pattern,
        tooling,
    };

    // 7) Build new workpiece, register feature
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::counterbore applied: pilot {}x{}, seat {}x{}, faces {}→{}",
                  in.pilot_dia_mm, in.pilot_depth_mm,
                  in.seat_dia_mm, in.seat_depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern matching strategy (geometric, axis-orientation independent):
//   1. Collect every CONCAVE cylindrical bore face (concavity gate rejects
//      convex rods / stepped shafts / bosses + freeform slivers) with its
//      radius, axis, both rim centers, and the largest planar face each rim
//      opens onto.  All tolerances are size-adaptive (∝ radius).
//   2. For every coaxial pair (parallel axis + shared infinite-line within a
//      radius-scaled tolerance) with distinct radii:
//        - SEAT  = larger radius, PILOT = smaller radius.
//        - ENTRY = the seat rim that opens onto the LARGEST planar workpiece
//          face (adjacency, NOT an assumed +Z) — works for a counterbore cut
//          from any face, including the bottom.
//        - The other seat rim is the step; the drill axis points entry → step.
//   3. Recover: pilot_dia = 2·r_small, seat_dia = 2·r_large, seat_depth =
//      proj(step), pilot_depth = proj(pilot deepest rim), all from the entry
//      origin.  Reject unless seat_depth < pilot_depth and the pilot starts at
//      the seat step (clean coaxial junction).

namespace {

// OCCT 8.0: use NCollection types directly; TopTools_* typedefs are deprecated.
using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

// Machinability / concavity gate — is this cylindrical face an actual BORE
// wall (concave), not a convex rod / boss / pin?
//
// A bore is CONCAVE: the solid is OUTSIDE the cylinder, so the face's
// orientation-aware outward normal points INWARD, toward the axis.  A stepped
// shaft / pin / boss is CONVEX (normal points outward) and must be rejected —
// otherwise two coaxial convex cylinders of different radii (a stepped shaft)
// would be mis-recognized as a counterbore.  A freeform tessellation sliver
// barely wraps the axis and is also rejected.  Mirrors drill_hole's
// isDrillableHoleWall().
bool isConcaveBoreWall(const TopoDS_Face& f, const gp_Cylinder& cyl)
{
    BRepAdaptor_Surface s(f);

    // (a) Sliver guard: the wall must wrap a meaningful arc of the axis.  A
    //     real bore is a full cylinder (2π) or a few face-splits (≥ ~90° each).
    const double uSpan = s.LastUParameter() - s.FirstUParameter();
    if (uSpan < 0.45 * M_PI) return false;   // < 81° → freeform patch, not a wall

    // (b) Concavity: the solid's outward normal points toward the axis.
    const double u = 0.5 * (s.FirstUParameter() + s.LastUParameter());
    const double v = 0.5 * (s.FirstVParameter() + s.LastVParameter());
    gp_Pnt P; gp_Vec du, dv;
    s.D1(u, v, P, du, dv);
    gp_Vec n = du.Crossed(dv);
    if (n.Magnitude() < 1e-9) return false;
    n.Normalize();
    if (f.Orientation() == TopAbs_REVERSED) n.Reverse();

    const gp_Pnt aLoc = cyl.Axis().Location();
    const gp_Vec aDir(cyl.Axis().Direction());
    gp_Vec ap(aLoc, P);
    const gp_Vec radial = ap - aDir * ap.Dot(aDir);   // outward from the axis
    if (radial.Magnitude() < 1e-9) return false;
    return n.Dot(radial) < 0.0;                        // inward outward-normal ⇒ bore
}

struct CylInfo
{
    int        faceIdx = -1;
    double     radius  = 0.0;
    gp_Ax1     axis;
    gp_Pnt     topCenter;       // ring with smaller proj along axis direction
    gp_Pnt     botCenter;       // ring with larger  proj along axis direction
    double     topAdjPlanar = 0.0;  // area of largest planar face the top ring opens onto
    double     botAdjPlanar = 0.0;  // area of largest planar face the bot ring opens onto
    double     length = 0.0;
};

// Largest planar face (by area) adjacent to edge `e`, excluding `self`.  This
// is the workpiece surface that a circular rim "opens onto" — large for an
// entry face, ≈ π r² (the step annulus / blind bottom) for an interior rim.
double largestAdjacentPlanarArea(const EdgeFaceMap& edgeFaces,
                                 const TopoDS_Edge& e,
                                 const TopoDS_Face& self)
{
    double area = 0.0;
    if (!edgeFaces.Contains(e)) return area;
    const auto& adj = edgeFaces.FindFromKey(e);
    for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
        const TopoDS_Face& af = TopoDS::Face(it.Value());
        if (af.IsSame(self)) continue;
        if (BRepAdaptor_Surface(af).GetType() != GeomAbs_Plane) continue;
        GProp_GProps gp; BRepGProp::SurfaceProperties(af, gp);
        area = std::max(area, gp.Mass());
    }
    return area;
}

// Collect CONCAVE cylindrical bore faces with their bounding-circle endpoints
// and the planar-face adjacency at each rim (so entry can be picked by which
// rim opens onto the larger workpiece face, axis-orientation independent).
std::vector<CylInfo> collectCylinders(const Workpiece& wp, const EdgeFaceMap& edgeFaces)
{
    std::vector<CylInfo> out;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();

        // Concavity gate: only inward-facing bore walls — reject convex
        // rods/bosses (a stepped shaft) and freeform slivers.
        if (!isConcaveBoreWall(cylFace, cyl)) continue;

        CylInfo info;
        info.faceIdx = fIdx;
        info.radius  = cyl.Radius();
        info.axis    = cyl.Axis();

        const gp_Dir adir = info.axis.Direction();
        // Size-adaptive radius match: works from sub-mm bores to large seats
        // and tolerates STEP-rebuild noise (a fixed 1e-3 is too tight for a
        // re-tessellated face and too loose for a 0.5 mm pilot).
        const double radTol = std::max(1e-4, 1e-3 * info.radius);

        auto projOnAxis = [&](const gp_Pnt& p) {
            return (p.X() - info.axis.Location().X()) * adir.X() +
                   (p.Y() - info.axis.Location().Y()) * adir.Y() +
                   (p.Z() - info.axis.Location().Z()) * adir.Z();
        };

        // Find circular rims on this face, along the same axis, recording the
        // largest planar face each rim opens onto.
        struct Rim { gp_Pnt center; double adjPlanar; double proj; };
        std::vector<Rim> rims;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(adir)) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - info.radius) > radTol) continue;
            rims.push_back({ c.Location(),
                             largestAdjacentPlanarArea(edgeFaces, e, cylFace),
                             projOnAxis(c.Location()) });
        }
        if (rims.size() < 2) continue;

        const auto minIt = std::min_element(rims.begin(), rims.end(),
            [](const Rim& a, const Rim& b){ return a.proj < b.proj; });
        const auto maxIt = std::max_element(rims.begin(), rims.end(),
            [](const Rim& a, const Rim& b){ return a.proj < b.proj; });
        // Axis direction is a geometric label only (a cylinder's axis can point
        // either way after STEP import); "top"/"bot" here just mean the two
        // extreme rims along that direction.  Entry vs deep is decided later by
        // ADJACENCY, never by an assumed +Z.
        info.topCenter    = minIt->center;
        info.botCenter    = maxIt->center;
        info.topAdjPlanar = minIt->adjPlanar;
        info.botAdjPlanar = maxIt->adjPlanar;
        info.length       = info.botCenter.Distance(info.topCenter);
        out.push_back(info);
    }
    return out;
}

// Two cylinders share an axis (infinite-line) within size-adaptive tolerance?
bool sameAxis(const CylInfo& a, const CylInfo& b, double angTolDeg = 0.5)
{
    const gp_Dir da = a.axis.Direction();
    const gp_Dir db = b.axis.Direction();
    const double dot = std::abs(da.X() * db.X() + da.Y() * db.Y() + da.Z() * db.Z());
    if (dot < std::cos(angTolDeg * M_PI / 180.0)) return false;
    // Position check: distance from b.axis.Location() to the infinite line
    // through a.axis.Location() along da.  Tolerance scales with the larger
    // radius so a big seat survives STEP-rebuild axis jitter while a tiny
    // pilot still demands genuine coaxiality.
    const gp_Pnt& pa = a.axis.Location();
    const gp_Pnt& pb = b.axis.Location();
    gp_Vec v(pa, pb);
    gp_Vec axV(da.X(), da.Y(), da.Z());
    gp_Vec perp = v - axV * v.Dot(axV);
    const double posTolMm = std::max(1e-3, 1e-3 * std::max(a.radius, b.radius));
    return perp.Magnitude() < posTolMm;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const EdgeFaceMap edgeFaces = buildEdgeFaceMap(wp.shape());
    const auto cyls = collectCylinders(wp, edgeFaces);
    if (cyls.size() < 2) return out;

    std::vector<bool> consumed(cyls.size(), false);

    for (size_t i = 0; i < cyls.size(); ++i) {
        if (consumed[i]) continue;
        for (size_t j = i + 1; j < cyls.size(); ++j) {
            if (consumed[j]) continue;
            if (!sameAxis(cyls[i], cyls[j])) continue;

            const CylInfo& small = (cyls[i].radius < cyls[j].radius) ? cyls[i] : cyls[j];
            const CylInfo& large = (cyls[i].radius < cyls[j].radius) ? cyls[j] : cyls[i];

            // Distinct radii required (size-adaptive on the larger seat).
            const double diaTol = std::max(1e-4, 1e-3 * large.radius);
            if (std::abs(large.radius - small.radius) < diaTol) continue;

            // ── Entry identification by ADJACENCY, not by an assumed +Z ──────
            //
            // The four rims of the pair each open onto a planar face.  The seat
            // entry rim opens onto the LARGE workpiece surface (the part face
            // where the screw head seats); the step annulus and pilot bottom
            // open onto ≈ π r² planar faces.  Whichever seat rim has the
            // largest planar neighbour is the entry — this is what makes the
            // recognizer work for a counterbore cut from ANY face, including
            // the bottom (axis pointing +Z), not just a top-face (-Z) bore.
            //
            // Pick the seat entry rim by max adjacent planar area; the OTHER
            // seat rim is the step.  This is robust to STEP face-area noise
            // because the entry surface (50×50 block face) dwarfs the annulus.
            const bool seatTopIsEntry = large.topAdjPlanar >= large.botAdjPlanar;
            const gp_Pnt seatEntryCenter = seatTopIsEntry ? large.topCenter : large.botCenter;
            const gp_Pnt seatStepCenter  = seatTopIsEntry ? large.botCenter : large.topCenter;

            // Drill direction = entry → into the material (entry → step), with
            // a meaningful sign for ANY axis orientation.
            gp_Vec drillVec(seatEntryCenter, seatStepCenter);
            if (drillVec.Magnitude() < 1e-6) continue;
            drillVec.Normalize();
            const gp_Dir adir(drillVec);

            // PHYSICAL gate (B1.5 follow-up): the tool must be able to REACH
            // the entry — the point just before the entry rim along the
            // drill axis must be open air (or an open cavity), never solid
            // material.  Without this, a blind UNDERCUT bore (narrow upper,
            // wide lower — bore_with_shelf geometry) recognizes as a
            // counterbore seen upside-down with the blind floor as "entry",
            // shadowing the correct bws candidate in dedupe (corpus B7
            // measurement, 2026-06-11).
            {
                const gp_Pnt probe(seatEntryCenter.X() - adir.X() * 0.5,
                                   seatEntryCenter.Y() - adir.Y() * 0.5,
                                   seatEntryCenter.Z() - adir.Z() * 0.5);
                BRepClass3d_SolidClassifier cls(wp.shape());
                cls.Perform(probe, 1e-6);
                if (cls.State() == TopAbs_IN) continue;   // entry buried in material
            }

            // Project every relevant center onto the drill axis; entry is the
            // origin (proj 0), deeper is positive.
            auto proj = [&](const gp_Pnt& p) {
                return (p.X() - seatEntryCenter.X()) * adir.X() +
                       (p.Y() - seatEntryCenter.Y()) * adir.Y() +
                       (p.Z() - seatEntryCenter.Z()) * adir.Z();
            };

            const double seatDepth = proj(seatStepCenter);                // > 0
            if (seatDepth <= 1e-6) continue;                              // seat must descend

            // Pilot deepest rim along the drill axis → pilot depth from entry.
            const double pilotDeepProj = std::max(proj(small.topCenter), proj(small.botCenter));
            const double pilotShallowProj = std::min(proj(small.topCenter), proj(small.botCenter));
            const double pilotDepth = pilotDeepProj;

            // Counterbore geometry sanity: seat is shallower than pilot.
            if (seatDepth >= pilotDepth) continue;

            // Verify the pilot starts at the seat step (clean coaxial step):
            // the pilot's shallow rim sits at ≈ seatDepth along the axis.
            // Tolerance scales with the seat depth so STEP rebuild jitter on a
            // large feature doesn't reject a genuine step.
            const double junctionTol = std::max(1e-2, 1e-3 * seatDepth);
            const double junctionGap = std::abs(pilotShallowProj - seatDepth);
            if (junctionGap > 10.0 * junctionTol) continue;   // not a clean step

            // Confidence heuristic: penalize a loose step junction.
            double conf = 0.92;
            if (junctionGap > junctionTol) conf -= 0.2;

            json recovered = {
                { "position_x_mm",   seatEntryCenter.X() },
                { "position_y_mm",   seatEntryCenter.Y() },
                { "position_z_mm",   seatEntryCenter.Z() },   // full 3-D entry (any axis)
                { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
                { "pilot_dia_mm",    2.0 * small.radius },
                { "pilot_depth_mm",  pilotDepth },
                { "seat_dia_mm",     2.0 * large.radius },
                { "seat_depth_mm",   seatDepth },
            };
            // Claim the planar ENTRY face and the STEP ANNULUS (shoulder) in
            // matched_geometry.  dedupe's specificity rule (B1.5) replaces a
            // shadowing drill candidate only when this candidate's face set
            // STRICTLY CONTAINS the drill's — and a drill recognized on the
            // pilot reports the shoulder as ITS entry face, so without the
            // shoulder id here the counterbore stays blocked forever.
            int    entryFaceId    = -1;
            int    shoulderFaceId = -1;
            double entryArea      = 0.0;
            for (int fi = 0; fi < wp.faceCount(); ++fi) {
                if (!wp.isFacePlanar(fi)) continue;
                gp_Dir n;
                try { n = wp.faceNormal(fi); } catch (...) { continue; }
                const double nDot = std::abs(n.X() * adir.X() +
                                             n.Y() * adir.Y() +
                                             n.Z() * adir.Z());
                if (nDot < 0.99) continue;            // not ⊥ to the bore axis
                const gp_Pnt c = wp.faceCenter(fi);
                const double p = proj(c);
                if (std::abs(p) < 0.05) {
                    // Entry plane: the largest coplanar face at proj ≈ 0.
                    const double area = wp.faceArea(fi);
                    if (area > entryArea) { entryArea = area; entryFaceId = fi; }
                } else if (std::abs(p - seatDepth) <
                           std::max(0.05, junctionTol)) {
                    // Step annulus: centroid sits on the axis at seat depth.
                    const gp_Vec rel(seatEntryCenter, c);
                    const double along = rel.X() * adir.X() +
                                         rel.Y() * adir.Y() +
                                         rel.Z() * adir.Z();
                    const gp_Vec radial = rel - gp_Vec(adir) * along;
                    if (radial.Magnitude() <= large.radius)
                        shoulderFaceId = fi;
                }
            }

            json matched = {
                { "seat_cyl_face_id",  large.faceIdx },
                { "pilot_cyl_face_id", small.faceIdx },
                { "entry_center", { seatEntryCenter.X(), seatEntryCenter.Y(),
                                    seatEntryCenter.Z() } },
            };
            if (entryFaceId >= 0)    matched["entry_face_id"]    = entryFaceId;
            if (shoulderFaceId >= 0) matched["shoulder_face_id"] = shoulderFaceId;
            out.push_back(RecognizedFeature{
                kSkillId, recovered, conf, matched
            });
            consumed[i] = consumed[j] = true;
            break;
        }
    }
    return out;
}

}  // namespace koocadcam::skill::counterbore
