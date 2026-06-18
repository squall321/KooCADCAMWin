// @lat: [[engine/skills#drill_hole]]

#include "drill_hole.hpp"

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
#include <TopAbs.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Vec.hxx>

#include <limits>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::drill_hole {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    // DFM-002 — minimum drill diameter.
    // Standards reference: ISO 235:2016 (parallel-shank twist drills) lists
    // a smallest standard HSS twist-drill nominal of 0.8 mm; carbide micro-
    // drills extend down to 0.3 mm but are out of the general-purpose range.
    // We adopt 0.8 mm as the catalog floor for "standard HSS drill". (Cf.
    // also ISO 286 fit-class system, which presumes ≥ 0.8 mm bores.)
    constexpr double kMinDrillDiaMm = 0.8;  // ISO 235:2016 smallest standard HSS twist-drill
    if (in.diameter_mm < kMinDrillDiaMm) {
        r.add("DFM-002", "error",
              "drill_hole diameter " + std::to_string(in.diameter_mm) +
              " mm < min 0.8 mm (ISO 235:2016 smallest standard HSS drill)");
    }
    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "drill_hole diameter must be > 0");
    }
    if (!in.through_hole && in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "blind drill depth must be > 0");
    }
    // DFM-PECK — depth/dia ratio threshold above which peck cycle (G83) is
    // required for chip evacuation.  Sandvik Coromant Modern Metal Cutting
    // handbook (§4-2 "Drilling") recommends peck drilling for L/D > 4 with
    // jobber-length HSS and L/D > 8 with solid carbide.  We adopt 8 as the
    // general warning threshold (carbide-friendly default).
    constexpr double kPeckRatio = 8.0;  // Sandvik Coromant Modern Metal Cutting §4-2: solid-carbide peck threshold
    const double ratio = (in.diameter_mm > 0.0) ? (in.depth_mm / in.diameter_mm) : 0.0;
    if (ratio > kPeckRatio) {
        r.add("DFM-PECK", "warning",
              "depth/dia ratio " + std::to_string(ratio) +
              " > 8 — peck drilling recommended (Sandvik Coromant §4-2: carbide L/D > 8)");
    }
    // DFM-001 — minimum wall thickness (1mm) from edge of hole to workpiece outer
    // would require BRepExtrema_DistShapeShape; deferred to step-level check
    // run by the process planner.
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    // 1) DFM gate
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "drill_hole DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // 2) Resolve entry face datum
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("drill_hole: entry_face datum unresolved");

    // 3) Compute drill geometry
    //    Tool start point = (position_x, position_y, on entry face plane)
    //    Tool extends along axis_dir for length = (depth or through-length).
    //
    //    For a through-hole we extend beyond the bbox in axis direction.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    // Start the tool slightly outside the workpiece along axis_dir reversed,
    // to guarantee a clean Boolean entry cut.
    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt toolStart(
        in.position_x_mm - adir.X() * kEntryOverhang,
        in.position_y_mm - adir.Y() * kEntryOverhang,
        // start "above" the workpiece relative to drilling direction:
        // we walk against the axis to find the entry plane.  Simplest: use
        // workpiece bbox to pick a start that is guaranteed outside.
        0.0);
    // Refine Z start: cast against -axis_dir from some far above-Z to find
    // a Z that is outside the bbox along -axis_dir direction.
    {
        // Project a point along -axis_dir, far enough to be outside bbox.
        // We assume axis_dir has a non-trivial component along Z for typical
        // top-face drilling.  For arbitrary directions, the caller must
        // ensure the position+axis goes through the workpiece.
        const double margin = bboxDiag + 1.0;
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * (margin + kEntryOverhang),
            in.position_y_mm - adir.Y() * (margin + kEntryOverhang),
            // start point along -axis_dir from a centered point
            (zMin + zMax) / 2.0 - adir.Z() * (margin + kEntryOverhang));
    }
    // For the common case (drill straight down from top face), simplify:
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kEntryOverhang);
        }
    }

    const double toolHeight = in.through_hole
        ? (bboxDiag + 2.0 * kEntryOverhang)
        : (in.depth_mm + kEntryOverhang);

    // 4) Build cutter
    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, in.diameter_mm / 2.0, toolHeight);

    // 5) Cut
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // 6) Build signature
    json params = {
        { "entry_face_kind", "resolved_id" },
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "diameter_mm",     in.diameter_mm },
        { "depth_mm",        in.depth_mm },
        { "through_hole",    in.through_hole },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        in.through_hole ? 2 : 2 },
        { "bottom_planar_face_present", !in.through_hole },
        { "diameter_mm",                in.diameter_mm },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "drill";
    tooling.tool_dia_mm      = in.diameter_mm;
    tooling.tool_length_mm   = in.depth_mm * 1.5 + 5.0;   // rule of thumb
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 300.0;  // typical aluminum
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = M_PI * (in.diameter_mm / 2.0) * (in.diameter_mm / 2.0)
                              * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.0, in.depth_mm / 50.0);  // rough

    FeatureSignature sig {
        kSkillId,
        params,
        pattern,
        tooling,
    };

    // 7) Build new workpiece, register feature (preserve the chain history so
    //    wp.features() stays the FULL build history, per the Workpiece contract).
    //    validateOrHeal gates the Boolean output (heal-or-throw) so an invalid
    //    BRep never silently becomes a "valid" Workpiece.
    auto wpNew = std::make_shared<Workpiece>(validateOrHeal(newShape, "drill_hole"), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::drill_hole applied: dia={} depth={} faces {}→{}",
                  in.diameter_mm, in.depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern matching strategy:
//   1. Iterate every cylindrical face F.
//   2. Collect all circular edges bounding F (top + bottom).
//   3. Validate the topology: exactly 2 circles, both perpendicular to
//      cylinder axis, same radius as F.
//   4. Recover parameters:
//      - diameter   = 2 × cylinder radius
//      - axis_dir   = cylinder axis direction (project onto -Z conventionally)
//      - position   = projection of (top circle center) onto entry-face plane
//      - depth      = |top_center - bottom_center|
//      - through    = both circles share faces with the original workpiece
//                     boundary (heuristic: detect by checking adjacency to
//                     non-cylinder faces).

namespace {

// OCCT 8.0: use NCollection types directly; TopTools_* typedefs are deprecated.
using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

// Build a map from edges to the faces that share them.  Used to find
// "adjacent planar face" for each circular edge.
EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

}  // namespace

namespace {

// Machinability gate — is this cylindrical face an actual DRILLED-HOLE wall?
//
// A hole is CONCAVE: the solid is outside the cylinder, so the face's outward
// (into-the-void) normal points INWARD, toward the axis.  A rod / boss / pin is
// CONVEX (normal points outward), and a tessellated sliver on a freeform
// surface barely wraps the axis at all.  Rejecting convex + sliver cylinders
// stops the recognizer from reporting thousands of phantom "holes" on organic
// CAD (e.g. an RC-buggy suspension), while leaving real machined holes — which
// are concave and wrap >= ~90 deg — fully recovered.
bool isDrillableHoleWall(const TopoDS_Face& f, const gp_Cylinder& cyl)
{
    BRepAdaptor_Surface s(f);

    // (a) Sliver guard: the wall must wrap a meaningful arc of the axis.  A real
    //     hole is a full cylinder (2π) or a few face-splits (≥ ~90° each).
    const double uSpan = s.LastUParameter() - s.FirstUParameter();
    if (uSpan < 0.45 * M_PI) return false;   // < 81° → freeform patch, not a wall

    // (b) Concavity: solid's outward normal points toward the axis.
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
    return n.Dot(radial) < 0.0;                        // inward outward-normal ⇒ hole
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const TopoDS_Shape& shape = wp.shape();
    const auto edgeFaces = buildEdgeFaceMap(shape);

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;

        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        const gp_Ax1 axis = cyl.Axis();

        // Machinability gate: only concave, axis-wrapping cylinders are holes
        // (rejects rods/bosses and freeform tessellation slivers).
        if (!isDrillableHoleWall(cylFace, cyl)) continue;

        // Collect the circular edges that ring the cylinder waist.  For each,
        // record its centre and the area of the LARGEST planar face adjacent to
        // it — the workpiece surface it opens onto (large for an entry/exit,
        // ≈ π r² for a blind drill bottom).  Radius match is size-adaptive so it
        // works from sub-mm holes to large bores and tolerates STEP rebuild noise.
        const double radTol = std::max(1e-4, 1e-3 * radius);
        struct Ring { gp_Pnt center; double adjPlanarArea; };
        std::vector<Ring> rings;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            // Circle must ring the cylinder (its normal parallel to the axis).
            if (std::abs(std::abs(c.Axis().Direction().Dot(axis.Direction())) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - radius) > radTol) continue;
            double adjMax = 0.0;
            if (edgeFaces.Contains(e)) {
                const auto& adj = edgeFaces.FindFromKey(e);
                for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                    const TopoDS_Face& af = TopoDS::Face(it.Value());
                    if (af.IsSame(cylFace)) continue;
                    if (BRepAdaptor_Surface(af).GetType() != GeomAbs_Plane) continue;
                    GProp_GProps gp; BRepGProp::SurfaceProperties(af, gp);
                    adjMax = std::max(adjMax, gp.Mass());
                }
            }
            rings.push_back({ c.Location(), adjMax });
        }
        if (rings.size() < 2) continue;  // not a clean drill

        // Two extreme rings along the cylinder axis (orientation-independent —
        // the axis can point any way; we never assume entry is "up" in Z).
        const gp_Dir adir = axis.Direction();
        auto projOnAxis = [&](const gp_Pnt& p) {
            return (p.X() - axis.Location().X()) * adir.X() +
                   (p.Y() - axis.Location().Y()) * adir.Y() +
                   (p.Z() - axis.Location().Z()) * adir.Z();
        };
        const Ring& loRing = *std::min_element(rings.begin(), rings.end(),
            [&](const Ring& a, const Ring& b){ return projOnAxis(a.center) < projOnAxis(b.center); });
        const Ring& hiRing = *std::max_element(rings.begin(), rings.end(),
            [&](const Ring& a, const Ring& b){ return projOnAxis(a.center) < projOnAxis(b.center); });
        const double depth = hiRing.center.Distance(loRing.center);
        if (depth < 1e-6) continue;

        // Entry vs blind-bottom from adjacency, NOT from Z order:
        //   - a ring opening onto a small (≈ π r²) face is the blind bottom;
        //   - the entry is the ring opening onto the larger workpiece surface;
        //   - through-hole ⇔ neither ring is a blind bottom.
        const double bottomArea = M_PI * radius * radius;
        const bool loIsBottom = loRing.adjPlanarArea > 0.0 && loRing.adjPlanarArea <= bottomArea * 1.5;
        const bool hiIsBottom = hiRing.adjPlanarArea > 0.0 && hiRing.adjPlanarArea <= bottomArea * 1.5;
        const bool through    = !loIsBottom && !hiIsBottom;

        const Ring* entry = &hiRing;
        const Ring* other = &loRing;
        if (loIsBottom)            { entry = &hiRing; other = &loRing; }
        else if (hiIsBottom)       { entry = &loRing; other = &hiRing; }
        else if (loRing.adjPlanarArea > hiRing.adjPlanarArea) { entry = &loRing; other = &hiRing; }

        // Drill direction points entry → into the material (sign now meaningful
        // for any axis orientation, not just a Z+ stock).
        gp_Vec drillVec(entry->center, other->center);
        if (drillVec.Magnitude() < 1e-6) continue;
        drillVec.Normalize();

        const double pos_x = entry->center.X();
        const double pos_y = entry->center.Y();
        const double pos_z = entry->center.Z();

        json recovered = {
            { "position_x_mm", pos_x },
            { "position_y_mm", pos_y },
            { "position_z_mm", pos_z },   // full 3-D entry point (any axis)
            { "axis_dir",      { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "diameter_mm",   2.0 * radius },
            { "depth_mm",      through ? 0.0 : depth },
            { "through_hole",  through },
        };
        // Angular wrap of THIS cylindrical face (radians).  A full drilled hole
        // wraps 2π — possibly across several STEP-split faces that SUM to 2π;
        // a concave corner fillet wraps only ~90°.  Downstream grammar (e.g.
        // bolt-circle) sums distinct-face wrap per hole to tell a real hole from
        // a pocket-corner fillet, whose centres can be incidentally concyclic.
        const double wrapRad = surf.LastUParameter() - surf.FirstUParameter();
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "cyl_wrap_rad", wrapRad },
            { "entry_center", { entry->center.X(), entry->center.Y(), entry->center.Z() } },
            { "far_center",   { other->center.X(), other->center.Y(), other->center.Z() } },
        };
        out.push_back(RecognizedFeature{
            kSkillId, recovered, /*confidence*/ 0.95, matched
        });
    }
    return out;
}

}  // namespace koocadcam::skill::drill_hole
