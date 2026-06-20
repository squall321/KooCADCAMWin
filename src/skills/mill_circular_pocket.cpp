// @lat: [[engine/skills#mill_circular_pocket]]

#include "mill_circular_pocket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Fillets.hpp"
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
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::mill_circular_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.diameter_mm < 0.8) {
        r.add("DFM-002", "error",
              "mill_circular_pocket diameter " + std::to_string(in.diameter_mm) +
              " mm < min 0.8 mm (end-mill min size)");
    }
    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "diameter must be > 0");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "depth must be > 0");
    }
    if (in.bottom_corner_r_mm < 0.0) {
        r.add("DFM-INPUT", "error", "bottom_corner_r must be >= 0");
    }
    if (in.bottom_corner_r_mm > 0.0 && in.bottom_corner_r_mm < 0.2) {
        r.add("DFM-004", "error",
              "bottom_corner_r " + std::to_string(in.bottom_corner_r_mm) +
              " mm < min R 0.2 mm");
    }
    if (in.bottom_corner_r_mm > 0.0 &&
        in.bottom_corner_r_mm > in.diameter_mm / 2.0 - 1e-6) {
        r.add("DFM-INPUT", "error",
              "bottom_corner_r must be < radius (diameter/2)");
    }
    if (in.bottom_corner_r_mm > 0.0 &&
        in.bottom_corner_r_mm > in.depth_mm - 1e-6) {
        r.add("DFM-INPUT", "error",
              "bottom_corner_r must be < depth");
    }

    const double ratio = (in.diameter_mm > 0.0) ? (in.depth_mm / in.diameter_mm) : 0.0;
    if (ratio > 5.0) {
        r.add("DFM-AR", "warning",
              "depth/dia ratio " + std::to_string(ratio) +
              " > 5 — pocket too deep for end-mill stiffness");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mill_circular_pocket DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("mill_circular_pocket: entry_face datum unresolved");

    // Build the cutter cylinder.  Cutter starts slightly above the entry face
    // (along -axis_dir) and extends `depth + kEntryOverhang` along axis_dir.
    // The bottom of the cylinder lies at exactly `depth_mm` below the entry
    // plane → flat bottom.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Resolve a Z for the entry plane: project bbox along axis_dir.
    // For the common Z-aligned drill-down case (axis = -Z) start at zMax.
    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kEntryOverhang);
        }
    } else {
        // Fallback: position the tool start on the bbox boundary along -axis_dir
        // from the requested XY.  For non-axial pockets, the caller is
        // responsible for ensuring this places the tool above the entry face.
        const double bboxDiag = std::sqrt(
            (xMax - xMin)*(xMax - xMin) +
            (yMax - yMin)*(yMax - yMin) +
            (zMax - zMin)*(zMax - zMin));
        const double margin = bboxDiag + 1.0;
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * margin,
            in.position_y_mm - adir.Y() * margin,
            (zMin + zMax)/2.0 - adir.Z() * margin);
    }

    const double toolHeight = in.depth_mm + kEntryOverhang;
    const gp_Ax2 toolAx(toolStart, adir);
    TopoDS_Shape cutter = pr::cylinder(toolAx, in.diameter_mm / 2.0, toolHeight);

    // Bottom corner fillet on the CUTTER: the cutter's bottom-edge is a
    // single circle of radius `diameter/2`.  Filleting that edge by
    // `bottom_corner_r` produces a torus blend at the bottom of the cutter,
    // which after Boolean cut becomes the bottom-corner fillet in the pocket.
    if (in.bottom_corner_r_mm > 0.0) {
        // The bottom edge of the cutter sits at parametric "bottom" of the
        // cylinder.  In world coords (for the axial top-down case) that's at
        // Z = toolStart.Z() + adir.Z() * toolHeight.
        const double zBottom = toolStart.Z() + adir.Z() * toolHeight;
        cutter = pr::filletEdges(cutter, in.bottom_corner_r_mm,
            [zBottom](const TopoDS_Edge& e, const gp_Pnt& mp) {
                (void)e;
                return std::abs(mp.Z() - zBottom) < 1e-3;
            });
    }

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // Build signature.  Emit the resolved entry-plane Z so the CAM generator
    // places the toolpath on the real surface (matches drill_hole; the
    // generator falls back to 0 for ambiguous non-vertical axes).
    const bool vertical = std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6;
    const double entryZ = vertical ? (adir.Z() < 0 ? zMax : zMin) : 0.0;
    json params = {
        { "entry_face_kind",      "resolved_id" },
        { "entry_face_id",        *entryId },
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
        { "position_z_mm",        entryZ },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
        { "diameter_mm",          in.diameter_mm },
        { "depth_mm",             in.depth_mm },
        { "bottom_corner_r_mm",   in.bottom_corner_r_mm },
    };
    const bool hasFillet = in.bottom_corner_r_mm > 0.0;
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     hasFillet ? 1 : 1 },
        { "toroidal_face_count",        hasFillet ? 1 : 0 },
        { "bottom_planar_face_present", true },
        { "circular_edge_count",        hasFillet ? 2 : 2 },
        { "diameter_mm",                in.diameter_mm },
        { "depth_mm",                   in.depth_mm },
        { "bottom_corner_r_mm",         in.bottom_corner_r_mm },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = in.diameter_mm;
    tooling.tool_length_mm    = in.depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 600.0;
    tooling.feed_per_tooth_mm = 0.025;
    tooling.stock_removed_mm3 = M_PI * (in.diameter_mm/2.0) * (in.diameter_mm/2.0)
                                * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.0, in.depth_mm * 0.5 + in.diameter_mm * 0.1);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::mill_circular_pocket applied: dia={} depth={} cornerR={} faces {}→{}",
                  in.diameter_mm, in.depth_mm, in.bottom_corner_r_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

// Concavity gate — is this cylindrical face an actual POCKET / BORE wall?
//
// A milled circular pocket is a CONCAVE cylinder: the solid lies OUTSIDE the
// cylinder, so the face's orientation-aware outward normal (into the void)
// points INWARD, toward the axis.  A rod / boss / pin / spigot is CONVEX (its
// outward normal points away from the axis) and must be rejected — otherwise
// the recognizer would report a turned post that happens to butt against a
// small flat as a "pocket".  A tessellated sliver on a freeform surface barely
// wraps the axis at all, so we also require the wall to span a meaningful arc.
// Mirrors drill_hole::isDrillableHoleWall so drill and pocket agree on what a
// concave bore wall is; the two skills are then separated by aspect ratio.
bool isConcavePocketWall(const TopoDS_Face& f, const gp_Cylinder& cyl)
{
    BRepAdaptor_Surface s(f);

    // (a) Sliver guard: the wall must wrap a meaningful arc of the axis.  A
    //     real pocket wall is a full cylinder (2π) or a few face-splits
    //     (>= ~90° each).  < 81° ⇒ freeform patch, not a machined wall.
    const double uSpan = s.LastUParameter() - s.FirstUParameter();
    if (uSpan < 0.45 * M_PI) return false;

    // (b) Concavity: the solid's outward normal points TOWARD the axis.
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
    return n.Dot(radial) < 0.0;                        // inward normal ⇒ concave bore
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
        const double radius   = cyl.Radius();
        const gp_Ax1   axis   = cyl.Axis();
        const gp_Dir   adir   = axis.Direction();

        // Concavity gate: only a concave, axis-wrapping cylinder is a pocket
        // wall.  Rejects convex rods/bosses/spigots and freeform tessellation
        // slivers (mirrors drill_hole::isDrillableHoleWall).
        if (!isConcavePocketWall(cylFace, cyl)) continue;

        // Size-adaptive tolerances: a fixed 1e-2 mm rejects sub-mm pockets and
        // over-accepts large bores, and STEP rebuild noise scales with radius.
        const double radTol = std::max(1e-4, 1e-3 * radius);

        // Each circular edge that rings the cylinder is recorded together with
        // the area of the LARGEST planar face adjacent to it — that is the
        // workpiece surface the ring opens onto: large for the open entry
        // mouth, ≈ π r² for the flat pocket bottom.  We use this adjacency
        // (NOT an "entry is higher in Z" assumption) to tell entry from bottom,
        // so the recovery is correct for a pocket milled along ANY axis.
        struct Ring { gp_Pnt center; double adjPlanarArea; };
        std::vector<Ring> rings;
        bool hasToroidalAdjacent  = false;
        double recoveredCornerR   = 0.0;
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(adir)) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - radius) > radTol) continue;
            double adjMax = 0.0;
            if (edgeFaces.Contains(e)) {
                const auto& adj = edgeFaces.FindFromKey(e);
                for (NCollection_List<TopoDS_Shape>::Iterator it(adj); it.More(); it.Next()) {
                    const TopoDS_Face& af = TopoDS::Face(it.Value());
                    if (af.IsSame(cylFace)) continue;
                    BRepAdaptor_Surface as(af);
                    if (as.GetType() == GeomAbs_Plane) {
                        GProp_GProps gp; BRepGProp::SurfaceProperties(af, gp);
                        adjMax = std::max(adjMax, gp.Mass());
                    } else if (as.GetType() == GeomAbs_Torus) {
                        hasToroidalAdjacent = true;
                        // Corner-radius end-mill ⇒ MEASURE the blend minor
                        // radius directly from the torus rather than guessing.
                        if (recoveredCornerR <= 0.0)
                            recoveredCornerR = as.Torus().MinorRadius();
                    }
                }
            }
            rings.push_back({ c.Location(), adjMax });
        }
        if (rings.size() < 2) continue;

        // A pocket must open onto at least one bounded flat: the floor (small,
        // ≈ π r²) and/or the entry mouth (large).  If NO ring opens onto a
        // planar face, this is not a flat-bottom pocket.
        bool hasPlanarAdjacent = false;
        for (const Ring& r : rings) if (r.adjPlanarArea > 0.0) hasPlanarAdjacent = true;
        if (!hasPlanarAdjacent) continue;

        // Two extreme rings along the cylinder axis — orientation-independent.
        auto projOnAxis = [&](const gp_Pnt& p) {
            return (p.X() - axis.Location().X()) * adir.X() +
                   (p.Y() - axis.Location().Y()) * adir.Y() +
                   (p.Z() - axis.Location().Z()) * adir.Z();
        };
        const Ring& loRing = *std::min_element(rings.begin(), rings.end(),
            [&](const Ring& a, const Ring& b){ return projOnAxis(a.center) < projOnAxis(b.center); });
        const Ring& hiRing = *std::max_element(rings.begin(), rings.end(),
            [&](const Ring& a, const Ring& b){ return projOnAxis(a.center) < projOnAxis(b.center); });

        // depth = wall length = cylinder-wall span to the flat bottom.
        const double depth = hiRing.center.Distance(loRing.center);
        if (depth < 1e-6) continue;

        // Distinguish from drill_hole by aspect ratio: pockets are wide
        // (depth/dia < 2).  Above 2, defer to drill_hole skill.
        const double dia = 2.0 * radius;
        const double ratio = (dia > 0.0) ? depth / dia : 0.0;
        if (ratio >= 2.0) continue;

        // Entry vs flat bottom from ADJACENCY, not axis order: the floor opens
        // onto a small (≈ π r²) face, the entry mouth onto the larger surface.
        // The recovered position is the ENTRY centre (where the tool plunges).
        const double bottomArea = M_PI * radius * radius;
        const bool loIsBottom = loRing.adjPlanarArea > 0.0 && loRing.adjPlanarArea <= bottomArea * 1.5;
        const bool hiIsBottom = hiRing.adjPlanarArea > 0.0 && hiRing.adjPlanarArea <= bottomArea * 1.5;

        const Ring* entry  = &hiRing;
        const Ring* bottom = &loRing;
        if (loIsBottom && !hiIsBottom)            { entry = &hiRing; bottom = &loRing; }
        else if (hiIsBottom && !loIsBottom)       { entry = &loRing; bottom = &hiRing; }
        else if (loRing.adjPlanarArea > hiRing.adjPlanarArea) { entry = &loRing; bottom = &hiRing; }
        // else: keep entry=hiRing (both/neither resolved → fall back to axis order)

        // The pocket-bottom planar area drives confidence: a clean flat bottom
        // is ≈ π r² (sharp corner) or π (r - rCorner)² for a corner-radius mill.
        const double expectedSharpBottom = M_PI * radius * radius;
        const double floorArea = bottom->adjPlanarArea;
        const double areaErr = (floorArea > 0.0)
            ? std::abs(floorArea - expectedSharpBottom) / std::max(1e-6, expectedSharpBottom)
            : 1.0;
        const double conf = std::clamp(0.90 - 0.5 * std::min(areaErr, 1.0), 0.4, 0.95);

        // Plunge direction points entry → bottom (sign meaningful for any axis).
        gp_Vec drillVec(entry->center, bottom->center);
        if (drillVec.Magnitude() < 1e-6) continue;
        drillVec.Normalize();

        json recovered = {
            { "position_x_mm",       entry->center.X() },
            { "position_y_mm",       entry->center.Y() },
            { "position_z_mm",       entry->center.Z() },   // full 3-D entry (any axis)
            { "axis_dir",            { drillVec.X(), drillVec.Y(), drillVec.Z() } },
            { "diameter_mm",         dia },
            { "depth_mm",            depth },
            { "bottom_corner_r_mm",  recoveredCornerR },
        };
        json matched = {
            { "cylindrical_face_id",   fIdx },
            { "top_center",            { entry->center.X(),  entry->center.Y(),  entry->center.Z()  } },
            { "bottom_center",         { bottom->center.X(), bottom->center.Y(), bottom->center.Z() } },
            { "bottom_planar_area",    floorArea },
            { "has_torus_blend",       hasToroidalAdjacent },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::mill_circular_pocket
