// @lat: [[engine/skills#bore_cylindrical]]

#include "bore_cylindrical.hpp"

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
#include <TopAbs.hxx>
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

namespace koocadcam::skill::bore_cylindrical {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.diameter_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bore_cylindrical diameter must be > 0");
    }
    // DFM-002 — minimum bore diameter (matches drill_hole floor: ISO 235:2016
    // smallest standard HSS twist-drill nominal = 0.8 mm).
    constexpr double kMinBoreDiaMm = 0.8;  // ISO 235:2016 smallest standard HSS drill
    if (in.diameter_mm < kMinBoreDiaMm) {
        r.add("DFM-002", "error",
              "bore_cylindrical diameter " + std::to_string(in.diameter_mm) +
              " mm < min 0.8 mm (ISO 235:2016 smallest standard HSS drill)");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "bore_cylindrical depth must be > 0");
    }

    // Smallest practical boring-bar shank.  Sandvik Coromant CoroBore /
    // CoroTurn 107 family starts at 6 mm shank; smaller bores reach H7
    // via drill + reamer instead.
    constexpr double kMinBoringBarDiaMm = 6.0;  // Sandvik Coromant CoroBore minimum shank
    if (in.diameter_mm > 0.0 && in.diameter_mm < kMinBoringBarDiaMm) {
        r.add("DFM-BORE-DIA", "warning",
              "bore_cylindrical diameter " + std::to_string(in.diameter_mm) +
              " mm < 6 mm — Sandvik CoroBore minimum shank is 6 mm; consider drill+ream");
    }

    // Boring-bar overhang limit (L/D ratio) — beyond this, bar deflection
    // causes chatter and taper that exceed the H7 tolerance band.
    // Standards reference: Sandvik Coromant turning handbook §C-50, table
    // "Maximum overhang for boring bars": steel bar L/D ≤ 4.  (Carbide bar
    // L/D ≤ 6, anti-vibration L/D ≤ 10 — out of scope for the default gate.)
    constexpr double kMaxBoreRatioSteel = 4.0;  // Sandvik Coromant turning handbook §C-50 (steel boring bar)
    const double ratio = (in.diameter_mm > 0.0) ? (in.depth_mm / in.diameter_mm) : 0.0;
    if (ratio > kMaxBoreRatioSteel) {
        r.add("DFM-BORE-RATIO", "warning",
              "depth/dia ratio " + std::to_string(ratio) +
              " > 4 — Sandvik §C-50: steel boring bar L/D ≤ 4 to hold H7 tolerance");
    }

    // Tolerance class validation — ISO 286-1:2010 hole-basis precision system.
    // H7/H8/H9 are the standard machined-bore classes (H7 IT7 = ±0/+15 µm @
    // Ø ≤ 50 mm; H8 IT8 = ±0/+25 µm; H9 IT9 = ±0/+40 µm).
    if (!in.tolerance_class.empty() &&
        in.tolerance_class != "H7" &&
        in.tolerance_class != "H8" &&
        in.tolerance_class != "H9") {
        r.add("DFM-INPUT", "warning",
              "unknown tolerance_class '" + in.tolerance_class +
              "' — ISO 286-1:2010 standard machined-bore classes are H7/H8/H9");
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
        std::string msg = "bore_cylindrical DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // 2) Resolve entry face datum
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("bore_cylindrical: entry_face datum unresolved");

    // 3) Compute bore geometry & start point
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Entry Z: the authored position_z_mm when given, else the true entry-face
    // plane Z (axis∩face, correct for any axis — not a Z=0 guess).  We stamp this
    // into the signature so downstream CAM machines from the real surface.
    double entryZ = in.position_z_mm;
    if (std::abs(in.position_z_mm) < 1e-9) {
        entryZ = entryPointOnFacePlane(wp, *entryId, in.position_x_mm,
                                       in.position_y_mm, adir, zMin, zMax).Z();
    }

    // Tool start = the ENTRY point (position_x/y, entryZ) pulled back a hair along
    // -adir so the cut breaks cleanly through the entry surface.  Works for ANY
    // axis — a radial (±X/±Y) side bore starts on its side face.
    gp_Pnt toolStart(
        in.position_x_mm - adir.X() * kEntryOverhang,
        in.position_y_mm - adir.Y() * kEntryOverhang,
        entryZ           - adir.Z() * kEntryOverhang);

    const double toolHeight = in.depth_mm + kEntryOverhang;

    // 4) Build cutter
    const gp_Ax2 toolAx(toolStart, adir);
    const TopoDS_Shape cutter = pr::cylinder(toolAx, in.diameter_mm / 2.0, toolHeight);

    // 5) Cut
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // 6) Signature
    json params = {
        { "entry_face_kind", "resolved_id" },
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "position_z_mm",   entryZ },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "diameter_mm",     in.diameter_mm },
        { "depth_mm",        in.depth_mm },
        { "tolerance_class", in.tolerance_class },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     1 },
        { "circular_edge_count",        2 },
        { "bottom_planar_face_present", true },
        { "diameter_mm",                in.diameter_mm },
        { "depth_mm",                   in.depth_mm },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
        { "tolerance_class",            in.tolerance_class },
    };

    // Boring uses slower SFM than drilling.  Carbide insert, single point.
    ToolingMeta tooling;
    tooling.tool_type        = "boring_bar";
    tooling.tool_dia_mm      = in.diameter_mm;
    tooling.tool_length_mm   = in.depth_mm * 1.5 + 10.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 1;             // single-point boring tool
    tooling.cutting_speed_sfm = 100.0;        // slower than drilling
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = M_PI * (in.diameter_mm / 2.0) * (in.diameter_mm / 2.0)
                              * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(2.0, in.depth_mm / 25.0);
    tooling.extra = json{
        { "tolerance_class", in.tolerance_class },
    };

    FeatureSignature sig {
        kSkillId,
        params,
        pattern,
        tooling,
    };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::bore_cylindrical applied: dia={} depth={} tol={} faces {}→{}",
                  in.diameter_mm, in.depth_mm, in.tolerance_class,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Strategy mirrors drill_hole::recognize, but uses recovered metrics to
// decide whether the topology represents a bore (dia ≥ 6 mm AND
// depth/dia ≤ 4) versus a drilled hole.  Confidence is high when both
// criteria match, low when only one does (ambiguous), and the candidate
// is skipped when neither matches.
//
// Hardening for FOREIGN STEP files (real downloaded CAD, not just synthetic
// round-trips):
//   * Concavity gate (isBoreWall) — an internal bore is a CONCAVE cylinder
//     (solid OUTSIDE the wall, outward normal points toward the axis).  A
//     convex rod / boss / shaft is rejected; a freeform tessellation sliver
//     that barely wraps the axis is rejected.  Mirrors
//     drill_hole::isDrillableHoleWall.
//   * All measurements come straight off BRepAdaptor_Surface::Cylinder()
//     (radius/axis) and BRepAdaptor_Curve::Circle() (ring radius/centre).
//   * Axis-orientation-independent: entry vs. blind bottom is decided by
//     ADJACENCY (which ring opens onto the larger planar workpiece face),
//     never by "highest Z".  The full 3-D entry point is recovered.
//   * Size-adaptive tolerances: max(1e-4, 1e-3·radius) instead of a fixed
//     1e-3, so it works from small precision bores to large ones and
//     tolerates STEP rebuild noise.

namespace {

using EdgeFaceMap = NCollection_IndexedDataMap<
    TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

EdgeFaceMap buildEdgeFaceMap(const TopoDS_Shape& shape)
{
    EdgeFaceMap m;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m);
    return m;
}

// Machinability gate — is this cylindrical face an actual BORE wall?
//
// A bore is CONCAVE: the solid is outside the cylinder, so the face's outward
// (into-the-void) normal points INWARD, toward the axis.  A rod / boss / pin /
// shaft is CONVEX (normal points outward) and must be rejected — otherwise the
// recognizer reports phantom "bores" on every turned shaft of a foreign STEP
// assembly.  A tessellated sliver on a freeform surface barely wraps the axis
// and is rejected by the U-span guard.  Identical technique to
// drill_hole::isDrillableHoleWall.
bool isBoreWall(const TopoDS_Face& f, const gp_Cylinder& cyl)
{
    BRepAdaptor_Surface s(f);

    // (a) Sliver guard: the wall must wrap a meaningful arc of the axis.  A real
    //     bore is a full cylinder (2π) or a few face-splits (≥ ~90° each).
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

        // Concavity gate: only a concave, axis-wrapping cylinder is a bore wall
        // (rejects convex rods/bosses/shafts and freeform tessellation slivers).
        if (!isBoreWall(cylFace, cyl)) continue;

        // Gather the circular edges that ring this cylinder.  Record each ring's
        // centre and the area of the LARGEST planar face adjacent to it — the
        // workpiece surface it opens onto (large for an entry, ≈ π r² for a blind
        // bottom).  Radius / axis-parallel tolerances are size-adaptive so the
        // match survives STEP rebuild noise and scales from small to large bores.
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
        if (rings.size() < 2) continue;

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
        const double depth    = hiRing.center.Distance(loRing.center);
        if (depth < 1e-6) continue;
        const double diameter = 2.0 * radius;

        // Entry vs. blind-bottom from adjacency, NOT from Z order:
        //   - a ring opening onto a small (≈ π r²) face is the blind bottom;
        //   - the entry is the ring opening onto the larger workpiece surface;
        //   - through-bore ⇔ neither ring is a blind bottom.
        const double bottomArea = M_PI * radius * radius;
        const bool loIsBottom = loRing.adjPlanarArea > 0.0 && loRing.adjPlanarArea <= bottomArea * 1.5;
        const bool hiIsBottom = hiRing.adjPlanarArea > 0.0 && hiRing.adjPlanarArea <= bottomArea * 1.5;
        const bool through    = !loIsBottom && !hiIsBottom;

        // bore_cylindrical synthesis is BLIND; a through-bore is line_boring /
        // drill_through territory, so skip it here.
        if (through) continue;

        const Ring* entry  = &hiRing;
        const Ring* bottom = &loRing;
        if (loIsBottom)            { entry = &hiRing; bottom = &loRing; }
        else if (hiIsBottom)       { entry = &loRing; bottom = &hiRing; }
        else if (loRing.adjPlanarArea > hiRing.adjPlanarArea) { entry = &loRing; bottom = &hiRing; }

        // Bore direction: entry → bottom (into the material).  Sign is now
        // meaningful for ANY axis orientation, not just a Z+ stock.
        gp_Vec boreVec(entry->center, bottom->center);
        if (boreVec.Magnitude() < 1e-6) continue;
        boreVec.Normalize();

        // Bore signature: blind, precision diameter, low aspect ratio.
        const double ratio = (diameter > 0.0) ? (depth / diameter) : 0.0;
        const bool diaOk   = diameter >= 6.0;
        const bool ratioOk = ratio <= 4.0;
        if (!diaOk && !ratioOk) continue;

        // Confidence:
        //   both criteria → 0.9 (strong bore signature)
        //   only diameter → 0.55 (could be a shallow drill)
        //   only ratio    → 0.45 (small dia, but shallow — likely drill+ream)
        double confidence;
        if (diaOk && ratioOk)      confidence = 0.9;
        else if (diaOk)            confidence = 0.55;
        else                       confidence = 0.45;

        json recovered = {
            { "position_x_mm",   entry->center.X() },
            { "position_y_mm",   entry->center.Y() },
            { "position_z_mm",   entry->center.Z() },   // full 3-D entry point (any axis)
            { "axis_dir",        { boreVec.X(), boreVec.Y(), boreVec.Z() } },
            { "diameter_mm",     diameter },
            { "depth_mm",        depth },
            { "tolerance_class", "H7" },   // default precision class
        };
        // Angular wrap of this cylindrical face (radians) — lets downstream
        // grammar (coaxial step-bore) demand a revolution-complete section and
        // reject partial concave arcs, mirroring drill_hole's cyl_wrap_rad.
        const double wrapRad = surf.LastUParameter() - surf.FirstUParameter();
        json matched = {
            { "cylindrical_face_id", fIdx },
            { "cyl_wrap_rad", wrapRad },
            { "top_center", { entry->center.X(),  entry->center.Y(),  entry->center.Z()  } },
            { "bot_center", { bottom->center.X(), bottom->center.Y(), bottom->center.Z() } },
            { "depth_dia_ratio", ratio },
        };
        out.push_back(RecognizedFeature{
            kSkillId, recovered, confidence, matched
        });
    }
    return out;
}

}  // namespace koocadcam::skill::bore_cylindrical
