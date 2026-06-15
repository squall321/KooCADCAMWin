// @lat: [[engine/dfm-rules#ProductDFM]]
//
// Implementation moved VERBATIM from WatchFrontModel.cpp (helpers + runDFM,
// 2026-06-11) and parameterized by DFMProfile — see ProductDFM.hpp.

#include "ProductDFM.hpp"

#include "engine/probe/GeometryProbe.hpp"
#include "engine/primitives/Bbox.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <GeomLProp_CLProps.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax1.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Lin.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace koocadcam::engine::dfm {

namespace pr = koocadcam::engine::prim;

// ── DFM geometric-check helpers (anonymous namespace) ─────────────────────
// These helpers implement the topology-driven DFM rules (DFM-001/003/004/
// 011/013/017).  They are intentionally conservative: each loops over a
// bounded sample of topology elements so the overall O(n²) cost stays in
// the milli-second range on a ~200-face watch-case shape.
namespace {

// Sample a face normal at the centre of its UV parameter range.
// Returns false if the face is degenerate / can't be evaluated.
bool sampleFaceNormalAndPoint(const TopoDS_Face& f, gp_Dir& nOut, gp_Pnt& pOut)
{
    try {
        BRepAdaptor_Surface surf(f);
        const double uMid = (surf.FirstUParameter() + surf.LastUParameter()) / 2.0;
        const double vMid = (surf.FirstVParameter() + surf.LastVParameter()) / 2.0;
        gp_Pnt p;
        gp_Vec du, dv;
        surf.D1(uMid, vMid, p, du, dv);
        gp_Vec n = du.Crossed(dv);
        if (n.Magnitude() < 1e-9) return false;
        n.Normalize();
        if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
        nOut = gp_Dir(n);
        pOut = p;
        return true;
    } catch (const Standard_Failure&) {
        return false;
    } catch (const std::exception&) {
        return false;
    }
}

// DFM-001 helper: scan all face pairs; if normals oppose (dot ≤ -0.8) and
// the two faces are within the global bbox's max extent (sanity bound),
// compute their minimum point-to-point distance via BRepExtrema_DistShapeShape.
// Returns the minimum thickness observed across the entire shape (or +inf
// if no opposing pair found).  Limited to bbox-near faces by checking the
// face-centre distance against the bbox span.
double minWallThickness(const TopoDS_Shape& s)
{
    std::vector<TopoDS_Face> faces;
    faces.reserve(64);
    for (TopExp_Explorer ex(s, TopAbs_FACE); ex.More(); ex.Next())
        faces.push_back(TopoDS::Face(ex.Current()));

    if (faces.size() < 2) return std::numeric_limits<double>::infinity();

    std::vector<gp_Dir> normals(faces.size());
    std::vector<gp_Pnt> centres(faces.size());
    std::vector<bool>   ok     (faces.size(), false);
    for (std::size_t i = 0; i < faces.size(); ++i)
        ok[i] = sampleFaceNormalAndPoint(faces[i], normals[i], centres[i]);

    // Cap the pair count: 64 faces → 2016 pairs, ~10 ms on watch-case sized
    // shapes.  Above that, sample every Nth face to keep runtime bounded.
    const std::size_t kMaxFaces = 64;
    const std::size_t stride = (faces.size() > kMaxFaces)
        ? (faces.size() + kMaxFaces - 1) / kMaxFaces : 1;

    const auto bb     = pr::optimalBbox(s);
    const double diag = std::sqrt(bb.dx() * bb.dx() + bb.dy() * bb.dy()
                                  + bb.dz() * bb.dz());
    const double maxCandidateDist = diag;   // sanity upper bound

    double minD = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < faces.size(); i += stride) {
        if (!ok[i]) continue;
        for (std::size_t j = i + stride; j < faces.size(); j += stride) {
            if (!ok[j]) continue;
            // Opposing normals?  dot ≤ -0.8 → angle ≥ ~143°.
            const double dot = normals[i].X() * normals[j].X()
                             + normals[i].Y() * normals[j].Y()
                             + normals[i].Z() * normals[j].Z();
            if (dot > -0.8) continue;
            // Bbox-near filter: skip pairs whose centres are obviously far.
            const double dxC = centres[i].X() - centres[j].X();
            const double dyC = centres[i].Y() - centres[j].Y();
            const double dzC = centres[i].Z() - centres[j].Z();
            const double approx = std::sqrt(dxC*dxC + dyC*dyC + dzC*dzC);
            if (approx > maxCandidateDist) continue;
            try {
                BRepExtrema_DistShapeShape dss(faces[i], faces[j]);
                if (dss.IsDone() && dss.NbSolution() > 0) {
                    const double d = dss.Value();
                    if (d > 1e-6 && d < minD) minD = d;
                }
            } catch (...) { /* skip flaky pair */ }
        }
    }
    return minD;
}

// DFM-003 helper: collect cylindrical-face axes, compare pairwise; if axes
// are parallel (|dot| > 0.99) and the perpendicular distance between them
// is < `limit`, return the smallest such distance.  +inf when no holes.
// `minHoleRadius` filters out sub-threshold holes before the pitch check.
// DFM-003 is a STRUCTURAL hole-to-hole rule; dense perforation arrays
// (e.g. a speaker grille's sub-mm holes) are intentionally tight and are
// governed by the web-thickness rule DFM-014, not by structural pitch.
double minHoleToHoleAxisDistance(const TopoDS_Shape& s, double minHoleRadius = 0.0)
{
    struct CylAxis { gp_Ax1 axis; double radius; };
    std::vector<CylAxis> axes;
    axes.reserve(32);
    for (TopExp_Explorer ex(s, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face& f = TopoDS::Face(ex.Current());
        try {
            BRepAdaptor_Surface surf(f);
            if (surf.GetType() != GeomAbs_Cylinder) continue;
            const gp_Cylinder cyl = surf.Cylinder();
            if (cyl.Radius() < minHoleRadius) continue;  // perforation → DFM-014
            axes.push_back({ cyl.Axis(), cyl.Radius() });
        } catch (...) { /* skip */ }
    }
    if (axes.size() < 2) return std::numeric_limits<double>::infinity();

    double minEdgeGap = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < axes.size(); ++i) {
        for (std::size_t j = i + 1; j < axes.size(); ++j) {
            const gp_Dir& di = axes[i].axis.Direction();
            const gp_Dir& dj = axes[j].axis.Direction();
            const double dotAxes = std::abs(
                di.X() * dj.X() + di.Y() * dj.Y() + di.Z() * dj.Z());
            if (dotAxes < 0.99) continue;   // not parallel
            // Distance between two parallel infinite lines.
            const gp_Lin li(axes[i].axis);
            const double centreDist = li.Distance(axes[j].axis.Location());
            // Edge-to-edge clearance = centre-to-centre - both radii.
            const double edgeGap =
                centreDist - axes[i].radius - axes[j].radius;
            if (edgeGap > 0.0 && edgeGap < minEdgeGap) minEdgeGap = edgeGap;
        }
    }
    return minEdgeGap;
}

// DFM-004 helper: walk concave edges of the shape; estimate curvature via
// `GeomLProp_CLProps` at the curve midpoint; return the minimum radius
// among all sampled non-linear edges, or +inf for an all-straight shape.
// "Concave" is approximated by checking adjacent face normals — if the
// edge curls toward the inside (dot of outward normals < 0.95) we treat
// it as a candidate corner edge.
double minConcaveCornerRadius(const TopoDS_Shape& s)
{
    // Build edge → list-of-faces map so we know each edge's neighbours.
    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> e2f;
    TopExp::MapShapesAndAncestors(s, TopAbs_EDGE, TopAbs_FACE, e2f);

    double minR = std::numeric_limits<double>::infinity();
    for (int i = 1; i <= e2f.Extent(); ++i) {
        const TopoDS_Edge& e = TopoDS::Edge(e2f.FindKey(i));
        const NCollection_List<TopoDS_Shape>& nb = e2f.FindFromIndex(i);
        if (nb.Extent() < 2) continue;   // need two adjacent faces
        const TopoDS_Face& fA = TopoDS::Face(nb.First());
        const TopoDS_Face& fB = TopoDS::Face(nb.Last());

        try {
            BRepAdaptor_Curve crv(e);
            // Skip linear edges (infinite radius).
            if (crv.GetType() == GeomAbs_Line) continue;

            // Concavity gate: skip flat edges (G1 continuous) and convex
            // edges that wrap an outer round (handled by DFM-017).
            gp_Dir nA, nB;
            gp_Pnt pA, pB;
            const bool okA = sampleFaceNormalAndPoint(fA, nA, pA);
            const bool okB = sampleFaceNormalAndPoint(fB, nB, pB);
            (void)pA; (void)pB;
            if (okA && okB) {
                const double nDot = nA.X() * nB.X() + nA.Y() * nB.Y()
                                  + nA.Z() * nB.Z();
                if (nDot > 0.95) continue;        // ~near-flat
            }

            // Sample curvature at the curve midpoint.
            const double tMid = (crv.FirstParameter() + crv.LastParameter()) / 2.0;
            GeomLProp_CLProps prop(crv.Curve().Curve(), tMid, 2, 1e-7);
            const double k = prop.Curvature();
            if (k < 1e-9) continue;   // numerically straight
            const double R = 1.0 / k;
            if (R > 0.0 && R < minR) minR = R;
        } catch (...) { /* skip */ }
    }
    return minR;
}

// DFM-011 helper moved to probe::minDihedralAngleDeg (B5.2) — the
// product-agnostic GeometryProbe module carries the verbatim logic.

// DFM-013 helper: find the largest planar face whose normal points -Z
// (downward) — a candidate display pocket floor — then sample a 10×10
// grid of UV points and report the worst point-to-best-fit-plane deviation.
// Returns +inf when no such face exists (shape has no display pocket).
double displayPocketFlatness(const TopoDS_Shape& s)
{
    TopoDS_Face best;
    double      bestArea = -1.0;
    for (TopExp_Explorer ex(s, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face& f = TopoDS::Face(ex.Current());
        try {
            BRepAdaptor_Surface surf(f);
            if (surf.GetType() != GeomAbs_Plane) continue;
            gp_Dir n; gp_Pnt p;
            if (!sampleFaceNormalAndPoint(f, n, p)) continue;
            (void)p;
            // Look for a face whose OUTWARD normal points +Z (the pocket
            // floor's outward normal — we drilled DOWN into the top, so the
            // floor's outward normal is +Z, opposite of cut direction).
            if (n.Z() < 0.9) continue;
            GProp_GProps props;
            BRepGProp::SurfaceProperties(f, props);
            const double area = props.Mass();
            if (area > 100.0 && area > bestArea) { bestArea = area; best = f; }
        } catch (...) {}
    }
    if (best.IsNull()) return std::numeric_limits<double>::infinity();

    // For a true planar face the flatness is 0 by construction.  We still
    // sample a small grid and report the max deviation from the best-fit
    // plane (= surface plane) — guards against degenerate / B-spline floors
    // that happen to be near-planar.
    BRepAdaptor_Surface surf(best);
    if (surf.GetType() != GeomAbs_Plane)
        return std::numeric_limits<double>::infinity();
    const gp_Pln pln  = surf.Plane();
    const gp_Pnt loc  = pln.Location();
    const gp_Dir nrm  = pln.Axis().Direction();

    double maxDev = 0.0;
    const int    N    = 10;
    const double uMin = surf.FirstUParameter();
    const double uMax = surf.LastUParameter();
    const double vMin = surf.FirstVParameter();
    const double vMax = surf.LastVParameter();
    for (int iu = 0; iu < N; ++iu) {
        const double u = uMin + (uMax - uMin) * iu / (N - 1);
        for (int iv = 0; iv < N; ++iv) {
            const double v = vMin + (vMax - vMin) * iv / (N - 1);
            try {
                gp_Pnt q = surf.Value(u, v);
                const gp_Vec dv(loc, q);
                const double signed_d =
                    dv.X() * nrm.X() + dv.Y() * nrm.Y() + dv.Z() * nrm.Z();
                if (std::abs(signed_d) > maxDev) maxDev = std::abs(signed_d);
            } catch (...) {}
        }
    }
    return maxDev;
}

// DFM-017 helper: walk outer-silhouette edges (vertical-tangent faces) and
// check tangent continuity at shared vertices.  Returns the largest tangent
// angle gap observed in degrees, or 0.0 when there are no candidate edges.
double maxOuterTangentGapDeg(const TopoDS_Shape& s)
{
    // Collect candidate edges: those that lie between two faces whose
    // normals are near-horizontal (|n·Z| < 0.2) — i.e., side-wall edges.
    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> e2f;
    TopExp::MapShapesAndAncestors(s, TopAbs_EDGE, TopAbs_FACE, e2f);

    std::vector<TopoDS_Edge> silhouette;
    silhouette.reserve(16);
    for (int i = 1; i <= e2f.Extent(); ++i) {
        const NCollection_List<TopoDS_Shape>& nb = e2f.FindFromIndex(i);
        if (nb.Extent() < 2) continue;
        const TopoDS_Face& fA = TopoDS::Face(nb.First());
        const TopoDS_Face& fB = TopoDS::Face(nb.Last());
        gp_Dir nA, nB; gp_Pnt pA, pB;
        if (!sampleFaceNormalAndPoint(fA, nA, pA)) continue;
        if (!sampleFaceNormalAndPoint(fB, nB, pB)) continue;
        (void)pA; (void)pB;
        if (std::abs(nA.Z()) > 0.2 || std::abs(nB.Z()) > 0.2) continue;
        silhouette.push_back(TopoDS::Edge(e2f.FindKey(i)));
    }
    if (silhouette.size() < 2) return 0.0;

    // For each edge, sample tangent at first and last parameter.  If the
    // edge's end is within 0.05 mm of another edge's start (= shared
    // vertex proxy), measure the tangent-angle gap and remember the worst.
    auto sampleEnd = [](const TopoDS_Edge& e, bool atStart,
                        gp_Pnt& pOut, gp_Vec& tOut) -> bool {
        try {
            BRepAdaptor_Curve c(e);
            const double param = atStart ? c.FirstParameter() : c.LastParameter();
            gp_Vec d1;
            c.D1(param, pOut, d1);
            if (d1.Magnitude() < 1e-9) return false;
            d1.Normalize();
            tOut = d1;
            return true;
        } catch (...) { return false; }
    };

    double maxGap = 0.0;
    for (std::size_t i = 0; i < silhouette.size(); ++i) {
        gp_Pnt pe; gp_Vec tEnd;
        if (!sampleEnd(silhouette[i], false, pe, tEnd)) continue;
        for (std::size_t j = 0; j < silhouette.size(); ++j) {
            if (i == j) continue;
            gp_Pnt ps; gp_Vec tStart;
            if (!sampleEnd(silhouette[j], true, ps, tStart)) continue;
            // Endpoint proximity gate (shared-vertex proxy).
            if (pe.Distance(ps) > 0.05) continue;
            const double dot = std::clamp(tEnd.Dot(tStart), -1.0, 1.0);
            const double angDeg = std::acos(std::abs(dot)) * 180.0 / M_PI;
            if (angDeg > maxGap) maxGap = angDeg;
        }
    }
    return maxGap;
}

}  // namespace

// ── The shared rule catalog ────────────────────────────────────────────────
DFMReport runProductDFM(const TopoDS_Shape& shape,
                        const nlohmann::json& spec,
                        const DFMProfile& profile)
{
    DFMReport report;
    auto addFinding = [&](const char* code, const char* severity,
                          const std::string& msg) {
        report.add(code, severity, msg);
    };

    if (shape.IsNull()) {
        addFinding("DFM-NULL", "error", "shape is null — cannot run DFM");
        return report;
    }

    // ── DFM-002: minimum hole / cavity diameter ≥ 0.8 mm ───────────────
    constexpr double kMinHoleDia = 0.8;
    auto checkHoleDia = [&](double dia, const char* src) {
        if (dia > 0.0 && dia < kMinHoleDia) {
            addFinding("DFM-002", "error",
                std::string(src) + " diameter " + std::to_string(dia) +
                " mm < min " + std::to_string(kMinHoleDia) + " mm");
        }
    };
    if (spec.contains("crown_cavity")) {
        const auto& cc = spec["crown_cavity"];
        checkHoleDia(cc["diameter_mm"].get<double>(),  "crown_cavity.diameter_mm");
        checkHoleDia(cc["shaft_dia_mm"].get<double>(), "crown_cavity.shaft_dia_mm");
    }
    if (spec.contains("speaker_grille")) {
        checkHoleDia(spec["speaker_grille"]["hole_dia_mm"].get<double>(),
                     "speaker_grille.hole_dia_mm");
    }
    if (spec.contains("rear_sensors")) {
        std::size_t i = 0;
        for (const auto& s : spec["rear_sensors"]) {
            checkHoleDia(s["dia_mm"].get<double>(),
                         ("rear_sensors[" + std::to_string(i) + "].dia_mm").c_str());
            ++i;
        }
    }
    if (spec.contains("lugs")) {
        std::size_t i = 0;
        for (const auto& l : spec["lugs"]) {
            checkHoleDia(l.value("pin_hole_dia_mm", 0.0),
                         ("lugs[" + std::to_string(i) + "].pin_hole_dia_mm").c_str());
            ++i;
        }
    }

    // ── DFM-009: bezel minimum width ≥ 0.6 mm ─────────────────────────
    constexpr double kMinBezelW = 0.6;
    if (spec.contains("bezel")) {
        const double w = spec["bezel"]["width_mm"].get<double>();
        if (w < kMinBezelW) {
            addFinding("DFM-009", "error",
                "bezel.width_mm " + std::to_string(w) +
                " < min " + std::to_string(kMinBezelW));
        }
    }

    // ── DFM-014: speaker-grille pin (web between holes) ≥ 0.25 mm ─────
    constexpr double kMinPinThick = 0.25;
    if (spec.contains("speaker_grille")) {
        const auto& sg = spec["speaker_grille"];
        const double colSp = sg["col_spacing_mm"].get<double>();
        const double rowSp = sg["row_spacing_mm"].get<double>();
        const double dia   = sg["hole_dia_mm"].get<double>();
        const double pinT  = std::min(colSp, rowSp) - dia;
        if (pinT < kMinPinThick) {
            addFinding("DFM-014", "error",
                "speaker_grille pin thickness " + std::to_string(pinT) +
                " mm < min " + std::to_string(kMinPinThick) + " mm");
        }
    }

    // ── DFM-020: all shells closed (topology check) ────────────────────
    {
        BRepCheck_Analyzer analyzer(shape);
        if (!analyzer.IsValid()) {
            addFinding("DFM-020", "warning",
                "BRepCheck_Analyzer reports invalid topology — likely "
                "non-closed shell or geometry tolerance issue");
        }
    }

    // ── DFM-022: OBB Z aligned (thickness axis is the smallest extent) ─
    {
        const auto bb = pr::optimalBbox(shape);
        const double dx = bb.dx(), dy = bb.dy(), dz = bb.dz();
        const double minExt = std::min({ dx, dy, dz });
        if (std::abs(dz - minExt) > 1e-3) {
            addFinding("DFM-022", "warning",
                "thickness axis dz=" + std::to_string(dz) +
                " mm is not the smallest extent (min=" +
                std::to_string(minExt) + " mm)");
        }
    }

    // ══════════════════════════════════════════════════════════════════
    //  Newly added rules (10/25) — see [[engine/dfm-rules]] for criteria.
    // ══════════════════════════════════════════════════════════════════

    // ── DFM-001: minimum wall thickness (profile: watch 0.35 / phone 0.40) ─
    {
        const double minW = minWallThickness(shape);
        if (std::isfinite(minW) && minW < profile.minWallMm) {
            addFinding("DFM-001", "error",
                "minimum wall thickness " + std::to_string(minW) +
                " mm < " + profile.product + " limit " +
                std::to_string(profile.minWallMm) + " mm");
        }
    }

    // ── DFM-003: minimum STRUCTURAL hole-to-hole edge distance ≥ 1.5 mm ─
    // Only holes ≥ 1.0 mm Ø participate: sub-mm perforation arrays (speaker
    // grille) are intentionally tight and are governed by DFM-014 web
    // thickness, not by the structural hole-pitch rule.
    {
        constexpr double kMinHoleGap        = 1.5;
        constexpr double kStructuralHoleRad = 0.5;   // Ø1.0 mm threshold
        const double gap = minHoleToHoleAxisDistance(shape, kStructuralHoleRad);
        if (std::isfinite(gap) && gap < kMinHoleGap) {
            addFinding("DFM-003", "error",
                "hole-to-hole gap " + std::to_string(gap) +
                " mm < min " + std::to_string(kMinHoleGap) + " mm");
        }
    }

    // ── DFM-004: minimum concave corner radius ≥ 0.2 mm ────────────────
    {
        constexpr double kMinCornerR = 0.2;
        const double r = minConcaveCornerRadius(shape);
        if (std::isfinite(r) && r < kMinCornerR) {
            addFinding("DFM-004", "error",
                "minimum concave corner radius " + std::to_string(r) +
                " mm < min " + std::to_string(kMinCornerR) + " mm");
        }
    }

    // ── DFM-006: button pocket depth ≤ min(0.8, thickness − 0.3) ───────
    if (spec.contains("side_buttons") && !spec["side_buttons"].empty() &&
        spec.contains("base") && spec["base"].contains("thickness_mm")) {
        const double thickness = spec["base"]["thickness_mm"].get<double>();
        const double limit     = std::min(0.8, thickness - 0.3);
        std::size_t idx = 0;
        for (const auto& btn : spec["side_buttons"]) {
            const double depth = btn.value("depth_mm", 0.0);
            if (depth > limit) {
                addFinding("DFM-006", "error",
                    "side_buttons[" + std::to_string(idx) + "].depth_mm " +
                    std::to_string(depth) + " > limit " + std::to_string(limit) +
                    " (= min(0.8, thickness-0.3))");
            }
            ++idx;
        }
    }

    // ── DFM-011: anti-knife edge — adjacent face wedge ≥ 5° ────────────
    {
        constexpr double kMinDihedral = 5.0;
        const double ang = probe::minDihedralAngleDeg(shape);
        if (std::isfinite(ang) && ang < kMinDihedral) {
            addFinding("DFM-011", "error",
                "minimum adjacent-face wedge angle " + std::to_string(ang) +
                " deg < min " + std::to_string(kMinDihedral) + " deg");
        }
    }

    // ── DFM-013: display pocket flatness ≤ 0.02 mm  (phone scope) ──────
    // Per spec, DFM-013 is Phone-only — gated by the product profile.
    if (profile.displayPocketFlatnessRule) {
        constexpr double kMaxFlatness = 0.02;
        const double f = displayPocketFlatness(shape);
        if (std::isfinite(f) && f > kMaxFlatness) {
            addFinding("DFM-013", "error",
                "display pocket flatness " + std::to_string(f) +
                " mm > max " + std::to_string(kMaxFlatness) + " mm");
        }
    }

    // ── DFM-015: TAPPED-hole min surrounding material ≥ tap-dia × 1.5 ──
    // Applies only to THREADED (tapped) holes — a tap's thread root needs
    // generous surrounding stock to avoid blow-out.  A watch strap lug's
    // spring-bar hole is a CLEARANCE hole, not a tap, so the 1.5× rule does
    // not apply: real lug tips legitimately have ~1 pin-dia of edge stock.
    // We therefore check a lug pin only when it is explicitly flagged
    // `"pin_is_tapped": true` (default false → skipped).
    if (spec.contains("lugs")) {
        std::size_t idx = 0;
        for (const auto& l : spec["lugs"]) {
            const double pinDia    = l.value("pin_hole_dia_mm", 0.0);
            const double length    = l.value("length_mm",       0.0);
            const bool   isTapped  = l.value("pin_is_tapped",   false);
            if (isTapped && pinDia > 0.0 && pinDia <= 3.0 && length > 0.0) {
                // Pin lies at 0.7 × length from lug root (see addLugs).
                const double tipMaterial = length - 0.7 * length;
                const double rootMaterial = 0.7 * length;
                const double minSurround  = pinDia * 1.5;
                const double worst =
                    std::min(tipMaterial, rootMaterial);
                if (worst < minSurround) {
                    addFinding("DFM-015", "error",
                        "lugs[" + std::to_string(idx) + "] pin surrounding " +
                        "material " + std::to_string(worst) +
                        " mm < required " + std::to_string(minSurround) +
                        " mm (= " + std::to_string(pinDia) + " × 1.5)");
                }
            }
            ++idx;
        }
    }

    // ── DFM-017: outer-edge curvature continuity (G1, > 0.5° gap) ──────
    {
        constexpr double kMaxTangentGapDeg = 0.5;
        const double gap = maxOuterTangentGapDeg(shape);
        if (gap > kMaxTangentGapDeg) {
            addFinding("DFM-017", "warning",
                "outer silhouette tangent gap " + std::to_string(gap) +
                " deg > G1 limit " + std::to_string(kMaxTangentGapDeg) + " deg");
        }
    }

    // ── DFM-018: camera deco-ring annulus validity (Phone only) ────────
    // Now that PhoneFrontModel models spec["camera_deco_rings"], validate each
    // ring: it must be a real annulus (inner < outer) whose remaining annular
    // land is at least the profile's minimum wall — a thinner land tears during
    // anodising/machining.  Watch profiles leave decoRingRule false → N/A.
    if (profile.decoRingRule && spec.contains("camera_deco_rings") &&
        spec["camera_deco_rings"].is_array()) {
        const auto& rings = spec["camera_deco_rings"];
        for (std::size_t i = 0; i < rings.size(); ++i) {
            const auto&  r     = rings[i];
            const double outer = r.value("outer_dia_mm", 0.0);
            const double inner = r.value("inner_dia_mm", 0.0);
            const std::string at = "camera_deco_rings[" + std::to_string(i) + "]";
            if (inner >= outer) {
                addFinding("DFM-018", "error",
                    at + " inner_dia " + std::to_string(inner) +
                    " >= outer_dia " + std::to_string(outer) + " mm (not an annulus)");
                continue;
            }
            const double land = (outer - inner) / 2.0;
            if (land < profile.minWallMm) {
                addFinding("DFM-018", "warning",
                    at + " annular land " + std::to_string(land) +
                    " mm < min wall " + std::to_string(profile.minWallMm) + " mm");
            }
        }
    }

    // ── DFM-023: watch lug span symmetry ≤ 0.05 mm ─────────────────────
    if (spec.contains("lugs") && spec["lugs"].is_array() &&
        spec["lugs"].size() >= 2) {
        const auto& a = spec["lugs"][0];
        const auto& b = spec["lugs"][1];
        auto checkSpan = [&](const char* field) {
            const double va = a.value(field, 0.0);
            const double vb = b.value(field, 0.0);
            const double diff = std::abs(va - vb);
            if (diff > 0.05) {
                addFinding("DFM-023", "warning",
                    std::string("lug span asymmetry on '") + field +
                    "': |" + std::to_string(va) + " - " +
                    std::to_string(vb) + "| = " + std::to_string(diff) +
                    " mm > 0.05 mm");
            }
        };
        checkSpan("length_mm");
        checkSpan("width_mm");
        checkSpan("thickness_mm");
    }

    // ══════════════════════════════════════════════════════════════════
    //  M1.5 expansion: bbox / topology summary rules (see [[engine/dfm-rules]])
    // ══════════════════════════════════════════════════════════════════

    // ── DFM-005: max bbox aspect ratio ≤ 10 (fixture-impossible parts) ─
    try {
        const auto bb = pr::optimalBbox(shape);
        const double dx = bb.dx(), dy = bb.dy(), dz = bb.dz();
        const double maxExt = std::max({ dx, dy, dz });
        const double minExt = std::min({ dx, dy, dz });
        if (minExt > 1e-9) {
            const double aspect = maxExt / minExt;
            if (aspect > 10.0) {
                addFinding("DFM-005", "warning",
                    "bbox aspect ratio " + std::to_string(aspect) +
                    " > 10 (max/min = " + std::to_string(maxExt) + "/" +
                    std::to_string(minExt) + ") — fixture-impossible");
            } else {
                addFinding("DFM-005", "info",
                    "bbox aspect ratio " + std::to_string(aspect) +
                    " within limit (10)");
            }
        }
    } catch (...) { /* skip on bbox failure */ }

    // ── DFM-007: no planar face larger than the part's bounding footprint ─
    // Purpose: flag a degenerate boolean that leaves a runaway planar patch.
    // The old "5 × min-extent²" limit was WRONG for thin plate/disk parts —
    // a 44 mm watch's flat circular face (~1417 mm²) is correct geometry yet
    // dwarfs 5×(10 mm)²=500 mm², so the rule rejected every legitimate watch.
    // A genuine *planar* face cannot exceed the largest bbox rectangle, so we
    // bound max planar-face area by that footprint (×1.02 margin).  This is a
    // suspicion signal (warning); hard broken-boolean detection lives in
    // DFM-016 (fill ratio) / DFM-020 (validity) / DFM-021 (shell count).
    try {
        const auto bb = pr::optimalBbox(shape);
        const double footprint = std::max({ bb.dx() * bb.dy(),
                                            bb.dy() * bb.dz(),
                                            bb.dx() * bb.dz() });
        const double areaLimit = 1.02 * footprint;
        double maxArea = 0.0;
        for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
            const TopoDS_Face& f = TopoDS::Face(ex.Current());
            try {
                if (BRepAdaptor_Surface(f).GetType() != GeomAbs_Plane) continue;
                GProp_GProps props;
                BRepGProp::SurfaceProperties(f, props);
                const double a = props.Mass();
                if (a > maxArea) maxArea = a;
            } catch (...) { /* skip degenerate face */ }
        }
        if (maxArea > areaLimit && areaLimit > 0.0) {
            addFinding("DFM-007", "warning",
                "max planar face area " + std::to_string(maxArea) +
                " mm² > footprint " + std::to_string(areaLimit) +
                " mm² — possible degenerate boolean (verify)");
        }
    } catch (...) { /* skip */ }

    // ── DFM-012: face count ≤ 200 (info only; high counts are suspicious) ─
    try {
        std::size_t faceCount = 0;
        for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next())
            ++faceCount;
        addFinding("DFM-012", "info",
            "face count = " + std::to_string(faceCount) +
            (faceCount > 200
                ? " (> 200 — suspicious for a watch frame)"
                : " (within typical range)"));
    } catch (...) { /* skip */ }

    // ── DFM-016: volume fill ratio (excessively solid → wrong file?) ───
    try {
        const auto bb = pr::optimalBbox(shape);
        const double bboxVol = bb.dx() * bb.dy() * bb.dz();
        GProp_GProps vp;
        BRepGProp::VolumeProperties(shape, vp);
        const double solidVol = vp.Mass();
        if (bboxVol > 1e-9) {
            const double ratio = solidVol / bboxVol;
            if (ratio > 0.95) {
                addFinding("DFM-016", "warning",
                    "solid volume / bbox volume = " + std::to_string(ratio) +
                    " > 0.95 — insufficiently hollowed (wrong file?)");
            } else if (ratio < 0.05) {
                addFinding("DFM-016", "warning",
                    "solid volume / bbox volume = " + std::to_string(ratio) +
                    " < 0.05 — excessively hollow (CAD error?)");
            } else {
                addFinding("DFM-016", "info",
                    "volume fill ratio = " + std::to_string(ratio));
            }
        }
    } catch (...) { /* skip */ }

    // ── DFM-021: distinct closed shells > 1 (unintended compound) ──────
    try {
        std::size_t shellCount = 0;
        for (TopExp_Explorer ex(shape, TopAbs_SHELL); ex.More(); ex.Next())
            ++shellCount;
        if (shellCount > 1) {
            addFinding("DFM-021", "warning",
                "distinct closed shells = " + std::to_string(shellCount) +
                " — unintended compound (expected 1)");
        }
    } catch (...) { /* skip */ }

    return report;
}

DFMProfile watchProfile()
{
    DFMProfile p;
    p.product   = "watch";
    p.minWallMm = 0.35;
    p.displayPocketFlatnessRule = false;
    p.decoRingRule              = false;
    return p;
}

DFMProfile phoneProfile()
{
    DFMProfile p;
    p.product   = "phone";
    p.minWallMm = 0.40;
    p.displayPocketFlatnessRule = true;
    p.decoRingRule              = true;
    return p;
}

}  // namespace koocadcam::engine::dfm
