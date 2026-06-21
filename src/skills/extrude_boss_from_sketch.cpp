// @lat: [[engine/skills#extrude_boss_from_sketch]]

#include "extrude_boss_from_sketch.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_CurveType.hxx>
#include <Geom_Circle.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::extrude_boss_from_sketch {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Shoelace polygon area (signed).
double polygonAreaXY(const std::vector<std::pair<double,double>>& poly)
{
    if (poly.size() < 3) return 0.0;
    double a = 0.0;
    const int n = static_cast<int>(poly.size());
    for (int i = 0; i < n; ++i) {
        const auto& A = poly[i];
        const auto& B = poly[(i + 1) % n];
        a += A.first * B.second - B.first * A.second;
    }
    return std::abs(a) * 0.5;
}

using ProfileSeg = extrude_boss_from_sketch::ProfileSeg;

// Promote a legacy straight-line polygon to a Line-segment profile (each vertex
// becomes the END point of a Line whose START is the previous vertex).
std::vector<ProfileSeg> polygonToProfile(
    const std::vector<std::pair<double,double>>& poly)
{
    std::vector<ProfileSeg> prof;
    prof.reserve(poly.size());
    for (const auto& p : poly) {
        ProfileSeg s;
        s.kind = ProfileSeg::Kind::Line;
        s.x = p.first; s.y = p.second;
        prof.push_back(s);
    }
    return prof;
}

// Area enclosed by a closed line|arc profile: the end-point polygon area
// (shoelace) plus each arc's circular-segment area (the sliver between the
// chord and the arc), signed by whether the arc bulges OUT of or INTO the
// polygon.  Exact for the volume specificity gate.
double profileArea(const std::vector<ProfileSeg>& prof)
{
    const std::size_t n = prof.size();
    if (n == 0) return 0.0;
    // An all-line profile needs >= 3 vertices to enclose area; a profile with
    // arcs can enclose area with as few as 1 segment (a full circle), so only
    // bail early when there are no arcs AND too few vertices.
    const bool anyArc = std::any_of(prof.begin(), prof.end(),
        [](const ProfileSeg& s) { return s.kind == ProfileSeg::Kind::Arc; });
    if (!anyArc && n < 3) return 0.0;

    // 1) Shoelace over the end points (the chord polygon).
    double a = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const auto& A = prof[i];
        const auto& B = prof[(i + 1) % n];
        a += A.x * B.y - B.x * A.y;
    }
    const double polyArea = a;            // signed (>0 if CCW)
    const double polySign  = polyArea >= 0.0 ? 1.0 : -1.0;

    // 2) Add each arc's circular-segment sliver.
    double slivers = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const ProfileSeg& s = prof[i];
        if (s.kind != ProfileSeg::Kind::Arc) continue;
        const ProfileSeg& prev = prof[(i + n - 1) % n];   // start = previous end
        const double sx = prev.x, sy = prev.y, ex = s.x, ey = s.y;
        const double r  = std::hypot(sx - s.cx, sy - s.cy);
        if (r < 1e-9) continue;
        // Swept angle start->end in the arc's sweep direction.
        double a0 = std::atan2(sy - s.cy, sx - s.cx);
        double a1 = std::atan2(ey - s.cy, ex - s.cx);
        double sweep = a1 - a0;
        if (s.ccw) { while (sweep <= 0) sweep += 2.0 * M_PI; }
        else       { while (sweep >= 0) sweep -= 2.0 * M_PI; sweep = -sweep; }
        // Circular-segment area for the (positive) swept angle.
        const double segArea = 0.5 * r * r * (sweep - std::sin(sweep));
        // A CCW arc bulges to the LEFT of start->end; whether that adds to or
        // subtracts from the chord polygon depends on the polygon orientation.
        const double bulgeSign = (s.ccw ? 1.0 : -1.0) * polySign;
        slivers += bulgeSign * segArea;
    }
    return std::abs(0.5 * polyArea + slivers);
}

// Smallest edge length in the polygon (min wall heuristic).
double minEdgeLengthXY(const std::vector<std::pair<double,double>>& poly)
{
    if (poly.size() < 2) return 0.0;
    double m = std::numeric_limits<double>::infinity();
    const int n = static_cast<int>(poly.size());
    for (int i = 0; i < n; ++i) {
        const auto& A = poly[i];
        const auto& B = poly[(i + 1) % n];
        const double dx = B.first - A.first;
        const double dy = B.second - A.second;
        const double d = std::sqrt(dx * dx + dy * dy);
        if (d < m) m = d;
    }
    return std::isfinite(m) ? m : 0.0;
}

// Construct a sketch wire on the plane defined by (origin, normal, xDir).
// `prof` is a closed line|arc profile in the local (xDir, yDir = normal × xDir)
// frame.  Each segment's START is the previous segment's END; the loop closes
// from the last END back to the first.
TopoDS_Wire makeSketchWire(const std::vector<ProfileSeg>& prof,
                           const gp_Pnt& origin,
                           const gp_Dir& normal,
                           const gp_Dir& xDir)
{
    const gp_Vec vx(xDir);
    gp_Vec vyTmp = gp_Vec(normal).Crossed(vx);
    if (vyTmp.Magnitude() < 1e-9)
        throw SkillError("extrude_boss_from_sketch: degenerate frame (normal || xDir)");
    vyTmp.Normalize();

    // Lift a face-local (u,v) point into world coords.
    auto lift = [&](double u, double v) {
        return gp_Pnt(origin.X() + u * vx.X() + v * vyTmp.X(),
                      origin.Y() + u * vx.Y() + v * vyTmp.Y(),
                      origin.Z() + u * vx.Z() + v * vyTmp.Z());
    };

    const std::size_t n = prof.size();
    BRepBuilderAPI_MakeWire wireMk;
    for (std::size_t i = 0; i < n; ++i) {
        const ProfileSeg& seg  = prof[i];
        const ProfileSeg& prev = prof[(i + n - 1) % n];   // start = previous end
        const gp_Pnt a = lift(prev.x, prev.y);
        const gp_Pnt b = lift(seg.x,  seg.y);
        if (a.Distance(b) < 1e-9 && seg.kind == ProfileSeg::Kind::Line) continue;

        if (seg.kind == ProfileSeg::Kind::Arc) {
            // Circle in the face plane: centre lifted, axis = face normal.
            const gp_Pnt   c   = lift(seg.cx, seg.cy);
            const double   r   = std::hypot(prev.x - seg.cx, prev.y - seg.cy);
            // Axis sign sets the parametrisation direction; flip to honour ccw.
            // (Geom_Circle param = angle CCW about its axis.  For a CW sweep we
            //  reverse the built edge — see _separation_common.hpp.)
            const gp_Ax2 ax(c, normal, xDir);
            Handle(Geom_Circle) circ = new Geom_Circle(ax, r);
            // Parameters of a, b on this circle (CCW angle about `normal`).
            const gp_Vec ca(c, a), cb(c, b);
            const double t0 = std::atan2(ca.Dot(vyTmp), ca.Dot(vx));
            const double t1 = std::atan2(cb.Dot(vyTmp), cb.Dot(vx));
            double p0 = t0, p1 = t1;
            if (p1 <= p0) p1 += 2.0 * M_PI;   // MakeEdge needs p0 < p1 (CCW)
            BRepBuilderAPI_MakeEdge em(circ, p0, p1);
            if (!em.IsDone())
                throw SkillError("extrude_boss_from_sketch: arc edge build failed");
            // The CCW edge runs a->b along +angle.  If the sketch arc sweeps
            // CW, reverse it so the wire stays continuous.
            wireMk.Add(seg.ccw ? em.Edge() : TopoDS::Edge(em.Edge().Reversed()));
        } else {
            BRepBuilderAPI_MakeEdge em(a, b);
            if (!em.IsDone())
                throw SkillError("extrude_boss_from_sketch: edge build failed");
            wireMk.Add(em.Edge());
        }
    }
    if (!wireMk.IsDone())
        throw SkillError("extrude_boss_from_sketch: wire build failed");
    return wireMk.Wire();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    // The working profile: the richer `profile` if supplied, else the legacy
    // straight-line `polygon` promoted to Line segments.  A closed arc/circle
    // profile can be as few as 1 segment (a full circle is one arc edge), so
    // the vertex-count rule only applies to all-line profiles.
    const std::vector<ProfileSeg> prof =
        in.profile.empty() ? polygonToProfile(in.polygon) : in.profile;
    const bool allLine = std::all_of(prof.begin(), prof.end(),
        [](const ProfileSeg& s) { return s.kind == ProfileSeg::Kind::Line; });

    if (prof.empty() || (allLine && prof.size() < 3)) {
        r.add("DFM-INPUT", "error",
              "extrude_boss_from_sketch: profile needs >= 3 line vertices (or "
              "an arc/circle), got " + std::to_string(prof.size()) + " segments");
    }
    if (in.height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "extrude_boss_from_sketch: height_mm must be > 0");
    }
    if (in.draft_angle_deg < 0.0 || in.draft_angle_deg > 30.0) {
        r.add("DFM-DRAFT", "error",
              "extrude_boss_from_sketch: draft_angle_deg " +
              std::to_string(in.draft_angle_deg) +
              " out of [0, 30] range");
    }
    // Min-wall heuristic only meaningful for straight polygons.
    if (allLine && in.polygon.size() >= 2) {
        const double minEdge = minEdgeLengthXY(in.polygon);
        if (minEdge > 0.0 && minEdge < 0.4) {
            r.add("DFM-001", "error",
                  "extrude_boss_from_sketch: min polygon edge " +
                  std::to_string(minEdge) +
                  " mm < 0.4 mm (min wall thickness)");
        }
    }
    if (!prof.empty() && profileArea(prof) < 1e-6) {
        r.add("DFM-INPUT", "error",
              "extrude_boss_from_sketch: profile area is ~0 (collinear?)");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "extrude_boss_from_sketch DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Resolve entry face -> get origin + outward normal.
    auto faceIdOpt = wp.resolve(in.entry_face);
    if (!faceIdOpt)
        throw SkillError("extrude_boss_from_sketch: entry_face datum unresolved");
    const int faceId = *faceIdOpt;
    const gp_Pnt origin = wp.faceCenter(faceId);
    const gp_Dir normal = wp.faceNormal(faceId);

    // Pick an X-direction in the face plane: anything orthogonal to the normal.
    gp_Dir xDir = gp::DX();
    if (std::abs(gp_Vec(normal).Dot(gp_Vec(xDir))) > 0.99)
        xDir = gp::DY();   // fall back if normal ≈ ±X
    // Re-orthogonalize: project xDir into face plane.
    gp_Vec vx(xDir);
    vx = vx - gp_Vec(normal) * vx.Dot(gp_Vec(normal));
    if (vx.Magnitude() < 1e-9)
        throw SkillError("extrude_boss_from_sketch: cannot build face X-axis");
    vx.Normalize();
    const gp_Dir xDirFinal(vx);

    // Resolve the working profile: the richer `profile` if supplied, else the
    // legacy straight-line `polygon` promoted to Line segments.
    const std::vector<ProfileSeg> prof =
        in.profile.empty() ? polygonToProfile(in.polygon) : in.profile;

    // Build wire on face plane.
    const TopoDS_Wire wire = makeSketchWire(prof, origin, normal, xDirFinal);

    BRepBuilderAPI_MakeFace faceMk(wire, true);
    if (!faceMk.IsDone())
        throw SkillError("extrude_boss_from_sketch: face build failed");

    // Extrude along outward normal by height.
    const gp_Vec extrudeVec(normal.X() * in.height_mm,
                            normal.Y() * in.height_mm,
                            normal.Z() * in.height_mm);
    BRepPrimAPI_MakePrism prism(faceMk.Face(), extrudeVec);
    prism.Build();
    if (!prism.IsDone())
        throw SkillError("extrude_boss_from_sketch: prism build failed");
    const TopoDS_Shape boss = prism.Shape();

    // Fuse boss onto workpiece.
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), boss);

    const double area  = profileArea(prof);
    const double vol   = area * in.height_mm;

    // ── signature ────────────────────────────────────────────────────────
    // Emit the richer `profile` always (line|arc), and `polygon` too when the
    // profile is all straight (so legacy consumers and the metadata replay path
    // keep working unchanged).
    const bool allLine = std::all_of(prof.begin(), prof.end(),
        [](const ProfileSeg& s) { return s.kind == ProfileSeg::Kind::Line; });
    json profJson = json::array();
    for (const auto& s : prof) {
        json e = { { "kind", s.kind == ProfileSeg::Kind::Arc ? "arc" : "line" },
                   { "x", s.x }, { "y", s.y } };
        if (s.kind == ProfileSeg::Kind::Arc) {
            e["cx"] = s.cx; e["cy"] = s.cy; e["ccw"] = s.ccw;
        }
        profJson.push_back(e);
    }
    json polyJson = json::array();
    if (allLine)
        for (const auto& s : prof)
            polyJson.push_back({ { "x", s.x }, { "y", s.y } });

    json params = {
        { "entry_face_id",       faceId },
        { "profile",             profJson },
        { "height_mm",           in.height_mm },
        { "draft_angle_deg",     in.draft_angle_deg },
    };
    if (allLine) params["polygon"] = polyJson;
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    1 },     // sketch face + prism fuse
        { "sketch_vertex_count", static_cast<int>(prof.size()) },
        { "height_mm",           in.height_mm },
        { "draft_angle_deg",     in.draft_angle_deg },
        { "derived_volume_mm3",  vol },
        { "face_normal",         { normal.X(), normal.Y(), normal.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "form_die;prism_add";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.height_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = -vol;   // material ADDED (negative removed)
    tooling.est_cycle_time_s  = std::max(2.0,
        static_cast<double>(in.polygon.size()) * 0.2 + in.height_mm * 0.1);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::extrude_boss_from_sketch applied: n={} h={} vol={} "
                  "faces {}->{}",
                  in.polygon.size(), in.height_mm, vol,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // metadata fallback
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        // "metadata_replay" is the cap-exempt source (Recognizer analyze():
        // a same-session replay keeps full confidence); "feature_history" was
        // NOT exempt, so the round-trip candidate was silently capped to 0.5
        // and dropped below the 0.7 threshold.
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric (path B): recognise an extrusion in FOREIGN geometry (no
    // feature history).  An extrusion has two CONGRUENT, ANTI-PARALLEL planar
    // cap faces separated by the extrude height, joined by side walls.  Recover
    // the profile from one cap's outer wire and the height from the cap
    // separation, so an imported STEP extrusion round-trips like a replayed one.
    struct PlanarFace { int id; gp_Dir n; gp_Pnt c; double area; };
    std::vector<PlanarFace> planar;
    int verticalPlanar = 0;
    int curvedFaces    = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) { ++curvedFaces; continue; }
        try {
            const gp_Dir n = wp.faceNormal(i);
            planar.push_back({ i, n, wp.faceCenter(i), wp.faceArea(i) });
            if (std::abs(n.Z()) < 0.05) ++verticalPlanar;
        } catch (...) { continue; }
    }
    // A pristine rectangular box has exactly 4 vertical planar faces, all faces
    // planar, and is NOT a boss (project convention: bare stock recognises as
    // nothing).  Reject that specific shape — but a CURVED-walled prism (a
    // cylinder / D-boss) has curved side faces, so its few-or-zero vertical
    // planar walls must NOT disqualify it; the cap-pair + volume gates vet it.
    if (curvedFaces == 0 && verticalPlanar <= 4) return out;

    // Best congruent anti-parallel cap pair (largest matching area).
    int capA = -1, capB = -1;
    double bestArea = 0.0, height = 0.0;
    for (std::size_t i = 0; i < planar.size(); ++i) {
        for (std::size_t j = i + 1; j < planar.size(); ++j) {
            if (gp_Vec(planar[i].n).Dot(gp_Vec(planar[j].n)) > -0.98) continue;  // anti-parallel
            const double r = planar[i].area / std::max(1e-9, planar[j].area);
            if (r < 0.97 || r > 1.03) continue;                                   // congruent
            const double d = std::abs(gp_Vec(planar[i].c, planar[j].c).Dot(gp_Vec(planar[i].n)));
            if (d < 0.2) continue;                                                // not coincident
            if (planar[i].area > bestArea) {
                bestArea = planar[i].area; capA = planar[i].id; capB = planar[j].id; height = d;
            }
        }
    }
    if (capA < 0) return out;

    // Recover capA's outer-wire profile in capA's local frame — the SAME frame
    // apply() uses (origin=center, normal, xDir=orthogonalised global X), so the
    // recovered (u,v) regenerate the identical profile.
    const gp_Pnt origin = wp.faceCenter(capA);
    const gp_Dir normal = wp.faceNormal(capA);
    gp_Vec vx(gp::DX());
    if (std::abs(vx.Dot(gp_Vec(normal))) > 0.99) vx = gp_Vec(gp::DY());
    vx = vx - gp_Vec(normal) * vx.Dot(gp_Vec(normal));
    if (vx.Magnitude() < 1e-9) return out;
    vx.Normalize();
    const gp_Vec vy = gp_Vec(normal).Crossed(vx);

    // Walk capA's outer wire EDGE-by-edge (not just vertices) so a circular /
    // arc boundary is recovered as an Arc segment, not flattened to a chord.
    // Each segment stores its END point; an arc also stores centre + sweep dir.
    std::vector<ProfileSeg> prof;
    bool anyArc = false;
    try {
        const TopoDS_Wire ow = BRepTools::OuterWire(wp.face(capA));
        for (BRepTools_WireExplorer wexp(ow); wexp.More(); wexp.Next()) {
            const TopoDS_Edge   e   = wexp.Current();
            const gp_Pnt        pv  = BRep_Tool::Pnt(wexp.CurrentVertex());   // edge start vertex
            // End point = the NEXT vertex along the wire.  WireExplorer gives
            // the start vertex of the current edge; the end is the start of the
            // next, which equals this edge's far endpoint — recover it from the
            // edge curve's last parameter to be robust.
            BRepAdaptor_Curve crv(e);
            const gp_Pnt pEnd = crv.Value(crv.LastParameter());
            const gp_Pnt pBeg = crv.Value(crv.FirstParameter());
            // WireExplorer orients edges along the wire; the segment's END is
            // whichever curve endpoint is farther from the start vertex pv.
            const gp_Pnt end = (pEnd.Distance(pv) >= pBeg.Distance(pv)) ? pEnd : pBeg;
            const gp_Vec relEnd(origin, end);
            ProfileSeg seg;
            seg.x = relEnd.Dot(vx);
            seg.y = relEnd.Dot(vy);
            if (crv.GetType() == GeomAbs_Circle) {
                const gp_Circ c = crv.Circle();
                const gp_Vec relC(origin, c.Location());
                seg.kind = ProfileSeg::Kind::Arc;
                seg.cx   = relC.Dot(vx);
                seg.cy   = relC.Dot(vy);
                // ccw if the circle axis points the same way as the face normal
                // (Geom_Circle param increases CCW about its own axis).
                seg.ccw  = c.Axis().Direction().Dot(normal) >= 0.0;
                anyArc   = true;
            } else if (crv.GetType() != GeomAbs_Line) {
                return out;   // ellipse / spline boundary — out of scope, bail
            }
            prof.push_back(seg);
        }
    } catch (...) { return out; }
    if (prof.size() < 3 && !anyArc) return out;   // a full-circle boss can be 1-2 edges
    if (prof.empty()) return out;

    // SPECIFICITY gate 1 (topology, upper bound): a straight-walled N-gon prism
    // has EXACTLY N side walls + 2 caps; an N-segment line|arc profile has at
    // most #segments side walls (OCCT may MERGE adjacent co-circular arcs into a
    // single cylindrical face, so the count can be LOWER — never higher for a
    // pure prism).  A pocketed/drilled block has EXTRA feature faces, exceeding
    // the bound.  Hence `<=` here, with the volume gate below as the real check.
    if (wp.faceCount() > static_cast<int>(prof.size()) + 2) return out;

    // SPECIFICITY gate 2 (volume, primary): a TRUE prism has
    // volume == profile_area * height, where profile_area is arc-corrected.
    const double profArea    = profileArea(prof);
    const double expectedVol = profArea * height;
    double actualVol = 0.0;
    try {
        GProp_GProps gp;
        BRepGProp::VolumeProperties(wp.shape(), gp);
        actualVol = gp.Mass();
    } catch (...) { return out; }
    if (actualVol <= 1e-6 ||
        std::abs(expectedVol - actualVol) > 0.03 * actualVol)
        return out;   // not a pure prism (material added or removed)

    // Emit the richer `profile`; also a `polygon` when all-line so the metadata
    // replay path and legacy consumers keep working.
    const bool allLine = !anyArc;
    json profJson = json::array();
    for (const auto& s : prof) {
        json e = { { "kind", s.kind == ProfileSeg::Kind::Arc ? "arc" : "line" },
                   { "x", s.x }, { "y", s.y } };
        if (s.kind == ProfileSeg::Kind::Arc) {
            e["cx"] = s.cx; e["cy"] = s.cy; e["ccw"] = s.ccw;
        }
        profJson.push_back(e);
    }
    json recovered = {
        { "entry_face_id",   capA },
        { "profile",         profJson },
        { "height_mm",       height },
        { "draft_angle_deg", 0.0 },
    };
    if (allLine) {
        json polyJson = json::array();
        for (const auto& s : prof)
            polyJson.push_back({ { "x", s.x }, { "y", s.y } });
        recovered["polygon"] = polyJson;
    }
    json matched = {
        { "source",        "geometry" },
        { "cap_face_a",    capA },
        { "cap_face_b",    capB },
        { "segment_count", static_cast<int>(prof.size()) },
        { "has_arc",       anyArc },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.9, matched });
    return out;
}

}  // namespace koocadcam::skill::extrude_boss_from_sketch
