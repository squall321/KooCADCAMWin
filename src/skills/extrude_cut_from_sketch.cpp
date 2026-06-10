// @lat: [[engine/skills#extrude_cut_from_sketch]]

#include "extrude_cut_from_sketch.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace koocadcam::skill::extrude_cut_from_sketch {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

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

TopoDS_Wire makeSketchWire(const std::vector<std::pair<double,double>>& poly,
                           const gp_Pnt& origin,
                           const gp_Dir& normal,
                           const gp_Dir& xDir)
{
    const gp_Vec vx(xDir);
    gp_Vec vy = gp_Vec(normal).Crossed(vx);
    if (vy.Magnitude() < 1e-9)
        throw SkillError("extrude_cut_from_sketch: degenerate frame");
    vy.Normalize();

    std::vector<gp_Pnt> verts;
    verts.reserve(poly.size());
    for (const auto& p : poly) {
        const gp_Pnt q(
            origin.X() + p.first * vx.X() + p.second * vy.X(),
            origin.Y() + p.first * vx.Y() + p.second * vy.Y(),
            origin.Z() + p.first * vx.Z() + p.second * vy.Z());
        verts.push_back(q);
    }

    BRepBuilderAPI_MakeWire wireMk;
    for (size_t i = 0; i < verts.size(); ++i) {
        const gp_Pnt& a = verts[i];
        const gp_Pnt& b = verts[(i + 1) % verts.size()];
        if (a.Distance(b) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(a, b);
        if (!em.IsDone())
            throw SkillError("extrude_cut_from_sketch: edge build failed");
        wireMk.Add(em.Edge());
    }
    if (!wireMk.IsDone())
        throw SkillError("extrude_cut_from_sketch: wire build failed");
    return wireMk.Wire();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.polygon.size() < 3) {
        r.add("DFM-INPUT", "error",
              "extrude_cut_from_sketch: polygon needs >= 3 vertices, got " +
              std::to_string(in.polygon.size()));
    }
    if (in.height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "extrude_cut_from_sketch: height_mm must be > 0");
    }
    if (in.draft_angle_deg < 0.0 || in.draft_angle_deg > 30.0) {
        r.add("DFM-DRAFT", "error",
              "extrude_cut_from_sketch: draft_angle_deg " +
              std::to_string(in.draft_angle_deg) +
              " out of [0, 30] range");
    }
    if (in.polygon.size() >= 2) {
        const double minEdge = minEdgeLengthXY(in.polygon);
        if (minEdge > 0.0 && minEdge < 0.4) {
            r.add("DFM-001", "error",
                  "extrude_cut_from_sketch: min polygon edge " +
                  std::to_string(minEdge) +
                  " mm < 0.4 mm (min wall thickness after cut)");
        }
    }
    if (in.polygon.size() >= 3 && polygonAreaXY(in.polygon) < 1e-6) {
        r.add("DFM-INPUT", "error",
              "extrude_cut_from_sketch: polygon area is ~0 (collinear?)");
    }
    if (!in.through_all && !wp.shape().IsNull()) {
        const auto bb = pr::optimalBbox(wp.shape());
        const double thickness = std::max({ bb.dx(), bb.dy(), bb.dz() });
        if (in.height_mm > thickness + 1e-6) {
            r.add("DFM-INPUT", "error",
                  "extrude_cut_from_sketch: height_mm " +
                  std::to_string(in.height_mm) + " > stock max extent " +
                  std::to_string(thickness) +
                  " (use through_all=true if intended)");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "extrude_cut_from_sketch DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceIdOpt = wp.resolve(in.entry_face);
    if (!faceIdOpt)
        throw SkillError("extrude_cut_from_sketch: entry_face datum unresolved");
    const int faceId = *faceIdOpt;
    const gp_Pnt origin = wp.faceCenter(faceId);
    const gp_Dir normal = wp.faceNormal(faceId);

    gp_Dir xDir = gp::DX();
    if (std::abs(gp_Vec(normal).Dot(gp_Vec(xDir))) > 0.99) xDir = gp::DY();
    gp_Vec vx(xDir);
    vx = vx - gp_Vec(normal) * vx.Dot(gp_Vec(normal));
    if (vx.Magnitude() < 1e-9)
        throw SkillError("extrude_cut_from_sketch: cannot build face X-axis");
    vx.Normalize();
    const gp_Dir xDirFinal(vx);

    // Compute actual cut depth: through_all → use bbox diagonal as a generous
    // sweep so the prism punches clear through.
    double cutDepth = in.height_mm;
    if (in.through_all && !wp.shape().IsNull()) {
        const auto bb = pr::optimalBbox(wp.shape());
        const double diag = std::sqrt(bb.dx() * bb.dx() +
                                       bb.dy() * bb.dy() +
                                       bb.dz() * bb.dz());
        cutDepth = std::max(diag * 1.1, in.height_mm);
    }

    // Build the wire on the face plane, then offset its origin slightly
    // OUTWARD by `overhang` so the cut prism starts just above the face,
    // avoiding co-planar Boolean issues.
    constexpr double kOverhang = 0.05;
    const gp_Pnt liftedOrigin(
        origin.X() + normal.X() * kOverhang,
        origin.Y() + normal.Y() * kOverhang,
        origin.Z() + normal.Z() * kOverhang);

    const TopoDS_Wire wire = makeSketchWire(in.polygon, liftedOrigin,
                                            normal, xDirFinal);

    BRepBuilderAPI_MakeFace faceMk(wire, true);
    if (!faceMk.IsDone())
        throw SkillError("extrude_cut_from_sketch: face build failed");

    // Cut direction = INTO the face (opposite of outward normal).
    const double extLen = cutDepth + 2.0 * kOverhang;
    const gp_Vec extrudeVec(-normal.X() * extLen,
                            -normal.Y() * extLen,
                            -normal.Z() * extLen);
    BRepPrimAPI_MakePrism prism(faceMk.Face(), extrudeVec);
    prism.Build();
    if (!prism.IsDone())
        throw SkillError("extrude_cut_from_sketch: prism build failed");
    const TopoDS_Shape cutter = prism.Shape();

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double area = polygonAreaXY(in.polygon);
    const double vol  = area * cutDepth;

    json polyJson = json::array();
    for (const auto& p : in.polygon)
        polyJson.push_back({ { "x", p.first }, { "y", p.second } });

    json params = {
        { "entry_face_id",       faceId },
        { "polygon",             polyJson },
        { "height_mm",           in.height_mm },
        { "draft_angle_deg",     in.draft_angle_deg },
        { "through_all",         in.through_all },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    1 },
        { "sketch_vertex_count", static_cast<int>(in.polygon.size()) },
        { "height_mm",           cutDepth },
        { "draft_angle_deg",     in.draft_angle_deg },
        { "derived_volume_mm3",  vol },
        { "face_normal",         { normal.X(), normal.Y(), normal.Z() } },
        { "through_all",         in.through_all },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;sketch_cut";
    tooling.tool_dia_mm       = std::max(1.0, minEdgeLengthXY(in.polygon) / 2.0);
    tooling.tool_length_mm    = cutDepth + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 500.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = vol;
    tooling.est_cycle_time_s  = std::max(3.0,
        static_cast<double>(in.polygon.size()) * 0.3 + cutDepth * 0.2);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::extrude_cut_from_sketch applied: n={} h={} vol={} "
                  "faces {}->{}",
                  in.polygon.size(), cutDepth, vol,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "feature_history" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric: a cut pocket = N inward-facing planar wall faces + 1 bottom.
    // Count planar vertical walls (normal.Z ≈ 0) — same heuristic as boss.
    int verticalPlanarCount = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(n.Z()) < 0.05) ++verticalPlanarCount;
    }
    if (verticalPlanarCount < 3) return out;

    json recovered = {
        { "entry_face_id",   -1 },
        { "polygon",         json::array() },
        { "height_mm",       0.0 },
        { "draft_angle_deg", 0.0 },
        { "through_all",     false },
    };
    json matched = {
        { "vertical_planar_face_count", verticalPlanarCount },
    };
    const double conf = std::clamp(0.35 + 0.02 * verticalPlanarCount, 0.35, 0.75);
    out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    return out;
}

}  // namespace koocadcam::skill::extrude_cut_from_sketch
