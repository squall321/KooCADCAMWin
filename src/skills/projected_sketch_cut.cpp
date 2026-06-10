// @lat: [[engine/skills#projected_sketch_cut]]

#include "projected_sketch_cut.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"

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

namespace koocadcam::skill::projected_sketch_cut {

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

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.polygon.size() < 3) {
        r.add("DFM-INPUT", "error",
              "projected_sketch_cut: polygon needs >= 3 vertices, got " +
              std::to_string(in.polygon.size()));
    }
    if (in.cut_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "projected_sketch_cut: cut_depth_mm must be > 0");
    }
    const double projMag = std::sqrt(in.projection_dir_xyz.X() * in.projection_dir_xyz.X() +
                                     in.projection_dir_xyz.Y() * in.projection_dir_xyz.Y() +
                                     in.projection_dir_xyz.Z() * in.projection_dir_xyz.Z());
    if (projMag < 1e-9) {
        r.add("DFM-INPUT", "error",
              "projected_sketch_cut: projection_dir_xyz is zero-length");
    }
    if (in.polygon.size() >= 2) {
        const double minEdge = minEdgeLengthXY(in.polygon);
        if (minEdge > 0.0 && minEdge < 0.4) {
            r.add("DFM-001", "error",
                  "projected_sketch_cut: min polygon edge " +
                  std::to_string(minEdge) +
                  " mm < 0.4 mm (min wall thickness after cut)");
        }
    }
    if (in.target_face_id >= 0 && in.target_face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "projected_sketch_cut: target_face_id " +
              std::to_string(in.target_face_id) +
              " out of range (faceCount=" +
              std::to_string(wp.faceCount()) + ")");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "projected_sketch_cut DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Compute projection plane origin: top of the workpiece bbox along the
    // OPPOSITE of projection_dir (i.e. the "start" of the cut).  Sketch is
    // assumed to be authored in world XY at z = projOriginZ.
    const auto bb = pr::optimalBbox(wp.shape());

    // Build a local XY frame from the projection direction.  Take projection
    // axis as local Z.  Build local X = projection × world_Y (with fallback
    // to world_X if degenerate).
    const gp_Dir projDir = in.projection_dir_xyz;
    gp_Vec vz(projDir);
    gp_Vec vy(0.0, 1.0, 0.0);
    if (std::abs(vz.Dot(vy)) > 0.99) vy = gp_Vec(1.0, 0.0, 0.0);
    gp_Vec vx = vy.Crossed(vz);
    if (vx.Magnitude() < 1e-9)
        throw SkillError("projected_sketch_cut: cannot build orthonormal frame");
    vx.Normalize();
    vy = vz.Crossed(vx);
    vy.Normalize();

    // Origin: bbox center, lifted along -projDir to just above the bbox.
    const gp_Pnt bbCenter = bb.center();
    const double bboxRadius = std::sqrt(bb.dx() * bb.dx() +
                                        bb.dy() * bb.dy() +
                                        bb.dz() * bb.dz()) / 2.0 + 1.0;
    const gp_Pnt origin(
        bbCenter.X() - projDir.X() * bboxRadius,
        bbCenter.Y() - projDir.Y() * bboxRadius,
        bbCenter.Z() - projDir.Z() * bboxRadius);

    // Build polygon vertices in world frame.
    std::vector<gp_Pnt> verts;
    verts.reserve(in.polygon.size());
    for (const auto& p : in.polygon) {
        verts.emplace_back(
            origin.X() + p.first * vx.X() + p.second * vy.X(),
            origin.Y() + p.first * vx.Y() + p.second * vy.Y(),
            origin.Z() + p.first * vx.Z() + p.second * vy.Z());
    }

    BRepBuilderAPI_MakeWire wireMk;
    for (size_t i = 0; i < verts.size(); ++i) {
        const gp_Pnt& a = verts[i];
        const gp_Pnt& b = verts[(i + 1) % verts.size()];
        if (a.Distance(b) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(a, b);
        if (!em.IsDone())
            throw SkillError("projected_sketch_cut: edge build failed");
        wireMk.Add(em.Edge());
    }
    if (!wireMk.IsDone())
        throw SkillError("projected_sketch_cut: wire build failed");

    BRepBuilderAPI_MakeFace faceMk(wireMk.Wire(), true);
    if (!faceMk.IsDone())
        throw SkillError("projected_sketch_cut: face build failed");

    // Extrude along projection_dir by (bboxRadius + cut_depth_mm) so the
    // prism punches FROM above the bbox THROUGH the projection plus depth.
    const double extLen = bboxRadius + in.cut_depth_mm + 1.0;
    const gp_Vec extrudeVec(projDir.X() * extLen,
                            projDir.Y() * extLen,
                            projDir.Z() * extLen);
    BRepPrimAPI_MakePrism prism(faceMk.Face(), extrudeVec);
    prism.Build();
    if (!prism.IsDone())
        throw SkillError("projected_sketch_cut: prism build failed");
    const TopoDS_Shape cutter = prism.Shape();

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double area = polygonAreaXY(in.polygon);
    const double vol  = area * in.cut_depth_mm;

    json polyJson = json::array();
    for (const auto& p : in.polygon)
        polyJson.push_back({ { "x", p.first }, { "y", p.second } });

    json params = {
        { "target_face_id",     in.target_face_id },
        { "polygon",            polyJson },
        { "projection_dir_xyz", { in.projection_dir_xyz.X(),
                                  in.projection_dir_xyz.Y(),
                                  in.projection_dir_xyz.Z() } },
        { "cut_depth_mm",       in.cut_depth_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    1 },
        { "sketch_vertex_count", static_cast<int>(in.polygon.size()) },
        { "cut_depth_mm",        in.cut_depth_mm },
        { "derived_volume_mm3",  vol },
        { "projection_dir",      { in.projection_dir_xyz.X(),
                                   in.projection_dir_xyz.Y(),
                                   in.projection_dir_xyz.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;projected_cut";
    tooling.tool_dia_mm       = std::max(1.0, minEdgeLengthXY(in.polygon) / 2.0);
    tooling.tool_length_mm    = in.cut_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 450.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = vol;
    tooling.est_cycle_time_s  = std::max(3.0,
        static_cast<double>(in.polygon.size()) * 0.3 + in.cut_depth_mm * 0.2);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::projected_sketch_cut applied: n={} depth={} vol={} "
                  "faces {}->{}",
                  in.polygon.size(), in.cut_depth_mm, vol,
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

    // Geometric: a projected cut typically leaves a cluster of planar wall
    // faces oriented along the projection direction.  We don't know the
    // projection_dir a priori, so just count planar faces.
    int planarCount = 0;
    for (int i = 0; i < wp.faceCount(); ++i)
        if (wp.isFacePlanar(i)) ++planarCount;

    if (planarCount < 3) return out;

    json recovered = {
        { "target_face_id",     -1 },
        { "polygon",            json::array() },
        { "projection_dir_xyz", { 0.0, 0.0, -1.0 } },
        { "cut_depth_mm",       0.0 },
    };
    json matched = {
        { "planar_face_count", planarCount },
    };
    const double conf = std::clamp(0.30 + 0.01 * planarCount, 0.30, 0.65);
    out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    return out;
}

}  // namespace koocadcam::skill::projected_sketch_cut
