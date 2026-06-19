// @lat: [[engine/skills#extrude_boss_from_sketch]]

#include "extrude_boss_from_sketch.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"

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
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
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
// `poly` is XY in the local (xDir, yDir = normal × xDir) frame.
TopoDS_Wire makeSketchWire(const std::vector<std::pair<double,double>>& poly,
                           const gp_Pnt& origin,
                           const gp_Dir& normal,
                           const gp_Dir& xDir)
{
    const gp_Vec vx(xDir);
    gp_Vec vyTmp = gp_Vec(normal).Crossed(vx);
    if (vyTmp.Magnitude() < 1e-9)
        throw SkillError("extrude_boss_from_sketch: degenerate frame (normal || xDir)");
    vyTmp.Normalize();
    const gp_Vec vn(normal);

    std::vector<gp_Pnt> verts;
    verts.reserve(poly.size());
    for (const auto& p : poly) {
        const double dx = p.first;
        const double dy = p.second;
        const gp_Pnt q(
            origin.X() + dx * vx.X() + dy * vyTmp.X(),
            origin.Y() + dx * vx.Y() + dy * vyTmp.Y(),
            origin.Z() + dx * vx.Z() + dy * vyTmp.Z());
        verts.push_back(q);
        (void)vn;  // suppress unused if not needed below
    }

    BRepBuilderAPI_MakeWire wireMk;
    for (size_t i = 0; i < verts.size(); ++i) {
        const gp_Pnt& a = verts[i];
        const gp_Pnt& b = verts[(i + 1) % verts.size()];
        if (a.Distance(b) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(a, b);
        if (!em.IsDone())
            throw SkillError("extrude_boss_from_sketch: edge build failed");
        wireMk.Add(em.Edge());
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

    if (in.polygon.size() < 3) {
        r.add("DFM-INPUT", "error",
              "extrude_boss_from_sketch: polygon needs >= 3 vertices, got " +
              std::to_string(in.polygon.size()));
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
    if (in.polygon.size() >= 2) {
        const double minEdge = minEdgeLengthXY(in.polygon);
        if (minEdge > 0.0 && minEdge < 0.4) {
            r.add("DFM-001", "error",
                  "extrude_boss_from_sketch: min polygon edge " +
                  std::to_string(minEdge) +
                  " mm < 0.4 mm (min wall thickness)");
        }
    }
    if (in.polygon.size() >= 3 && polygonAreaXY(in.polygon) < 1e-6) {
        r.add("DFM-INPUT", "error",
              "extrude_boss_from_sketch: polygon area is ~0 (collinear?)");
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

    // Build wire on face plane.
    const TopoDS_Wire wire = makeSketchWire(in.polygon, origin, normal, xDirFinal);

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

    const double area  = polygonAreaXY(in.polygon);
    const double vol   = area * in.height_mm;

    // ── signature ────────────────────────────────────────────────────────
    json polyJson = json::array();
    for (const auto& p : in.polygon)
        polyJson.push_back({ { "x", p.first }, { "y", p.second } });

    json params = {
        { "entry_face_id",       faceId },
        { "polygon",             polyJson },
        { "height_mm",           in.height_mm },
        { "draft_angle_deg",     in.draft_angle_deg },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    1 },     // sketch face + prism fuse
        { "sketch_vertex_count", static_cast<int>(in.polygon.size()) },
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
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        try {
            const gp_Dir n = wp.faceNormal(i);
            planar.push_back({ i, n, wp.faceCenter(i), wp.faceArea(i) });
            if (std::abs(n.Z()) < 0.05) ++verticalPlanar;
        } catch (...) { continue; }
    }
    // A pristine rectangular box has exactly 4 vertical planar faces and is NOT
    // a boss (project convention: bare stock recognises as nothing).  A real
    // polygonal prism / boss has more side walls, so require > 4.
    if (verticalPlanar <= 4) return out;

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

    json polyJson = json::array();
    try {
        const TopoDS_Wire ow = BRepTools::OuterWire(wp.face(capA));
        for (BRepTools_WireExplorer wexp(ow); wexp.More(); wexp.Next()) {
            const gp_Pnt p = BRep_Tool::Pnt(wexp.CurrentVertex());
            const gp_Vec rel(origin, p);
            polyJson.push_back({ { "x", rel.Dot(vx) }, { "y", rel.Dot(vy) } });
        }
    } catch (...) { return out; }
    if (polyJson.size() < 3) return out;

    // SPECIFICITY gate 1 (topology, exact): a TRUE straight-walled N-gon prism
    // has EXACTLY N side walls + 2 caps = N+2 faces.  A pocketed/drilled/slotted
    // block has the same congruent cap pair but EXTRA faces (the feature's walls
    // + floor), so faceCount > N+2.  This catches even a SMALL machined feature
    // that volume tolerance alone would miss (fpscan rect_pocket / slot).
    if (wp.faceCount() != static_cast<int>(polyJson.size()) + 2) return out;

    // SPECIFICITY gate 2 (volume): a TRUE prism has volume == profile_area *
    // height — belt-and-braces against a degenerate same-face-count case (a
    // boss-on-stock has more volume, a through-feature less).
    double profArea = 0.0;
    {
        const std::size_t n = polyJson.size();
        for (std::size_t i = 0; i < n; ++i) {
            const auto& p = polyJson[i];
            const auto& q = polyJson[(i + 1) % n];
            profArea += p["x"].get<double>() * q["y"].get<double>()
                      - q["x"].get<double>() * p["y"].get<double>();
        }
        profArea = std::abs(profArea) * 0.5;
    }
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

    json recovered = {
        { "entry_face_id",   capA },
        { "polygon",         polyJson },
        { "height_mm",       height },
        { "draft_angle_deg", 0.0 },
    };
    json matched = {
        { "source",       "geometry" },
        { "cap_face_a",   capA },
        { "cap_face_b",   capB },
        { "vertex_count", static_cast<int>(polyJson.size()) },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.9, matched });
    return out;
}

}  // namespace koocadcam::skill::extrude_boss_from_sketch
