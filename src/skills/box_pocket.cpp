// @lat: [[engine/skills#box_pocket]]

#include "box_pocket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <BRep_Tool.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::box_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.length_mm <= 0.0) r.add("DFM-INPUT", "error", "box_pocket: length_mm must be > 0");
    if (in.width_mm  <= 0.0) r.add("DFM-INPUT", "error", "box_pocket: width_mm must be > 0");
    if (in.depth_mm  <= 0.0) r.add("DFM-INPUT", "error", "box_pocket: depth_mm must be > 0");
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "box_pocket DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double embed = 0.05;

    // Determine the pocket frame: either a resolved planar datum (forward path) or
    // a direct world placement (foreign recovery, incl. a side button on a face
    // with no resolvable datum after a STEP round-trip).
    gp_Pnt origin;     // the pocket MOUTH centre on the entry surface
    gp_Dir normal;     // entry outward direction (recess goes -normal)
    double cx = in.center_x_mm, cy = in.center_y_mm;
    if (in.use_world) {
        normal = gp_Dir(in.world_nx, in.world_ny, in.world_nz);
        // world_* is the MOUTH centre (on the entry face); the recess goes inward.
        origin = gp_Pnt(in.world_cx_mm, in.world_cy_mm, in.world_cz_mm);
        cx = 0.0; cy = 0.0;
    } else {
        auto faceIdOpt = wp.resolve(in.entry_face);
        if (!faceIdOpt) throw SkillError("box_pocket: entry_face datum unresolved");
        origin = wp.faceCenter(*faceIdOpt);
        normal = wp.faceNormal(*faceIdOpt);
    }

    // In-plane X axis: use the recovered length direction when provided (so an
    // off-axis pocket regenerates at the right orientation AND size), else any
    // axis orthogonal to the normal.
    gp_Vec vx;
    if (in.use_world &&
        (std::abs(in.world_xx) + std::abs(in.world_xy) + std::abs(in.world_xz)) > 1e-9) {
        vx = gp_Vec(in.world_xx, in.world_xy, in.world_xz);
    } else {
        vx = gp_Vec(gp::DX());
        if (std::abs(vx.Dot(gp_Vec(normal))) > 0.99) vx = gp_Vec(gp::DY());
    }
    vx = vx - gp_Vec(normal) * vx.Dot(gp_Vec(normal));
    if (vx.Magnitude() < 1e-9) throw SkillError("box_pocket: cannot build face X-axis");
    vx.Normalize();
    const gp_Vec vy = gp_Vec(normal).Crossed(vx);

    // Corner of the cutter footprint.  The box axis is gp_Ax2(corner, -normal,
    // vx), so its DX runs +vx (length) and its DY runs (-normal)x vx = -vy.  To
    // keep the pocket CENTRED on (cx,cy) despite the -vy DY, the corner's width
    // offset is +0.5*W*vy (the box then grows -vy from +0.5W to -0.5W, centred at
    // 0).  The length offset is -0.5*L*vx (DX=+vx grows to +0.5L, centred).  The
    // corner is pushed +embed OUTWARD so the cut breaks cleanly through the face.
    const gp_Pnt corner(
        origin.X() + cx * vx.X() + cy * vy.X() - 0.5 * in.length_mm * vx.X() + 0.5 * in.width_mm * vy.X() + embed * normal.X(),
        origin.Y() + cx * vx.Y() + cy * vy.Y() - 0.5 * in.length_mm * vx.Y() + 0.5 * in.width_mm * vy.Y() + embed * normal.Y(),
        origin.Z() + cx * vx.Z() + cy * vy.Z() - 0.5 * in.length_mm * vx.Z() + 0.5 * in.width_mm * vy.Z() + embed * normal.Z());

    // gp_Ax2(P, V=-normal, Vx): box DX along Vx (length), DY along (-N) x Vx
    // (= -vy, width), DZ along -V (depth, sinking into the body).
    const gp_Ax2 ax(corner, gp_Dir(-normal.X(), -normal.Y(), -normal.Z()), gp_Dir(vx));
    const TopoDS_Shape cutter = pr::box(ax, in.length_mm, in.width_mm, in.depth_mm + embed);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double vol = in.length_mm * in.width_mm * in.depth_mm;

    // World pocket-mouth centre (origin + face-local offset).  center_[xyz]_world
    // gives CAM the machining frame directly: the mouth (entry) plane is at
    // *_z_world and the recess sinks depth_mm below it.
    const gp_Pnt mouthCtr(origin.X() + cx * vx.X() + cy * vy.X(),
                          origin.Y() + cx * vx.Y() + cy * vy.Y(),
                          origin.Z() + cx * vx.Z() + cy * vy.Z());

    json params = {
        { "center_x_mm",       in.center_x_mm },
        { "center_y_mm",       in.center_y_mm },
        { "center_x_world_mm", mouthCtr.X() },
        { "center_y_world_mm", mouthCtr.Y() },
        { "center_z_world_mm", mouthCtr.Z() },   // entry (mouth) plane; recess sinks below
        { "length_mm",     in.length_mm },
        { "width_mm",      in.width_mm },
        { "depth_mm",      in.depth_mm },
        { "face_normal",   { normal.X(), normal.Y(), normal.Z() } },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "is_compound",        false },
        { "length_mm",          in.length_mm },
        { "width_mm",           in.width_mm },
        { "depth_mm",           in.depth_mm },
        { "derived_volume_mm3", vol },
        { "face_normal",        { normal.X(), normal.Y(), normal.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;box_pocket";
    tooling.tool_dia_mm       = std::min(in.length_mm, in.width_mm) * 0.5;
    tooling.tool_length_mm    = in.depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = vol;    // material REMOVED
    tooling.est_cycle_time_s  = std::max(2.0, in.depth_mm * 0.3);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(validateOrHeal(newShape, "box_pocket"),
                                             wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::box_pocket applied: L={} W={} D={} vol={}",
                  in.length_mm, in.width_mm, in.depth_mm, vol);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

namespace {

struct PlanarFace {
    int    id;
    gp_Dir n;
    gp_Pnt c;
    double area;
};

// Two faces are adjacent iff they share an edge (same TopoDS_Edge instance).
bool sharesEdge(const TopoDS_Face& a, const TopoDS_Face& b)
{
    for (TopExp_Explorer ea(a, TopAbs_EDGE); ea.More(); ea.Next()) {
        for (TopExp_Explorer eb(b, TopAbs_EDGE); eb.More(); eb.Next()) {
            if (ea.Current().IsSame(eb.Current())) return true;
        }
    }
    return false;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // metadata replay (cap-exempt full confidence).
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric path B: a rectangular POCKET presents a planar FLOOR bounded by
    // 3-or-4 perpendicular SIDE WALLS that face INWARD (toward the floor centre),
    // with the floor RECESSED below the entry plane.  This is the subtractive
    // mirror of box_boss (whose walls face OUTWARD and whose cap sits at the outer
    // boundary).  Recover L/W from the floor edges, depth from the wall extent.
    std::vector<PlanarFace> planar;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        try {
            planar.push_back({ i, wp.faceNormal(i), wp.faceCenter(i), wp.faceArea(i) });
        } catch (...) {}
    }

    auto axialOf = [](const gp_Pnt& p, const gp_Dir& n) {
        return p.X() * n.X() + p.Y() * n.Y() + p.Z() * n.Z();
    };

    Bnd_Box bodyBox;
    BRepBndLib::Add(wp.shape(), bodyBox);
    double bx0, by0, bz0, bx1, by1, bz1;
    bodyBox.Get(bx0, by0, bz0, bx1, by1, bz1);

    for (const auto& floor : planar) {
        // Side walls of a box pocket: planar faces ADJACENT to the floor (sharing
        // an edge) whose normal is PERPENDICULAR to the floor normal.  A fully
        // enclosed pocket has 4; a pocket whose one wall runs off a rounded corner
        // keeps 3.
        std::vector<int> walls;
        double wallDepth = 0.0;   // max side-wall extent along the floor normal
        int outwardWalls = 0, inwardWalls = 0;
        for (const auto& w : planar) {
            if (w.id == floor.id) continue;
            const double dot = std::abs(gp_Vec(floor.n).Dot(gp_Vec(w.n)));
            if (dot > 0.05) continue;                 // not perpendicular to floor
            if (!sharesEdge(wp.face(floor.id), wp.face(w.id))) continue;  // must touch the floor
            walls.push_back(w.id);
            // Wall extent along the floor normal = the pocket depth.  This world-
            // AABB span is exact when floor.n is axis-aligned (all shipped ±X/±Y/
            // ±Z pockets); for a TILTED pocket axis the wall's in-plane length
            // would leak into the projection — a follow-up (use per-vertex
            // projection onto floor.n, as the L/W frame below already does).
            Bnd_Box wb; BRepBndLib::Add(wp.face(w.id), wb);
            double a0, b0, c0, a1, b1, c1; wb.Get(a0, b0, c0, a1, b1, c1);
            const gp_Vec span(a1 - a0, b1 - b0, c1 - c0);
            wallDepth = std::max(wallDepth, std::abs(span.Dot(gp_Vec(floor.n))));
            // CONVEXITY — a POCKET wall faces INWARD (its outward normal points
            // BACK toward the floor centre); a BOSS wall faces OUTWARD.  This is
            // the exact inverse of box_boss's classifier, and rejects the outer
            // cap of a boss (whose walls face outward).
            const gp_Vec floorToWall(floor.c, w.c);
            if (floorToWall.Dot(gp_Vec(w.n)) < 0) ++inwardWalls; else ++outwardWalls;
        }
        if (walls.size() < 3 || walls.size() > 4) continue;   // a pocket: 3 or 4 walls
        if (inwardWalls < outwardWalls) continue;     // a protrusion (boss), not a pocket

        // The pocket DEPTH is the side-wall extent along the floor normal.
        const double depth = wallDepth;
        if (depth < 0.3) continue;

        // RECESS gate — the inverse of box_boss's protrusion gate.  The floor must
        // be RECESSED (inboard of the outer boundary), while the MOUTH (floor +
        // depth back toward the surface) is at the outer boundary.  Two conditions:
        //  (a) short: the pocket depth is a small fraction of the body span (a
        //      through-cut spans the FULL thickness);
        //  (b) the floor is NOT at the outer boundary (it is sunk inward), but the
        //      mouth IS — a genuine recess, not a boss cap or a bare face.
        const double floorAxial = axialOf(floor.c, floor.n);
        const double bodyMaxAlongN =
            std::max({ axialOf(gp_Pnt(bx0, by0, bz0), floor.n), axialOf(gp_Pnt(bx1, by0, bz0), floor.n),
                       axialOf(gp_Pnt(bx0, by1, bz0), floor.n), axialOf(gp_Pnt(bx1, by1, bz0), floor.n),
                       axialOf(gp_Pnt(bx0, by0, bz1), floor.n), axialOf(gp_Pnt(bx1, by0, bz1), floor.n),
                       axialOf(gp_Pnt(bx0, by1, bz1), floor.n), axialOf(gp_Pnt(bx1, by1, bz1), floor.n) });
        const double bodyMinAlongN =
            std::min({ axialOf(gp_Pnt(bx0, by0, bz0), floor.n), axialOf(gp_Pnt(bx1, by0, bz0), floor.n),
                       axialOf(gp_Pnt(bx0, by1, bz0), floor.n), axialOf(gp_Pnt(bx1, by1, bz0), floor.n),
                       axialOf(gp_Pnt(bx0, by0, bz1), floor.n), axialOf(gp_Pnt(bx1, by0, bz1), floor.n),
                       axialOf(gp_Pnt(bx0, by1, bz1), floor.n), axialOf(gp_Pnt(bx1, by1, bz1), floor.n) });
        const double bodySpan = bodyMaxAlongN - bodyMinAlongN;
        if (depth > 0.6 * bodySpan) continue;              // (a) not short → a through-cut
        // The floor's outward normal (floor.n) points UP out of the pocket toward
        // the mouth, so a recessed floor sits BELOW the outer boundary by ~depth.
        const double mouthAxial = floorAxial + wallDepth;  // walls rise back to the entry face
        if (mouthAxial < bodyMaxAlongN - 0.2) continue;    // (b) mouth at the outer boundary...
        if (floorAxial > bodyMaxAlongN - 0.2) continue;    // ...but the floor is NOT (recessed)

        // L/W in the FLOOR'S OWN in-plane frame (NOT a world AABB, which over-sizes
        // and mis-orients a rotated/side pocket).  Longest floor edge -> vX.
        gp_Dir floorX;
        {
            double bestLen = -1.0; gp_Vec bestDir(1, 0, 0);
            for (TopExp_Explorer ev(wp.face(floor.id), TopAbs_EDGE); ev.More(); ev.Next()) {
                TopoDS_Vertex v0, v1;
                TopExp::Vertices(TopoDS::Edge(ev.Current()), v0, v1);
                if (v0.IsNull() || v1.IsNull()) continue;
                gp_Vec e(BRep_Tool::Pnt(v0), BRep_Tool::Pnt(v1));
                e = e - gp_Vec(floor.n) * e.Dot(gp_Vec(floor.n));
                if (e.Magnitude() > bestLen) { bestLen = e.Magnitude(); bestDir = e; }
            }
            if (bestLen < 1e-6) continue;
            bestDir.Normalize();
            floorX = gp_Dir(bestDir);
        }
        const gp_Vec vX(floorX);
        const gp_Vec vY = gp_Vec(floor.n).Crossed(vX);
        double uMin = 1e30, uMax = -1e30, wMin = 1e30, wMax = -1e30;
        for (TopExp_Explorer vp(wp.face(floor.id), TopAbs_VERTEX); vp.More(); vp.Next()) {
            const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(vp.Current()));
            const gp_Vec rel(floor.c, p);
            const double u = rel.Dot(vX), w = rel.Dot(vY);
            uMin = std::min(uMin, u); uMax = std::max(uMax, u);
            wMin = std::min(wMin, w); wMax = std::max(wMax, w);
        }
        const double length = uMax - uMin;
        const double width  = wMax - wMin;
        if (length < 0.5 || width < 0.5) continue;

        // FOOTPRINT discriminator — reject a thin GROOVE/slot land (extreme aspect)
        // and a sliver.  box_pocket is precise-tier (cap-exempt at 0.85).
        const double aspect = std::max(length, width) / std::max(1e-6, std::min(length, width));
        if (aspect > 12.0) continue;                  // a groove / slot land, not a pocket
        if (std::min(length, width) < 0.8) continue;  // too sliver

        // MOUTH centre = floor centre lifted back along +floor.n by depth (the
        // entry plane).  apply() cuts inward from there.
        const gp_Pnt mouth(floor.c.X() + floor.n.X() * wallDepth,
                           floor.c.Y() + floor.n.Y() * wallDepth,
                           floor.c.Z() + floor.n.Z() * wallDepth);

        json recovered = {
            { "length_mm",    length },
            { "width_mm",     width },
            { "depth_mm",     depth },
            { "face_normal",  { floor.n.X(), floor.n.Y(), floor.n.Z() } },
            { "face_xaxis",   { floorX.X(), floorX.Y(), floorX.Z() } },
            { "use_world",    true },
            { "world_center", { mouth.X(), mouth.Y(), mouth.Z() } },
            // Mouth (entry) plane centre for CAM — the field/recess is cut below.
            { "center_x_world_mm", mouth.X() },
            { "center_y_world_mm", mouth.Y() },
            { "center_z_world_mm", mouth.Z() },
        };
        json wallIds = json::array();
        for (int wid : walls) wallIds.push_back(wid);
        json matched = {
            { "source",        "geometry" },
            { "floor_face_id", floor.id },
            { "wall_face_ids", wallIds },             // counted by collectFaceIds (dedupe)
            { "wall_count",    static_cast<int>(walls.size()) },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.85, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::box_pocket
