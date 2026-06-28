// @lat: [[engine/skills#box_boss]]

#include "box_boss.hpp"

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

namespace koocadcam::skill::box_boss {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.length_mm <= 0.0) r.add("DFM-INPUT", "error", "box_boss: length_mm must be > 0");
    if (in.width_mm  <= 0.0) r.add("DFM-INPUT", "error", "box_boss: width_mm must be > 0");
    if (in.height_mm <= 0.0) r.add("DFM-INPUT", "error", "box_boss: height_mm must be > 0");
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "box_boss DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double embed = 0.05;

    // Determine the boss frame: either a resolved planar datum (forward path) or
    // a direct world placement (foreign recovery, incl. a lug on a curved host).
    gp_Pnt origin;     // the boss-base centre on the host surface
    gp_Dir normal;     // outward direction
    double cx = in.center_x_mm, cy = in.center_y_mm;
    if (in.use_world) {
        normal = gp_Dir(in.world_nx, in.world_ny, in.world_nz);
        // world_* is the OUTER CAP centre; the base sits height inboard.
        origin = gp_Pnt(in.world_cx_mm - normal.X() * in.height_mm,
                        in.world_cy_mm - normal.Y() * in.height_mm,
                        in.world_cz_mm - normal.Z() * in.height_mm);
        cx = 0.0; cy = 0.0;
    } else {
        auto faceIdOpt = wp.resolve(in.entry_face);
        if (!faceIdOpt) throw SkillError("box_boss: entry_face datum unresolved");
        origin = wp.faceCenter(*faceIdOpt);
        normal = wp.faceNormal(*faceIdOpt);
    }

    // In-plane X axis: use the recovered length direction when provided (so an
    // off-axis lug regenerates at the right orientation AND size), else any axis
    // orthogonal to the normal (fine for a square boss or forward authoring).
    gp_Vec vx;
    if (in.use_world &&
        (std::abs(in.world_xx) + std::abs(in.world_xy) + std::abs(in.world_xz)) > 1e-9) {
        vx = gp_Vec(in.world_xx, in.world_xy, in.world_xz);
    } else {
        vx = gp_Vec(gp::DX());
        if (std::abs(vx.Dot(gp_Vec(normal))) > 0.99) vx = gp_Vec(gp::DY());
    }
    vx = vx - gp_Vec(normal) * vx.Dot(gp_Vec(normal));
    if (vx.Magnitude() < 1e-9) throw SkillError("box_boss: cannot build face X-axis");
    vx.Normalize();
    const gp_Vec vy = gp_Vec(normal).Crossed(vx);

    // Corner of the box footprint: centre - L/2 X - W/2 Y, lifted a hair INTO
    // the host so the fuse is clean.
    const gp_Pnt corner(
        origin.X() + cx * vx.X() + cy * vy.X() - 0.5 * in.length_mm * vx.X() - 0.5 * in.width_mm * vy.X() - embed * normal.X(),
        origin.Y() + cx * vx.Y() + cy * vy.Y() - 0.5 * in.length_mm * vx.Y() - 0.5 * in.width_mm * vy.Y() - embed * normal.Y(),
        origin.Z() + cx * vx.Z() + cy * vy.Z() - 0.5 * in.length_mm * vx.Z() - 0.5 * in.width_mm * vy.Z() - embed * normal.Z());

    // gp_Ax2(P, V=normal, Vx): box DX along Vx (length), DY along N x Vx (width),
    // DZ along V (height + embed, rising out of the face).
    const gp_Ax2 ax(corner, normal, gp_Dir(vx));
    const TopoDS_Shape boss = pr::box(ax, in.length_mm, in.width_mm, in.height_mm + embed);
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), boss);

    const double vol = in.length_mm * in.width_mm * in.height_mm;

    json params = {
        { "center_x_mm",   in.center_x_mm },
        { "center_y_mm",   in.center_y_mm },
        { "length_mm",     in.length_mm },
        { "width_mm",      in.width_mm },
        { "height_mm",     in.height_mm },
        { "face_normal",   { normal.X(), normal.Y(), normal.Z() } },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "is_compound",        false },
        { "length_mm",          in.length_mm },
        { "width_mm",           in.width_mm },
        { "height_mm",          in.height_mm },
        { "derived_volume_mm3", vol },
        { "face_normal",        { normal.X(), normal.Y(), normal.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "form_die;box_add";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = in.height_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = -vol;   // material ADDED
    tooling.est_cycle_time_s  = std::max(2.0, in.height_mm * 0.2);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(validateOrHeal(newShape, "box_boss"),
                                             wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::box_boss applied: L={} W={} H={} vol={}",
                  in.length_mm, in.width_mm, in.height_mm, vol);

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

    // Geometric path B: a rectangular boss presents an OUTER CAP (a planar face)
    // bounded by 4 perpendicular SIDE WALLS (planar, normals in-plane to the
    // cap).  The cap's normal is the boss/host outward direction.  We look for a
    // planar cap face that has EXACTLY 4 adjacent planar faces whose normals are
    // mutually perpendicular pairs and perpendicular to the cap normal — a box
    // top.  Recover L/W from the cap's edge extents, H from the side-wall extent.
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

    // Whole-body span along an axis (to tell a short boss from a full-thickness
    // cuboid wall).
    Bnd_Box bodyBox;
    BRepBndLib::Add(wp.shape(), bodyBox);
    double bx0, by0, bz0, bx1, by1, bz1;
    bodyBox.Get(bx0, by0, bz0, bx1, by1, bz1);

    for (const auto& cap : planar) {
        // Side walls of a box cap: planar faces ADJACENT to the cap (sharing an
        // edge) whose normal is PERPENDICULAR to the cap normal.  A free-standing
        // box top has 4; a box FUSED to a curved host (a watch lug on the round
        // case side) loses one wall into the case, so accept 3 or 4.
        std::vector<int> walls;
        double wallHeight = 0.0;   // max side-wall extent along the cap normal
        for (const auto& w : planar) {
            if (w.id == cap.id) continue;
            const double dot = std::abs(gp_Vec(cap.n).Dot(gp_Vec(w.n)));
            if (dot > 0.05) continue;                 // not perpendicular to cap
            if (!sharesEdge(wp.face(cap.id), wp.face(w.id))) continue;  // must touch the cap
            walls.push_back(w.id);
            Bnd_Box wb; BRepBndLib::Add(wp.face(w.id), wb);
            double a0, b0, c0, a1, b1, c1; wb.Get(a0, b0, c0, a1, b1, c1);
            const gp_Vec span(a1 - a0, b1 - b0, c1 - c0);
            wallHeight = std::max(wallHeight, std::abs(span.Dot(gp_Vec(cap.n))));
        }
        if (walls.size() < 3 || walls.size() > 4) continue;   // a box top: 3 or 4 walls

        // The boss HEIGHT is the side-wall extent along the cap normal.
        const double height = wallHeight;
        if (height < 0.3) continue;

        // PROTRUSION gate — distinguish a real boss from a bare cuboid's own
        // face (which also has 4 perpendicular walls).  Two conditions:
        //  (a) the boss is SHORT: its wall height is a small fraction of the
        //      body's full span along the cap normal (a cuboid wall spans the
        //      FULL thickness; a lug/island wall spans only the boss rise);
        //  (b) the cap sits at the OUTER boundary along the cap normal (it is the
        //      outermost material in that direction — a protrusion, not a recess).
        const double capAxial = axialOf(cap.c, cap.n);
        const double bodyMaxAlongN =
            std::max({ axialOf(gp_Pnt(bx0, by0, bz0), cap.n), axialOf(gp_Pnt(bx1, by0, bz0), cap.n),
                       axialOf(gp_Pnt(bx0, by1, bz0), cap.n), axialOf(gp_Pnt(bx1, by1, bz0), cap.n),
                       axialOf(gp_Pnt(bx0, by0, bz1), cap.n), axialOf(gp_Pnt(bx1, by0, bz1), cap.n),
                       axialOf(gp_Pnt(bx0, by1, bz1), cap.n), axialOf(gp_Pnt(bx1, by1, bz1), cap.n) });
        const double bodyMinAlongN =
            std::min({ axialOf(gp_Pnt(bx0, by0, bz0), cap.n), axialOf(gp_Pnt(bx1, by0, bz0), cap.n),
                       axialOf(gp_Pnt(bx0, by1, bz0), cap.n), axialOf(gp_Pnt(bx1, by1, bz0), cap.n),
                       axialOf(gp_Pnt(bx0, by0, bz1), cap.n), axialOf(gp_Pnt(bx1, by0, bz1), cap.n),
                       axialOf(gp_Pnt(bx0, by1, bz1), cap.n), axialOf(gp_Pnt(bx1, by1, bz1), cap.n) });
        const double bodySpan = bodyMaxAlongN - bodyMinAlongN;
        if (height > 0.6 * bodySpan) continue;        // (a) not short → a full wall, not a boss
        if (capAxial < bodyMaxAlongN - 0.2) continue; // (b) cap not at the outer boundary

        // L/W in the CAP'S OWN in-plane frame (NOT a world AABB, which would
        // over-size and mis-orient a rotated/radial lug: a 12x8 cap at 45deg has
        // a 14.14x14.14 world AABB).  Build orthonormal in-plane axes from the
        // longest cap edge, then project the cap's vertices onto them.
        gp_Dir capX;
        {
            // Longest straight edge of the cap defines the local X direction.
            double bestLen = -1.0; gp_Vec bestDir(1, 0, 0);
            for (TopExp_Explorer ev(wp.face(cap.id), TopAbs_EDGE); ev.More(); ev.Next()) {
                TopoDS_Vertex v0, v1;
                TopExp::Vertices(TopoDS::Edge(ev.Current()), v0, v1);
                if (v0.IsNull() || v1.IsNull()) continue;
                gp_Vec e(BRep_Tool::Pnt(v0), BRep_Tool::Pnt(v1));
                // project into the cap plane (drop any out-of-plane component)
                e = e - gp_Vec(cap.n) * e.Dot(gp_Vec(cap.n));
                if (e.Magnitude() > bestLen) { bestLen = e.Magnitude(); bestDir = e; }
            }
            if (bestLen < 1e-6) continue;
            bestDir.Normalize();
            capX = gp_Dir(bestDir);
        }
        const gp_Vec vX(capX);
        const gp_Vec vY = gp_Vec(cap.n).Crossed(vX);
        double uMin = 1e30, uMax = -1e30, wMin = 1e30, wMax = -1e30;
        for (TopExp_Explorer vp(wp.face(cap.id), TopAbs_VERTEX); vp.More(); vp.Next()) {
            const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(vp.Current()));
            const gp_Vec rel(cap.c, p);
            const double u = rel.Dot(vX), w = rel.Dot(vY);
            uMin = std::min(uMin, u); uMax = std::max(uMax, u);
            wMin = std::min(wMin, w); wMax = std::max(wMax, w);
        }
        const double length = uMax - uMin;
        const double width  = wMax - wMin;
        if (length < 0.5 || width < 0.5) continue;

        // FOOTPRINT discriminator — a box BOSS has a compact rectangular cap.
        // Reject a thin RIB (extreme aspect) and a broad LAND/face (cap nearly as
        // big as the host cross-section), which would otherwise fire as a boss
        // since box_boss is precise-tier (cap-exempt at conf 0.85).
        const double aspect = std::max(length, width) / std::max(1e-6, std::min(length, width));
        if (aspect > 12.0) continue;                  // a rib / slot land, not a pad
        if (std::min(length, width) < 0.8) continue;  // too sliver to be a boss

        json recovered = {
            { "length_mm",    length },
            { "width_mm",     width },
            { "height_mm",    height },
            { "face_normal",  { cap.n.X(), cap.n.Y(), cap.n.Z() } },
            { "face_xaxis",   { capX.X(), capX.Y(), capX.Z() } },  // in-plane L direction
            // World placement so replay reproduces the boss WITHOUT a planar
            // datum (a lug sits on the curved case side — no face to resolve).
            { "use_world",    true },
            { "world_center", { cap.c.X(), cap.c.Y(), cap.c.Z() } },
        };
        json wallIds = json::array();
        for (int wid : walls) wallIds.push_back(wid);
        json matched = {
            { "source",        "geometry" },
            { "cap_face_id",   cap.id },
            { "wall_face_ids", wallIds },             // counted by collectFaceIds (dedupe)
            { "wall_count",    static_cast<int>(walls.size()) },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.85, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::box_boss
