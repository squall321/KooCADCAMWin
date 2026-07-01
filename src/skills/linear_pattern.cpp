// @lat: [[engine/skills#linear_pattern]]

#include "linear_pattern.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::linear_pattern {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.count_x < 1) {
        r.add("DFM-INPUT", "error", "linear_pattern: count_x must be >= 1");
    }
    if (in.count_y < 1) {
        r.add("DFM-INPUT", "error", "linear_pattern: count_y must be >= 1");
    }
    if (in.hole_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "linear_pattern: hole_dia_mm must be > 0");
    }
    if (in.hole_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "linear_pattern: hole_depth_mm must be > 0");
    }
    // DFM-002 — minimum drill diameter (ISO 235:2016 HSS twist-drill floor).
    if (in.hole_dia_mm > 0.0 && in.hole_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "linear_pattern: hole_dia_mm " + std::to_string(in.hole_dia_mm) +
              " < 0.8 mm (ISO 235:2016 standard HSS drill floor)");
    }
    // DFM-PATT-1 — pitch must clear the hole diameter to avoid overlap.
    if (in.count_x > 1 && in.pitch_x_mm <= in.hole_dia_mm) {
        r.add("DFM-PATT-1", "error",
              "linear_pattern: pitch_x_mm " + std::to_string(in.pitch_x_mm) +
              " <= hole_dia_mm " + std::to_string(in.hole_dia_mm) +
              " (instances would overlap along X)");
    }
    if (in.count_y > 1 && in.pitch_y_mm <= in.hole_dia_mm) {
        r.add("DFM-PATT-1", "error",
              "linear_pattern: pitch_y_mm " + std::to_string(in.pitch_y_mm) +
              " <= hole_dia_mm " + std::to_string(in.hole_dia_mm) +
              " (instances would overlap along Y)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "linear_pattern DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Placement source: either a resolved face datum (design path) or a direct
    // world origin+normal (foreign-recovery path — a side grille has no design
    // datum).  Both feed the same face-local (u,v) grid below.
    int faceId = -1;
    gp_Pnt center;
    gp_Dir normal;
    if (in.use_world) {
        center = gp_Pnt(in.world_ox_mm, in.world_oy_mm, in.world_oz_mm);
        normal = gp_Dir(in.world_nx, in.world_ny, in.world_nz);
    } else {
        auto faceIdOpt = wp.resolve(in.face);
        if (!faceIdOpt) throw SkillError("linear_pattern: face datum unresolved");
        faceId = *faceIdOpt;
        center = wp.faceCenter(faceId);
        normal = wp.faceNormal(faceId);
    }

    // Tool length must overshoot the depth so the cut is clean at the entry.
    constexpr double kOverhang = 0.1;
    const double toolLen = in.hole_depth_mm + 2.0 * kOverhang;
    const double r = in.hole_dia_mm * 0.5;

    // Cut axis direction = INTO the face (opposite outward normal).
    const gp_Dir cutDir(-normal.X(), -normal.Y(), -normal.Z());

    // The (start_x/pitch_x, start_y/pitch_y) grid is laid out in the FACE'S OWN
    // in-plane frame (u along X-of-grid, v along Y-of-grid) — NOT world XY — so a
    // grid on a SIDE face (normal along +/-X or +/-Y, e.g. a watch speaker
    // grille) places correctly.  For a top face (normal +Z) u=+X and v=+Y, which
    // reproduces the legacy world-XY behaviour exactly (byte-parity).
    gp_Vec seed = std::abs(normal.X()) < 0.9 ? gp_Vec(1, 0, 0) : gp_Vec(0, 1, 0);
    seed = seed - gp_Vec(normal) * seed.Dot(gp_Vec(normal));   // project into face plane
    const gp_Dir uDir(seed);
    const gp_Vec vVec = gp_Vec(normal).Crossed(gp_Vec(uDir));  // right-handed in-plane
    const gp_Dir vDir(vVec);

    const int total = in.count_x * in.count_y;
    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(static_cast<size_t>(total));
    json entryCenters = json::array();   // 3-D entry point of every hole (for CAM)

    // The face center is on the entry plane.  Start each cylinder slightly
    // outside the face along +normal, then sweep INTO the body for toolLen.
    for (int j = 0; j < in.count_y; ++j) {
        for (int i = 0; i < in.count_x; ++i) {
            const double du = in.start_x_mm + i * in.pitch_x_mm;
            const double dv = in.start_y_mm + j * in.pitch_y_mm;
            const gp_Pnt entryOnPlane(
                center.X() + uDir.X() * du + vDir.X() * dv,
                center.Y() + uDir.Y() * du + vDir.Y() * dv,
                center.Z() + uDir.Z() * du + vDir.Z() * dv);
            entryCenters.push_back({ entryOnPlane.X(), entryOnPlane.Y(), entryOnPlane.Z() });
            // Lift slightly along +normal to start the cylinder above the face.
            const gp_Pnt cylStart(
                entryOnPlane.X() + normal.X() * kOverhang,
                entryOnPlane.Y() + normal.Y() * kOverhang,
                entryOnPlane.Z() + normal.Z() * kOverhang);
            const gp_Ax2 ax(cylStart, cutDir);
            cutters.push_back(pr::cylinder(ax, r, toolLen));
        }
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    const double perVol = M_PI * r * r * in.hole_depth_mm;
    const double totalVol = perVol * total;

    json params = {
        { "face_id",        faceId },
        { "hole_dia_mm",    in.hole_dia_mm },
        { "hole_depth_mm",  in.hole_depth_mm },
        { "count_x",        in.count_x },
        { "pitch_x_mm",     in.pitch_x_mm },
        { "count_y",        in.count_y },
        { "pitch_y_mm",     in.pitch_y_mm },
        { "start_x_mm",     in.start_x_mm },
        { "start_y_mm",     in.start_y_mm },
        { "hole_centers",   entryCenters },                       // 3-D entry points (CAM)
        { "face_normal",    { normal.X(), normal.Y(), normal.Z() } },  // for the CAM axis guard
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_pattern",           true },
        { "instance_count",       total },
        { "derived_params", {
            { "hole_dia_mm",      in.hole_dia_mm },
            { "hole_depth_mm",    in.hole_depth_mm },
            { "pitch_x_mm",       in.pitch_x_mm },
            { "pitch_y_mm",       in.pitch_y_mm },
            { "count_x",          in.count_x },
            { "count_y",          in.count_y },
            { "per_inst_vol_mm3", perVol },
            { "total_vol_mm3",    totalVol },
        } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill";
    tooling.tool_dia_mm       = in.hole_dia_mm;
    tooling.tool_length_mm    = in.hole_depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = totalVol;
    tooling.est_cycle_time_s  = std::max(1.0, in.hole_depth_mm / 50.0) * total;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::linear_pattern applied: {}x{} dia={} depth={} vol={}",
                  in.count_x, in.count_y, in.hole_dia_mm,
                  in.hole_depth_mm, totalVol);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Heuristic: group all cylindrical faces by (axis-direction, radius).  If
// any group of ≥2 has cylinders whose XY centers lie on a colinear set with
// a constant pitch, mark it a linear pattern candidate.

namespace {

struct CylInst
{
    double x, y, z;
    double radius;
    double depth = 0.0;   // axial extent of the cylinder face (recovered hole depth)
    gp_Dir dir;
};

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 0.95;
        r.matched_geometry = f.pattern;
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // ── Geometric path B (foreign geometry) ──────────────────────────────
    // A linear pattern is a regular GRID of identical, coaxial cylinders.  The
    // old fallback fired on ANY two same-radius cylinders and flattened them to
    // a 1-D count — so an unrelated pair of holes was mis-read as a pattern and
    // a 2-D grid was mangled.  We instead REQUIRE a real equally-spaced grid:
    // the largest set of same-radius / same-axis cylinders whose centres fall on
    // a 1-D (or 2-D) lattice with a consistent pitch.  At least 3 instances and
    // < 5 % pitch variation are needed, so two coincidental holes never qualify.
    std::vector<CylInst> cyls;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            CylInst ci{};
            ci.x = c.Location().X();
            ci.y = c.Location().Y();
            ci.z = c.Location().Z();
            ci.radius = c.Radius();
            ci.dir = c.Axis().Direction();
            // Depth = the cylinder face's extent ALONG its own axis (project the
            // face bbox corners onto the axis).  Recovering depth is essential:
            // apply()/validate reject hole_depth_mm<=0, so a pattern replayed with
            // a zero depth would ABORT the whole re-synthesis plan.
            Bnd_Box fb; BRepBndLib::Add(wp.face(i), fb);
            if (!fb.IsVoid()) {
                double q0, q1, q2, q3, q4, q5; fb.Get(q0, q1, q2, q3, q4, q5);
                const double qx[2] = { q0, q3 }, qy[2] = { q1, q4 }, qz[2] = { q2, q5 };
                double lo = 1e30, hi = -1e30;
                const gp_Pnt L = c.Location();
                for (int ix = 0; ix < 2; ++ix)
                    for (int iy = 0; iy < 2; ++iy)
                        for (int iz = 0; iz < 2; ++iz) {
                            const double a = gp_Vec(L, gp_Pnt(qx[ix], qy[iy], qz[iz]))
                                                 .Dot(gp_Vec(ci.dir));
                            lo = std::min(lo, a); hi = std::max(hi, a);
                        }
                ci.depth = hi - lo;
            }
            cyls.push_back(ci);
        } catch (...) { continue; }
    }
    if (cyls.size() < 3) return out;

    // Group by (radius, axis direction): only co-radial, co-axial cylinders can
    // belong to one pattern.
    auto sameGroup = [](const CylInst& a, const CylInst& b) {
        return std::abs(a.radius - b.radius) < 1e-2 &&
               std::abs(std::abs(a.dir.Dot(b.dir)) - 1.0) < 1e-2;
    };
    std::vector<bool> used(cyls.size(), false);
    size_t bestCount = 0;
    json bestRecovered, bestMatched;

    for (size_t g = 0; g < cyls.size(); ++g) {
        if (used[g]) continue;
        std::vector<CylInst> grp;
        for (size_t k = 0; k < cyls.size(); ++k)
            if (!used[k] && sameGroup(cyls[g], cyls[k])) grp.push_back(cyls[k]);
        for (size_t k = 0; k < cyls.size(); ++k)
            if (!used[k] && sameGroup(cyls[g], cyls[k])) used[k] = true;
        if (grp.size() < 3) continue;

        // The hole axis is grp[0].dir (up to sign).  apply() cuts along -normal
        // (INTO the body), so the OUTWARD normal points OUT of the body.  A
        // cylinder axis has an arbitrary OCCT sign, so orient the normal to point
        // AWAY from the solid's bbox centre (the entry face is on the outside).
        gp_Dir axis = grp[0].dir;
        double wxMin, wyMin, wzMin, wxMax, wyMax, wzMax;
        wp.boundingBox(wxMin, wyMin, wzMin, wxMax, wyMax, wzMax);
        const gp_Pnt bodyC(0.5 * (wxMin + wxMax), 0.5 * (wyMin + wyMax),
                           0.5 * (wzMin + wzMax));
        const gp_Vec cToHole(bodyC, gp_Pnt(grp[0].x, grp[0].y, grp[0].z));
        gp_Dir normal = axis;
        if (gp_Vec(axis).Dot(cToHole) < 0.0)       // axis points inward → flip
            normal = gp_Dir(-axis.X(), -axis.Y(), -axis.Z());
        if (std::abs(normal.Z()) > 0.99 && normal.Z() < 0.0)
            normal = gp_Dir(0, 0, 1);             // +/-Z parity: prefer +Z
        gp_Vec seed = std::abs(normal.X()) < 0.9 ? gp_Vec(1, 0, 0) : gp_Vec(0, 1, 0);
        seed = seed - gp_Vec(normal) * seed.Dot(gp_Vec(normal));
        const gp_Dir uDir(seed);
        const gp_Dir vDir(gp_Vec(normal).Crossed(gp_Vec(uDir)));

        // Project every centre into (u, v) relative to the group's first centre.
        const gp_Pnt o(grp[0].x, grp[0].y, grp[0].z);
        struct UV { double u, v; const CylInst* c; };
        std::vector<UV> pts; pts.reserve(grp.size());
        for (const auto& c : grp) {
            const gp_Vec d(o, gp_Pnt(c.x, c.y, c.z));
            pts.push_back({ d.Dot(gp_Vec(uDir)), d.Dot(gp_Vec(vDir)), &c });
        }
        double minU = pts[0].u, maxU = pts[0].u, minV = pts[0].v, maxV = pts[0].v;
        for (const auto& p : pts) {
            minU = std::min(minU, p.u); maxU = std::max(maxU, p.u);
            minV = std::min(minV, p.v); maxV = std::max(maxV, p.v);
        }
        const double spanU = maxU - minU, spanV = maxV - minV;
        const bool uDominant = spanU >= spanV;

        (void)uDominant;
        if (spanU < grp[0].radius && spanV < grp[0].radius) continue;  // degenerate

        // GRID detection (1-D row OR 2-D RECTANGULAR grid): cluster the (u,v)
        // coords along each axis into equally-spaced "tracks".  A speaker grille
        // is a 2-D rows x cols grid; the old code only accepted a 1-D collinear
        // line (it rejected any off-axis spread), so a 2x6 grille never recovered.
        // LIMITATION: only ALIGNED rectangular grids recover — a half-pitch
        // brick/HEX-staggered grille (common on phones) yields pts != cX*cY and is
        // dropped below.  A staggered-lattice model is a follow-up.
        // clusterAxis returns the distinct coordinate values (track centres) if
        // they are equally spaced within tolerance, else empty.
        auto clusterAxis = [&](bool useU) -> std::vector<double> {
            std::vector<double> vals;
            for (const auto& p : pts) vals.push_back(useU ? p.u : p.v);
            std::sort(vals.begin(), vals.end());
            std::vector<double> tracks;
            std::vector<int> trackN;    // holes accumulated per track (running mean)
            const double tol = std::max(0.05, grp[0].radius);  // same-track band
            for (double v : vals) {
                if (tracks.empty() || std::abs(v - tracks.back()) > tol) {
                    tracks.push_back(v); trackN.push_back(1);
                } else {
                    // true running mean, not a pairwise EWMA (which would bias the
                    // centre toward later points when a track holds >2 holes).
                    const int n = trackN.back();
                    tracks.back() = (tracks.back() * n + v) / (n + 1);
                    trackN.back() = n + 1;
                }
            }
            if (tracks.size() >= 2) {   // check equal spacing between tracks
                std::vector<double> g;
                for (size_t k = 1; k < tracks.size(); ++k) g.push_back(tracks[k] - tracks[k-1]);
                const double mg = std::accumulate(g.begin(), g.end(), 0.0) / g.size();
                if (mg < 2.0 * grp[0].radius) return {};          // overlapping tracks
                double dev = 0.0; for (double x : g) dev = std::max(dev, std::abs(x - mg));
                if (mg <= 0.0 || dev / mg > 0.05) return {};      // not equally spaced
            }
            return tracks;
        };
        const std::vector<double> uTracks = clusterAxis(true);
        const std::vector<double> vTracks = clusterAxis(false);
        if (uTracks.empty() || vTracks.empty()) continue;
        const int cX = static_cast<int>(uTracks.size());
        const int cY = static_cast<int>(vTracks.size());
        // Grid completeness: every (u-track, v-track) cell must hold a hole, so
        // the count matches cX*cY exactly (an incomplete/blob set is not a grid).
        if (static_cast<int>(pts.size()) != cX * cY) continue;
        // Pattern floor: at least one axis must have >= 3 tracks.  A bare 2x2
        // (cX==cY==2) is too weak — it also matches the four corner fillets of a
        // rectangular pocket, so it would false-fire on machined parts.  A real
        // grille/hole-array has a run of >= 3 along at least one axis (a 1-D row
        // is cX>=4, cY==1; a grid is >=2 x >=3).  Total must also be >= 4.
        if (cX * cY < 4) continue;
        if (std::max(cX, cY) < 3) continue;

        const double pitchX = cX > 1 ? (uTracks.back() - uTracks.front()) / (cX - 1) : 0.0;
        const double pitchY = cY > 1 ? (vTracks.back() - vTracks.front()) / (cY - 1) : 0.0;

        // SIDE-AXIS SIZE gate: recognising ANY-axis grids (not just +/-Z) newly
        // exposes functional SIDE bore rows — cross-drilled oil galleries, dowel-
        // pin rows, hinge-knuckle bores — that are geometrically identical to a
        // grille but are NOT one editable pattern.  A real side GRILLE (speaker /
        // mic / vent) uses SMALL holes (<= ~3 mm); a structural side bore row uses
        // larger holes.  So on a side (non-+/-Z) axis require small holes; +/-Z
        // grids (bolt/mounting grids) keep the legacy behaviour (no size cap).
        const bool verticalAxis = std::abs(normal.Z()) > 0.99;
        if (!verticalAxis && 2.0 * grp[0].radius > 3.0) continue;
        // A 2-D +Z grid of LARGE holes is a structural bolt/dowel array, not a
        // hole GRILLE — unifying it into one pattern would erase per-hole
        // tolerance identity.  The legacy un-capped path only ever accepted 1-D
        // +Z rows; extend a size ceiling to the NEW 2-D +Z case (a 1-D row stays
        // un-capped, and small mounting-hole grids <=6mm stay editable).
        if (verticalAxis && cX > 1 && cY > 1 && 2.0 * grp[0].radius > 6.0) continue;

        // A recovered hole must carry a positive depth (apply/validate reject
        // depth<=0).  A cylinder face always has axial extent, but guard anyway.
        if (grp[0].depth <= 0.0) continue;

        if (pts.size() <= bestCount) continue;
        bestCount = pts.size();

        // Recovered params are face-local (u,v) offsets FROM THE GROUP ORIGIN o;
        // the pattern axis/origin carry the 3-D placement so apply() replays it
        // in-plane.  hole_centers keeps the 3-D world centres for subsumption
        // (the dedupe matches on those, independent of the local frame).
        json centers = json::array();
        for (const auto& p : pts) centers.push_back({ p.c->x, p.c->y, p.c->z });
        // start = the first track corner's (u,v) offset from the group origin o.
        const double startU = uTracks.front();
        const double startV = vTracks.front();
        bestRecovered = {
            { "hole_dia_mm",   2.0 * grp[0].radius },
            { "hole_depth_mm", grp[0].depth },       // recovered from cyl axial length
            { "count_x",       cX },
            { "pitch_x_mm",    pitchX },
            { "count_y",       cY },
            { "pitch_y_mm",    pitchY },
            { "start_x_mm",    startU },
            { "start_y_mm",    startV },
            // Foreign recovery has no design face datum — carry the 3-D grid
            // origin `o` (the group's first cylinder centre) + outward normal so
            // apply() replays the grid in place.  start_x/start_y is the sorted
            // front hole's (u,v) offset FROM o, so apply's world_o + start lands
            // on the real first hole regardless of cylinder iteration order.
            { "use_world",     true },
            { "world_nx",      normal.X() },
            { "world_ny",      normal.Y() },
            { "world_nz",      normal.Z() },
            { "world_ox_mm",   o.X() },
            { "world_oy_mm",   o.Y() },
            { "world_oz_mm",   o.Z() },
            { "hole_centers",  centers },     // 3-D world centres for subsumption
        };
        bestMatched = {
            { "cyl_count",      static_cast<int>(pts.size()) },
            { "first_radius",   grp[0].radius },
            { "count_x",        cX },
            { "count_y",        cY },
            { "pitch_x_mm",     pitchX },
            { "pitch_y_mm",     pitchY },
            { "axis_z",         normal.Z() },
        };
    }

    // Emit only at >= 4 instances: with 3 holes the pitch-consistency gate has
    // just 2 gaps and is near-vacuous, so a chance trio of equally-spaced
    // same-radius holes would pass.  Four instances force 3 consistent gaps —
    // strong enough to keep full (precise-tier) confidence and then subsume.
    if (bestCount >= 4)
        out.push_back(RecognizedFeature{
            kSkillId, bestRecovered, /*confidence*/ 0.7, bestMatched });
    return out;
}

}  // namespace koocadcam::skill::linear_pattern
