// @lat: [[engine/skills#bolted_flange_compound]]

#include "bolted_flange_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::bolted_flange_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.flange_outer_dia_mm <= 0.0 || in.flange_thickness_mm <= 0.0 ||
        in.pipe_bore_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "bolted_flange_compound: outer_dia / thickness / bore_dia must be > 0");
    }

    if (in.pipe_bore_dia_mm >= in.flange_outer_dia_mm) {
        r.add("DFM-FLANGE-GEOM", "error",
              "pipe_bore_dia must be < flange_outer_dia (else there is no disc body)");
    }

    // ASME B16.5 — allowable bolt counts (4, 8, 12, 16).  4 and 8 are
    // standard for NPS ≤ 24; 12/16 for very large flanges.
    if (in.bolt_count != 4 && in.bolt_count != 8 &&
        in.bolt_count != 12 && in.bolt_count != 16) {
        r.add("DFM-ASME-B16.5", "error",
              "bolt_count " + std::to_string(in.bolt_count) +
              " not in ASME B16.5 standard set {4, 8, 12, 16}");
    }

    // Bolt circle dimensions: bolt holes must fit inside (PCD + dia) < outer_dia
    // and outside the raised face: PCD - dia > raised_face_dia.
    const double pcd = in.bolt_circle_dia_mm;
    if (pcd <= 0.0) {
        r.add("DFM-INPUT", "error", "bolt_circle_dia_mm must be > 0");
    }
    if (pcd + in.bolt_hole_dia_mm + 6.0 > in.flange_outer_dia_mm) {
        r.add("DFM-FLANGE-PCD", "error",
              "bolt-circle + hole + 3 mm rim < flange outer dia required "
              "(insufficient outer wall)");
    }
    if (pcd - in.bolt_hole_dia_mm <= in.raised_face_dia_mm + 4.0) {
        r.add("DFM-FLANGE-PCD", "error",
              "bolt-circle - hole must be > raised_face_dia + 4 mm (bolt heads "
              "would foul on raised face)");
    }

    // Raised face geom.
    if (in.raised_face_dia_mm <= in.pipe_bore_dia_mm) {
        r.add("DFM-RF-GEOM", "error",
              "raised_face_dia must be > pipe_bore_dia");
    }
    if (in.raised_face_height_mm < 1.6) {
        r.add("DFM-ASME-B16.5", "warning",
              "raised_face_height " + std::to_string(in.raised_face_height_mm) +
              " mm < ASME B16.5 recommended 1.6 mm (1/16 in)");
    }

    // Gasket groove (B16.20).
    if (in.gasket_groove_id_mm <= in.pipe_bore_dia_mm) {
        r.add("DFM-GASKET", "error",
              "gasket_groove_id must be > pipe_bore_dia");
    }
    if (in.gasket_groove_od_mm >= in.raised_face_dia_mm) {
        r.add("DFM-GASKET", "error",
              "gasket_groove_od must be < raised_face_dia (groove must sit on RF)");
    }
    if (in.gasket_groove_od_mm <= in.gasket_groove_id_mm) {
        r.add("DFM-GASKET", "error",
              "gasket_groove_od must be > gasket_groove_id (annular groove)");
    }
    if (in.gasket_groove_depth_mm > in.raised_face_height_mm) {
        r.add("DFM-GASKET", "error",
              "gasket_groove_depth must be ≤ raised_face_height");
    }

    if (in.bolt_hole_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "bolt_hole_dia " + std::to_string(in.bolt_hole_dia_mm) +
              " mm < min 0.8 mm");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Build sequence (REAL multi-cut compound):
//   step A: fuse disc cylinder onto wp.shape()
//   step B: fuse raised-face cylinder onto top of disc
//   step C: subtract central bore (through-hole)
//   step D: subtract gasket groove (annular ring on raised face)
//   step E: subtract N bolt holes through the disc
//
// Volume delta math (consumed FROM the new fused body):
//   V_central_bore   = π · (bore/2)²  · (flange_thickness + raised_face_height)
//   V_gasket_groove  = π · ((gg_od/2)² − (gg_id/2)²) · gg_depth
//   V_bolt_holes     = N · π · (bolt_dia/2)² · flange_thickness
//
// Volume ADDED (vs original input wp):
//   V_disc           = π · (outer/2)² · flange_thickness
//   V_raised_face    = π · (rf/2)² · raised_face_height
//
//   net delta = (disc + rf) - (bore + groove + bolts)  (positive = grew)

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bolted_flange_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Dir adir = in.axis_dir;
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // Disc origin: place flange disc on top of stock along +axis, base at zMax
    // for the canonical +Z config.  For arbitrary axis_dir, use the entry
    // face centroid.  Slice 1: handle +Z explicitly; arbitrary axes left as
    // canonical-axis-only for now.
    if (!(std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6 &&
          adir.Z() > 0)) {
        throw SkillError("bolted_flange_compound: only axis_dir = +Z is supported");
    }

    const double baseZ = zMax;    // top of stock = bottom of flange disc

    // ── A) disc cylinder ─────────────────────────────────────────────────
    const gp_Ax2 ax(gp_Pnt(cx, cy, baseZ), adir);
    const TopoDS_Shape discTool = pr::cylinder(
        ax, in.flange_outer_dia_mm / 2.0, in.flange_thickness_mm);
    const TopoDS_Shape afterDisc = pr::fuse(wp.shape(), discTool);

    // ── B) raised face cylinder (centred on disc top) ────────────────────
    const gp_Ax2 axRF(gp_Pnt(cx, cy, baseZ + in.flange_thickness_mm), adir);
    const TopoDS_Shape rfTool = pr::cylinder(
        axRF, in.raised_face_dia_mm / 2.0, in.raised_face_height_mm);
    const TopoDS_Shape afterRF = pr::fuse(afterDisc, rfTool);

    // ── C) central bore through full stack (disc + RF) + overhang ────────
    const double totalH = in.flange_thickness_mm + in.raised_face_height_mm;
    const double kOver  = 0.1;
    const gp_Ax2 axBoreDown(
        gp_Pnt(cx, cy, baseZ - kOver), adir);
    const TopoDS_Shape boreTool = pr::cylinder(
        axBoreDown, in.pipe_bore_dia_mm / 2.0, totalH + 2.0 * kOver);
    const TopoDS_Shape afterBore = pr::cut(afterRF, boreTool);

    // ── D) gasket groove on raised face top ─────────────────────────────
    // groove sits on RF top, cut INTO raised face by gg_depth.
    const double rfTopZ = baseZ + in.flange_thickness_mm + in.raised_face_height_mm;
    const gp_Ax2 axGroove(
        gp_Pnt(cx, cy, rfTopZ - in.gasket_groove_depth_mm), adir);
    const TopoDS_Shape grooveTool = pr::annularRing(
        axGroove,
        in.gasket_groove_od_mm / 2.0,
        in.gasket_groove_id_mm / 2.0,
        in.gasket_groove_depth_mm + kOver);
    const TopoDS_Shape afterGroove = pr::cut(afterBore, grooveTool);

    // ── E) N bolt holes evenly spaced on bolt circle, through disc ───────
    const double pcdR = in.bolt_circle_dia_mm / 2.0;
    std::vector<TopoDS_Shape> bolts;
    bolts.reserve(static_cast<size_t>(in.bolt_count));
    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        const double bx = cx + pcdR * std::cos(theta);
        const double by = cy + pcdR * std::sin(theta);
        const gp_Ax2 axBolt(gp_Pnt(bx, by, baseZ - kOver), adir);
        bolts.push_back(pr::cylinder(
            axBolt, in.bolt_hole_dia_mm / 2.0, in.flange_thickness_mm + 2.0 * kOver));
    }
    const TopoDS_Shape newShape = pr::cutMany(afterGroove, bolts);

    // ── signature ────────────────────────────────────────────────────────
    json bolts_json = json::array();
    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        bolts_json.push_back({
            { "x", cx + pcdR * std::cos(theta) },
            { "y", cy + pcdR * std::sin(theta) },
            { "dia_mm", in.bolt_hole_dia_mm },
        });
    }

    const int subfeatureCount = 4 + in.bolt_count;   // disc, RF, bore, groove, +N bolts

    json params = {
        { "center_x_mm",          cx },
        { "center_y_mm",          cy },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
        { "flange_outer_dia_mm",  in.flange_outer_dia_mm },
        { "flange_thickness_mm",  in.flange_thickness_mm },
        { "pipe_bore_dia_mm",     in.pipe_bore_dia_mm },
        { "raised_face_dia_mm",   in.raised_face_dia_mm },
        { "raised_face_height_mm",in.raised_face_height_mm },
        { "gasket_groove_id_mm",  in.gasket_groove_id_mm },
        { "gasket_groove_od_mm",  in.gasket_groove_od_mm },
        { "gasket_groove_depth_mm", in.gasket_groove_depth_mm },
        { "bolt_count",           in.bolt_count },
        { "bolt_circle_dia_mm",   in.bolt_circle_dia_mm },
        { "bolt_hole_dia_mm",     in.bolt_hole_dia_mm },
    };

    json pattern = {
        { "kind",               kSkillId },
        { "is_compound",        true },
        { "subfeature_count",   subfeatureCount },
        { "disc_outer_dia_mm",  in.flange_outer_dia_mm },
        { "raised_face_dia_mm", in.raised_face_dia_mm },
        { "bore_dia_mm",        in.pipe_bore_dia_mm },
        { "gasket_groove_id_mm",in.gasket_groove_id_mm },
        { "gasket_groove_od_mm",in.gasket_groove_od_mm },
        { "bolt_count",         in.bolt_count },
        { "bolt_circle_dia_mm", in.bolt_circle_dia_mm },
        { "bolt_positions",     bolts_json },
        { "axis_dir",           { adir.X(), adir.Y(), adir.Z() } },
        { "standard",           "ASME B16.5" },
    };

    ToolingMeta tooling;
    tooling.tool_type = "lathe;drill;groove_cutter";
    tooling.tool_dia_mm = std::max(in.bolt_hole_dia_mm, in.flange_outer_dia_mm);
    tooling.tool_material = "carbide";
    tooling.flute_count = 4;
    tooling.cutting_speed_sfm = 250.0;
    const double rBore = in.pipe_bore_dia_mm / 2.0;
    const double rGGO  = in.gasket_groove_od_mm / 2.0;
    const double rGGI  = in.gasket_groove_id_mm / 2.0;
    const double rBolt = in.bolt_hole_dia_mm / 2.0;
    const double rDisc = in.flange_outer_dia_mm / 2.0;
    const double rRF   = in.raised_face_dia_mm / 2.0;
    const double vAdded = M_PI * rDisc * rDisc * in.flange_thickness_mm
                        + M_PI * rRF   * rRF   * in.raised_face_height_mm;
    const double vRemoved = M_PI * rBore * rBore * totalH
                          + M_PI * (rGGO * rGGO - rGGI * rGGI) * in.gasket_groove_depth_mm
                          + static_cast<double>(in.bolt_count) *
                            M_PI * rBolt * rBolt * in.flange_thickness_mm;
    tooling.stock_removed_mm3 = std::max(0.0, vRemoved);
    tooling.est_cycle_time_s  = 60.0 + 6.0 * static_cast<double>(in.bolt_count);
    tooling.extra = {
        { "added_volume_mm3",    vAdded },
        { "removed_volume_mm3",  vRemoved },
        { "net_delta_mm3",       vAdded - vRemoved },
        { "standard",            "ASME B16.5 / B16.20" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::bolted_flange_compound: outer={} bolts={} pcd={} faces {}→{}",
                  in.flange_outer_dia_mm, in.bolt_count, in.bolt_circle_dia_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric primary: find a cluster of N (≥ 4) coaxial cylindrical faces
// arranged on a bolt-circle around a larger central cylindrical face.

namespace {

struct CylRec {
    int    faceIdx = -1;
    double radius = 0.0;
    gp_Ax1 axis;
};

std::vector<CylRec> collectCylinders(const Workpiece& wp)
{
    std::vector<CylRec> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder c = s.Cylinder();
        out.push_back({ i, c.Radius(), c.Axis() });
    }
    return out;
}

bool parallelAxis(const gp_Ax1& a, const gp_Ax1& b)
{
    const double dot = std::abs(a.Direction().Dot(b.Direction()));
    return dot > 0.999;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay first — highest confidence.  Other path may also
    // produce candidates; this gives a guaranteed exact answer when history
    // is available.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence = 1.0;
        rf.matched_geometry = { { "via", "metadata_replay" } };
        out.push_back(rf);
    }

    const auto cyls = collectCylinders(wp);
    if (cyls.size() < 5) return out;  // central bore + ≥4 bolt holes minimum

    // Find a candidate "central bore": small radius cylinder near the
    // workpiece centroid, with many parallel-axis cylinders around it of
    // smaller radius.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double midX = 0.5 * (xMin + xMax);
    const double midY = 0.5 * (yMin + yMax);

    for (const auto& central : cyls) {
        // Centre near workpiece centre and axis ~ +Z.
        if (std::abs(central.axis.Direction().Z()) < 0.9) continue;
        if (std::abs(central.axis.Location().X() - midX) > 5.0) continue;
        if (std::abs(central.axis.Location().Y() - midY) > 5.0) continue;

        // Collect bolt cylinders: smaller radius, axis parallel, axis NOT
        // centred (offset > 5 mm from central).
        std::vector<const CylRec*> boltCands;
        for (const auto& c : cyls) {
            if (c.faceIdx == central.faceIdx) continue;
            if (!parallelAxis(c.axis, central.axis)) continue;
            if (c.radius >= central.radius) continue;
            const double dx = c.axis.Location().X() - central.axis.Location().X();
            const double dy = c.axis.Location().Y() - central.axis.Location().Y();
            const double r  = std::hypot(dx, dy);
            if (r < 5.0) continue;          // too close — could be groove edge
            boltCands.push_back(&c);
        }

        if (boltCands.size() < 4) continue;

        // Group bolts by radial offset (within ±2 mm = on the same PCD).
        std::vector<double> rs;
        for (auto* b : boltCands) {
            const double dx = b->axis.Location().X() - central.axis.Location().X();
            const double dy = b->axis.Location().Y() - central.axis.Location().Y();
            rs.push_back(std::hypot(dx, dy));
        }
        std::sort(rs.begin(), rs.end());
        // Histogram into 2 mm buckets and pick the largest bucket.
        std::vector<std::pair<double, int>> hist;
        for (double r : rs) {
            bool placed = false;
            for (auto& [c, cnt] : hist) {
                if (std::abs(c - r) < 2.0) {
                    c = (c * static_cast<double>(cnt) + r) /
                        static_cast<double>(cnt + 1);
                    ++cnt;
                    placed = true;
                    break;
                }
            }
            if (!placed) hist.push_back({ r, 1 });
        }
        std::sort(hist.begin(), hist.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });
        if (hist.empty() || hist.front().second < 4) continue;

        const double pcdR = hist.front().first;
        const int boltCount = hist.front().second;

        // Compute average bolt-hole radius for diameter recovery.
        double sumR = 0.0; int cnt = 0;
        for (auto* b : boltCands) {
            const double dx = b->axis.Location().X() - central.axis.Location().X();
            const double dy = b->axis.Location().Y() - central.axis.Location().Y();
            const double r  = std::hypot(dx, dy);
            if (std::abs(r - pcdR) < 2.0) { sumR += b->radius; ++cnt; }
        }
        if (cnt == 0) continue;
        const double boltDia = 2.0 * (sumR / static_cast<double>(cnt));

        // Confidence — geometric primary path.
        double conf = 0.85;
        if (boltCount < 4 || boltCount > 16) conf -= 0.2;

        json recovered = {
            { "center_x_mm",          central.axis.Location().X() },
            { "center_y_mm",          central.axis.Location().Y() },
            { "axis_dir",             { 0.0, 0.0, 1.0 } },
            { "pipe_bore_dia_mm",     2.0 * central.radius },
            { "bolt_count",           boltCount },
            { "bolt_circle_dia_mm",   2.0 * pcdR },
            { "bolt_hole_dia_mm",     boltDia },
        };
        json matched = {
            { "central_face_id", central.faceIdx },
            { "bolt_cylinder_count", boltCount },
            { "via",            "geometric_primary" },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    }

    return out;
}

}  // namespace koocadcam::skill::bolted_flange_compound
