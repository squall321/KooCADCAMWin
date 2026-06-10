// @lat: [[engine/skills#flange_face_with_gasket_groove]]

#include "flange_face_with_gasket_groove.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::flange_face_with_gasket_groove {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.flange_outer_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "flange_face_with_gasket_groove: outer dia must be > 0");
    }
    if (in.facing_pass_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "facing_pass_depth must be > 0");
    }
    if (in.gasket_groove_id_mm <= 0.0 || in.gasket_groove_od_mm <= 0.0 ||
        in.gasket_groove_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gasket groove dims must be > 0");
    }
    if (in.gasket_groove_od_mm <= in.gasket_groove_id_mm) {
        r.add("DFM-GASKET", "error",
              "gasket_groove_od must be > gasket_groove_id");
    }
    if (in.gasket_groove_od_mm >= in.flange_outer_dia_mm - 6.0) {
        r.add("DFM-GASKET", "error",
              "gasket_groove_od must be ≤ flange_outer_dia - 6 mm "
              "(must leave ≥ 3 mm wall on each side)");
    }
    // ASME B16.20: groove depth ≤ 4 mm typical, ≥ 1.0 mm minimum.
    if (in.gasket_groove_depth_mm < 1.0) {
        r.add("DFM-ASME-B16.20", "error",
              "gasket_groove_depth " + std::to_string(in.gasket_groove_depth_mm) +
              " mm < min 1.0 mm");
    }
    if (in.gasket_groove_depth_mm > 4.0) {
        r.add("DFM-ASME-B16.20", "warning",
              "gasket_groove_depth " + std::to_string(in.gasket_groove_depth_mm) +
              " mm > typical max 4 mm — verify ring-joint spec");
    }
    // Bottom fillet ≥ 0.4 mm.
    if (in.groove_fillet_r_mm < 0.4) {
        r.add("DFM-ASME-B16.20", "error",
              "groove_fillet_r " + std::to_string(in.groove_fillet_r_mm) +
              " mm < min 0.4 mm (sharp-corner stress riser)");
    }
    // Facing pass: 0.2 mm min, 2 mm max.
    if (in.facing_pass_depth_mm < 0.2 || in.facing_pass_depth_mm > 2.0) {
        r.add("DFM-FACING", "warning",
              "facing_pass_depth " + std::to_string(in.facing_pass_depth_mm) +
              " mm outside typical [0.2, 2.0] mm");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sequence:
//   step A: facing pass — cut a thin disc off the top of the flange blank.
//     This is a "single cut" but logically is one sub-feature.  The cut
//     tool is a thin cylinder (flange_outer_dia, facing_pass_depth).
//   step B: groove cut — annularRing tool subtracted from the freshly
//     faced top.
//
// Volume removed (total):
//   V_facing = π · (flange_outer/2)² · facing_pass_depth
//   V_groove = π · ((gg_od/2)² − (gg_id/2)²) · gg_depth
//   V_total  = V_facing + V_groove

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "flange_face_with_gasket_groove DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Dir adir = in.axis_dir;
    if (!(std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6 && adir.Z() < 0)) {
        throw SkillError("flange_face_with_gasket_groove: only axis_dir = -Z supported");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;
    const double topZ = zMax;
    const double kOver = 0.05;

    // ── step A: facing pass (thin disc cut off the top) ─────────────────
    // Tool: cylinder centered at top, slightly larger than flange OD,
    // depth = facing_pass_depth.  Position: tool top above stock top by
    // kOver, tool extends downward by facing_pass_depth + kOver.
    const gp_Ax2 axFacing(
        gp_Pnt(cx, cy, topZ - in.facing_pass_depth_mm), gp_Dir(0, 0, 1));
    const TopoDS_Shape facingTool = pr::cylinder(
        axFacing, in.flange_outer_dia_mm / 2.0 + kOver,
        in.facing_pass_depth_mm + kOver);
    const TopoDS_Shape afterFacing = pr::cut(wp.shape(), facingTool);

    // ── step B: gasket groove (annular ring cut into freshly faced top) ─
    const double newTopZ = topZ - in.facing_pass_depth_mm;
    const gp_Ax2 axGroove(
        gp_Pnt(cx, cy, newTopZ - in.gasket_groove_depth_mm),
        gp_Dir(0, 0, 1));
    const TopoDS_Shape grooveTool = pr::annularRing(
        axGroove,
        in.gasket_groove_od_mm / 2.0,
        in.gasket_groove_id_mm / 2.0,
        in.gasket_groove_depth_mm + kOver);
    const TopoDS_Shape newShape = pr::cut(afterFacing, grooveTool);

    // ── volume bookkeeping ──────────────────────────────────────────────
    const double rOuter = in.flange_outer_dia_mm / 2.0;
    const double rOd    = in.gasket_groove_od_mm / 2.0;
    const double rId    = in.gasket_groove_id_mm / 2.0;
    const double vFacing = M_PI * rOuter * rOuter * in.facing_pass_depth_mm;
    const double vGroove = M_PI * (rOd * rOd - rId * rId) *
                           in.gasket_groove_depth_mm;
    const double vTotal  = vFacing + vGroove;

    // ── signature ───────────────────────────────────────────────────────
    json params = {
        { "center_x_mm",            cx },
        { "center_y_mm",            cy },
        { "axis_dir",               { adir.X(), adir.Y(), adir.Z() } },
        { "flange_outer_dia_mm",    in.flange_outer_dia_mm },
        { "facing_pass_depth_mm",   in.facing_pass_depth_mm },
        { "gasket_groove_id_mm",    in.gasket_groove_id_mm },
        { "gasket_groove_od_mm",    in.gasket_groove_od_mm },
        { "gasket_groove_depth_mm", in.gasket_groove_depth_mm },
        { "groove_fillet_r_mm",     in.groove_fillet_r_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    2 },
        { "concentric_circle_count", 2 },  // groove OD + ID circles
        { "annular_planar_face",     true },
        { "facing_pass_depth_mm", in.facing_pass_depth_mm },
        { "gasket_groove_id_mm",  in.gasket_groove_id_mm },
        { "gasket_groove_od_mm",  in.gasket_groove_od_mm },
        { "gasket_groove_depth_mm", in.gasket_groove_depth_mm },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
        { "standard",            "ASME B16.20" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "face_mill;groove_cutter";
    tooling.tool_dia_mm       = in.flange_outer_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 6;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.1;
    tooling.stock_removed_mm3 = vTotal;
    tooling.est_cycle_time_s  = 45.0;
    tooling.extra = {
        { "facing_volume_mm3",   vFacing },
        { "groove_volume_mm3",   vGroove },
        { "total_removed_mm3",   vTotal },
        { "standard",            "ASME B16.20" },
        { "groove_fillet_r_mm",  in.groove_fillet_r_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::flange_face_with_gasket_groove: facing={} groove ID/OD={}/{}",
                  in.facing_pass_depth_mm,
                  in.gasket_groove_id_mm, in.gasket_groove_od_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric: find a planar top face containing exactly 2 concentric
// circular edges (the groove OD and ID) at the same Z and a third planar
// annular face below it (groove bottom) parallel and offset by gg_depth.

namespace {

struct CircEdge {
    int    edgeIdx = -1;
    double radius  = 0.0;
    gp_Pnt center;
};

std::vector<CircEdge> circlesOnFace(const Workpiece& wp, int faceIdx)
{
    std::vector<CircEdge> out;
    for (TopExp_Explorer exp(wp.face(faceIdx), TopAbs_EDGE); exp.More(); exp.Next()) {
        const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
        BRepAdaptor_Curve crv(e);
        if (crv.GetType() != GeomAbs_Circle) continue;
        gp_Circ c = crv.Circle();
        CircEdge ce;
        ce.radius = c.Radius();
        ce.center = c.Location();
        out.push_back(ce);
    }
    return out;
}

}  // namespace

// Try to extract one recognition candidate from a given top-face index.
// Returns true if a candidate was appended to `out`.
namespace {

bool tryRecognizeOnFace(const Workpiece& wp, int i,
                        std::vector<RecognizedFeature>& out)
{
    const auto circles = circlesOnFace(wp, i);
    const gp_Pnt fc = wp.faceCenter(i);
    for (size_t a = 0; a < circles.size(); ++a) {
        for (size_t b = a + 1; b < circles.size(); ++b) {
            const double dx = circles[a].center.X() - circles[b].center.X();
            const double dy = circles[a].center.Y() - circles[b].center.Y();
            if (std::hypot(dx, dy) > 0.5) continue;
            if (std::abs(circles[a].radius - circles[b].radius) < 0.5) continue;
            const double r1 = std::min(circles[a].radius, circles[b].radius);
            const double r2 = std::max(circles[a].radius, circles[b].radius);
            int annId = -1;
            double annZ = 0.0;
            for (int j = 0; j < wp.faceCount(); ++j) {
                if (j == i) continue;
                if (!wp.isFacePlanar(j)) continue;
                gp_Dir nj;
                try { nj = wp.faceNormal(j); } catch (...) { continue; }
                if (std::abs(nj.Z() - 1.0) > 1e-2) continue;
                const gp_Pnt fcj = wp.faceCenter(j);
                if (fcj.Z() >= fc.Z() - 1e-3) continue;
                const auto cj = circlesOnFace(wp, j);
                bool foundOuter = false, foundInner = false;
                for (const auto& c : cj) {
                    if (std::abs(c.radius - r1) < 0.5) foundInner = true;
                    if (std::abs(c.radius - r2) < 0.5) foundOuter = true;
                }
                if (foundOuter && foundInner) {
                    annId = j;
                    annZ  = fcj.Z();
                    break;
                }
            }
            if (annId < 0) continue;
            const double grooveDepth = fc.Z() - annZ;
            nlohmann::json recovered = {
                { "center_x_mm",            circles[a].center.X() },
                { "center_y_mm",            circles[a].center.Y() },
                { "axis_dir",               { 0.0, 0.0, -1.0 } },
                { "gasket_groove_id_mm",    2.0 * r1 },
                { "gasket_groove_od_mm",    2.0 * r2 },
                { "gasket_groove_depth_mm", grooveDepth },
            };
            nlohmann::json matched = {
                { "top_face_id",   i },
                { "floor_face_id", annId },
                { "via",           "geometric_primary" },
            };
            out.push_back(RecognizedFeature{
                kSkillId, recovered, 0.85, matched });
            return true;
        }
    }
    return false;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n; try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(n.Z() - 1.0) > 1e-2) continue;
        const gp_Pnt fc = wp.faceCenter(i);
        if (std::abs(fc.Z() - zMax) > 5.0) continue;
        tryRecognizeOnFace(wp, i, out);
    }

    // Metadata fallback (only if no geometric match was found).
    if (out.empty()) {
        for (const auto& f : wp.features()) {
            if (f.skill_id != kSkillId) continue;
            RecognizedFeature rf;
            rf.skill_id = kSkillId;
            rf.recovered_params = f.params;
            rf.confidence = 1.0;
            rf.matched_geometry = { { "via", "metadata_replay" } };
            out.push_back(rf);
            break;
        }
    }
    return out;
}

}  // namespace koocadcam::skill::flange_face_with_gasket_groove
