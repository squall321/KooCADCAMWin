// @lat: [[engine/skills#cam_with_profile]]

#include "cam_with_profile.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::cam_with_profile {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double minRadius(const std::vector<Waypoint>& p)
{
    if (p.empty()) return 0.0;
    double m = p.front().r_mm;
    for (const auto& w : p) m = std::min(m, w.r_mm);
    return m;
}

double maxRadius(const std::vector<Waypoint>& p)
{
    if (p.empty()) return 0.0;
    double m = p.front().r_mm;
    for (const auto& w : p) m = std::max(m, w.r_mm);
    return m;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────
//
// Standards: DIN 8606 (cam profile machinery), ISO 13399 (tool data for
// profile milling).

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.profile.size() < 8) {
        r.add("DFM-CAM-POINTS", "error",
              "cam_with_profile: waypoint_count " +
              std::to_string(in.profile.size()) +
              " < 8 — too few for a smooth cam profile (DIN 8606)");
    }
    if (in.thickness_mm < 2.0) {
        r.add("DFM-CAM-THICK", "error",
              "cam_with_profile: thickness_mm " +
              std::to_string(in.thickness_mm) + " < 2 mm");
    }
    if (in.shaft_dia_mm < 2.0) {
        r.add("DFM-002", "error",
              "cam_with_profile: shaft_dia_mm " +
              std::to_string(in.shaft_dia_mm) + " < 2 mm");
    }
    const double minR = minRadius(in.profile);
    const double maxR = maxRadius(in.profile);
    if (minR < in.shaft_dia_mm / 2.0 + 2.0) {
        r.add("DFM-CAM-WALL", "error",
              "cam_with_profile: min profile radius " +
              std::to_string(minR) + " < shaft_r + 2mm wall (" +
              std::to_string(in.shaft_dia_mm / 2.0 + 2.0) + ")");
    }
    if (minR > 0.0 && maxR / minR > 5.0) {
        r.add("DFM-CAM-RATIO", "error",
              "cam_with_profile: max/min radius ratio " +
              std::to_string(maxR / minR) +
              " > 5 — extreme eccentricity, unmachinable cusps likely");
    }
    if (in.keyway_width_mm > 0.0) {
        if (in.keyway_width_mm < 0.8) {
            r.add("DFM-002", "error",
                  "cam_with_profile: keyway_width " +
                  std::to_string(in.keyway_width_mm) + " < 0.8 mm");
        }
        if (in.keyway_depth_mm <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "cam_with_profile: keyway_depth must be > 0 when keyway_width > 0");
        }
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cam_with_profile DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double overhang = 0.05;
    const double bottomZ  = in.center_z_mm;

    // ── step 1: build closed wire from waypoint polyline ────────────────
    std::vector<gp_Pnt> verts;
    verts.reserve(in.profile.size());
    for (const auto& w : in.profile) {
        const double px = in.center_x_mm + w.r_mm * std::cos(w.theta_rad);
        const double py = in.center_y_mm + w.r_mm * std::sin(w.theta_rad);
        verts.emplace_back(px, py, bottomZ);
    }

    BRepBuilderAPI_MakeWire wireMk;
    for (size_t i = 0; i < verts.size(); ++i) {
        const gp_Pnt& a = verts[i];
        const gp_Pnt& b = verts[(i + 1) % verts.size()];
        if (a.Distance(b) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(a, b);
        if (!em.IsDone()) throw SkillError("cam_with_profile: edge build failed");
        wireMk.Add(em.Edge());
    }
    if (!wireMk.IsDone()) throw SkillError("cam_with_profile: wire build failed");

    BRepBuilderAPI_MakeFace faceMk(wireMk.Wire(), true);
    if (!faceMk.IsDone()) throw SkillError("cam_with_profile: face build failed");

    // Extrude the profile face along +Z by thickness_mm.
    BRepPrimAPI_MakePrism prism(faceMk.Face(),
                                gp_Vec(0.0, 0.0, in.thickness_mm));
    prism.Build();
    if (!prism.IsDone()) throw SkillError("cam_with_profile: prism build failed");
    const TopoDS_Shape camDisc = prism.Shape();

    // ── step 2: fuse cam disc onto workpiece ────────────────────────────
    const TopoDS_Shape fused = pr::fuse(wp.shape(), camDisc);

    // ── step 3: CUT central shaft bore ──────────────────────────────────
    const gp_Pnt sbOrigin(in.center_x_mm, in.center_y_mm, bottomZ - overhang);
    const gp_Ax2 sbAx(sbOrigin, gp::DZ());
    const TopoDS_Shape shaftTool = pr::cylinder(
        sbAx, in.shaft_dia_mm / 2.0, in.thickness_mm + 2.0 * overhang);

    TopoDS_Shape combined = shaftTool;

    // ── step 4: optional keyway box on +X side of shaft bore ────────────
    if (in.keyway_width_mm > 0.0 && in.keyway_depth_mm > 0.0) {
        const double kwOriginX = in.center_x_mm + in.shaft_dia_mm / 2.0;
        const gp_Pnt kwOrigin(
            kwOriginX,
            in.center_y_mm - in.keyway_width_mm / 2.0,
            bottomZ - overhang);
        const gp_Ax2 kwAx(kwOrigin, gp::DZ());
        const TopoDS_Shape kwTool = pr::box(
            kwAx, in.keyway_depth_mm, in.keyway_width_mm,
            in.thickness_mm + 2.0 * overhang);
        combined = pr::fuse(combined, kwTool);
    }

    const TopoDS_Shape newShape = pr::cut(fused, combined);

    // ── signature ────────────────────────────────────────────────────────
    json wpJson = json::array();
    for (const auto& w : in.profile)
        wpJson.push_back({ { "theta_rad", w.theta_rad }, { "r_mm", w.r_mm } });

    const double minR = minRadius(in.profile);
    const double maxR = maxRadius(in.profile);

    json params = {
        { "waypoint_count",   static_cast<int>(in.profile.size()) },
        { "profile",          wpJson },
        { "thickness_mm",     in.thickness_mm },
        { "shaft_dia_mm",     in.shaft_dia_mm },
        { "keyway_width_mm",  in.keyway_width_mm },
        { "keyway_depth_mm",  in.keyway_depth_mm },
        { "center_x_mm",      in.center_x_mm },
        { "center_y_mm",      in.center_y_mm },
        { "center_z_mm",      in.center_z_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        // disc (prism + fuse) + shaft + optional keyway
        { "subfeature_count",     in.keyway_width_mm > 0.0 ? 3 : 2 },
        { "waypoint_count",       static_cast<int>(in.profile.size()) },
        { "axial_cyl_count",      1 },
        { "min_radius_mm",        minR },
        { "max_radius_mm",        maxR },
        { "shaft_dia_mm",         in.shaft_dia_mm },
        { "thickness_mm",         in.thickness_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "profile_mill;drill";
    tooling.tool_dia_mm       = in.shaft_dia_mm;
    tooling.tool_length_mm    = in.thickness_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double rS = in.shaft_dia_mm / 2.0;
    const double vShaft = M_PI * rS * rS * in.thickness_mm;
    tooling.stock_removed_mm3 = vShaft;
    tooling.est_cycle_time_s  = std::max(5.0,
        in.profile.size() * 0.3 + in.thickness_mm * 0.3);
    tooling.extra = {
        { "profile_perimeter_segments", static_cast<int>(in.profile.size()) },
        { "tool_sequence", {
            { { "tool_type", "profile_mill" },
              { "tool_dia_mm", in.thickness_mm * 0.4 },
              { "note",        "outer cam profile" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", in.shaft_dia_mm },
              { "depth_mm",    in.thickness_mm },
              { "note",        "central shaft bore" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::cam_with_profile applied: n={} t={} shaft={} faces {}→{}",
                  in.profile.size(), in.thickness_mm, in.shaft_dia_mm,
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
        r.matched_geometry = { { "source", "feature_history" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // geometric: scan for an axial cylinder (shaft) and assume the outer
    // boundary is a polyline-extruded prism.
    int shaftIdx = -1;
    double shaftR = 0.0;
    gp_Pnt shaftLoc;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cy = surf.Cylinder();
        if (std::abs(std::abs(cy.Axis().Direction().Z()) - 1.0) > 1e-2) continue;
        // Pick the SMALLEST axial cyl as the shaft (cam outer profile may
        // also have arcs but they wouldn't be a single full cylinder).
        if (shaftIdx < 0 || cy.Radius() < shaftR) {
            shaftIdx = i;
            shaftR   = cy.Radius();
            shaftLoc = cy.Axis().Location();
        }
    }
    if (shaftIdx < 0) return out;

    // Count planar vertical faces — those are the cam profile side faces.
    int verticalPlanarCount = 0;
    double zMin = 1e30, zMax = -1e30;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        const gp_Pnt c = wp.faceCenter(i);
        if (c.Z() < zMin) zMin = c.Z();
        if (c.Z() > zMax) zMax = c.Z();
        if (std::abs(n.Z()) < 1e-2) ++verticalPlanarCount;
    }
    if (verticalPlanarCount < 8) return out;       // not enough sides for a cam

    // Confidence: more profile sides → higher confidence.
    double conf = 0.55 + std::min(0.3, verticalPlanarCount * 0.01);

    json recovered = {
        { "waypoint_count",   verticalPlanarCount },
        { "profile",          json::array() },     // can't recover (θ, r) without depth analysis
        { "thickness_mm",     (zMax > zMin) ? (zMax - zMin) : 0.0 },
        { "shaft_dia_mm",     2.0 * shaftR },
        { "keyway_width_mm",  0.0 },
        { "keyway_depth_mm",  0.0 },
        { "center_x_mm",      shaftLoc.X() },
        { "center_y_mm",      shaftLoc.Y() },
        { "center_z_mm",      zMin },
    };
    json matched = {
        { "shaft_face_id",         shaftIdx },
        { "profile_face_count",    verticalPlanarCount },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    return out;
}

}  // namespace koocadcam::skill::cam_with_profile
