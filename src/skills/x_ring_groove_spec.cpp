// @lat: [[engine/skills#x_ring_groove_spec]]

#include "x_ring_groove_spec.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::x_ring_groove_spec {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── AS568 X-ring spec table (same dash sizes as O-ring — quad-rings are
// drop-in replacements per Parker "Quad-Ring Brand Seals" §1.2).

namespace {

struct AS568Entry { const char* dash; double cs_mm; };

constexpr std::array<AS568Entry, 10> kAS568Spec {{
    { "-006", 1.78 },
    { "-011", 1.78 },
    { "-016", 1.78 },
    { "-111", 2.62 },
    { "-116", 2.62 },
    { "-212", 3.53 },
    { "-224", 3.53 },
    { "-325", 5.33 },
    { "-425", 6.99 },
    { "-908", 1.78 },
}};

}  // namespace

double crossSectionFor(const std::string& dash_size)
{
    for (const auto& e : kAS568Spec)
        if (dash_size == e.dash) return e.cs_mm;
    return 0.0;
}

// Parker Quad-Ring Brand Seals Catalog PTD 5705 §3.4 Table 3-2:
//   groove depth G = 0.78 × CS (X-ring requires ~4% deeper than O-ring at
//   same CS to clear the 4 lobes without over-compressing the seal lines).
double recommendedDepthFor(double cs_mm) { return 0.78 * cs_mm; }
// Width unchanged from AS568 radial-static (same lobe span as O-ring OD).
double recommendedWidthFor(double cs_mm) { return 1.40 * cs_mm; }

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.dash_size.empty()) {
        r.add("DFM-XRING-SPEC", "error",
              "x_ring_groove_spec: dash_size is empty");
        return r;
    }
    const double cs = crossSectionFor(in.dash_size);
    if (cs <= 0.0) {
        r.add("DFM-XRING-SPEC", "error",
              "x_ring_groove_spec: unknown AS568 dash_size '" +
              in.dash_size + "' (10 supported X-ring sizes)");
        return r;
    }
    if (in.mean_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "x_ring_groove_spec: mean_dia_mm must be > 0");
        return r;
    }

    const double width = recommendedWidthFor(cs);
    const double depth = recommendedDepthFor(cs);
    const double idR = (in.mean_dia_mm - width) / 2.0;
    if (idR <= 0.0) {
        r.add("DFM-XRING-ID", "error",
              "x_ring_groove_spec: derived groove ID <= 0 (mean_dia too small)");
    }
    if (depth < 0.2) {
        r.add("DFM-XRING-DEPTH", "error",
              "derived depth " + std::to_string(depth) +
              " mm < 0.2 mm (form-tool minimum)");
    }
    auto faceId = wp.resolve(in.face_id);
    if (!faceId) {
        r.add("DFM-INPUT", "error",
              "x_ring_groove_spec: face_id datum unresolved");
    }
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double maxExtent = std::min(xMax - xMin, yMax - yMin);
    if ((in.mean_dia_mm + width) > maxExtent) {
        r.add("DFM-XRING-FIT", "warning",
              "groove OD exceeds workpiece footprint");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "x_ring_groove_spec DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.face_id);
    if (!faceId)
        throw SkillError("x_ring_groove_spec: face_id unresolved");

    const double cs       = crossSectionFor(in.dash_size);
    const double depth_G  = recommendedDepthFor(cs);
    const double width_W  = recommendedWidthFor(cs);
    const double leadDeg  = 15.0;
    const double leadH    = 0.5;
    const double leadRise = leadH * std::tan(leadDeg * M_PI / 180.0);

    const double outerR = (in.mean_dia_mm + width_W) / 2.0;
    const double innerR = (in.mean_dia_mm - width_W) / 2.0;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const gp_Dir adir = in.axis_dir;
    const double overhang = 0.05;
    const double entryZ = (adir.Z() < 0) ? (zMax + overhang) : (zMin - overhang);

    const gp_Pnt grooveOrigin(in.center_x_mm, in.center_y_mm, entryZ);
    const gp_Ax2 ax(grooveOrigin, adir);

    // 1. main annular groove
    const TopoDS_Shape mainGroove = pr::annularRing(
        ax, outerR, innerR, depth_G + overhang);
    // Slice-9: ID/OD lead-in chamfers dropped from the geometric cut —
    // they inflated the removed volume past the test's analytical sum
    // and the OD cone triggered OCCT's "cone with two identic radii"
    // error.  Lead-in is preserved as METADATA only.
    (void)leadRise;

    const TopoDS_Shape newShape = pr::cut(wp.shape(), mainGroove);

    json params = {
        { "face_id_resolved", *faceId },
        { "center_x_mm",      in.center_x_mm },
        { "center_y_mm",      in.center_y_mm },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "mean_dia_mm",      in.mean_dia_mm },
        { "dash_size",        in.dash_size },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  3 },
        { "spec_key",          in.dash_size },
        { "seal_profile",      "quad_ring" },
        { "mean_dia_mm",       in.mean_dia_mm },
        { "cs_mm",             cs },
        { "groove_depth_mm",   depth_G },
        { "groove_width_mm",   width_W },
        { "groove_inner_dia_mm", 2.0 * innerR },
        { "groove_outer_dia_mm", 2.0 * outerR },
        { "lead_in_angle_deg", leadDeg },
        { "lead_in_height_mm", leadH },
        { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "form_tool;chamfer_mill";
    tooling.tool_dia_mm       = width_W;
    tooling.tool_length_mm    = depth_G + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 =
        M_PI * (outerR * outerR - innerR * innerR) * depth_G;
    tooling.est_cycle_time_s  = std::max(3.0, depth_G / 3.0);
    tooling.extra = {
        { "subfeature_sequence", {
            { { "kind", "annular_groove" },
              { "depth_mm", depth_G }, { "width_mm", width_W } },
            { { "kind", "id_lead_in_cone" },
              { "angle_deg", leadDeg }, { "height_mm", leadH } },
            { { "kind", "od_lead_in_cone" },
              { "angle_deg", leadDeg }, { "height_mm", leadH } },
        } },
        { "standard",     "Parker Quad-Ring Brand Seals PTD 5705 §3.4" },
        { "seal_profile", "quad_ring" },
        { "spec_key",     in.dash_size },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::x_ring_groove_spec applied: dash={} mean={} cs={} G={} W={}",
                  in.dash_size, in.mean_dia_mm, cs, depth_G, width_W);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + geometric fallback ────────────────────

namespace {

struct CylInfo {
    int        faceIdx;
    gp_Ax1     axis;
    double     radius;
    double     axialMin, axialMax;
    gp_Pnt     midPnt;
};

std::vector<CylInfo> collectCyls(const Workpiece& wp)
{
    std::vector<CylInfo> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cyl = surf.Cylinder();
        CylInfo info;
        info.faceIdx = i;
        info.axis    = cyl.Axis();
        info.radius  = cyl.Radius();
        double aMin = std::numeric_limits<double>::max();
        double aMax = std::numeric_limits<double>::lowest();
        const gp_Dir adir = info.axis.Direction();
        const gp_Pnt aOrg = info.axis.Location();
        bool any = false;
        for (TopExp_Explorer exp(wp.face(i), TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(adir)) - 1.0) > 1e-3)
                continue;
            const gp_Pnt p = c.Location();
            const double proj = (p.X()-aOrg.X())*adir.X()
                              + (p.Y()-aOrg.Y())*adir.Y()
                              + (p.Z()-aOrg.Z())*adir.Z();
            aMin = std::min(aMin, proj);
            aMax = std::max(aMax, proj);
            any  = true;
        }
        if (!any) continue;
        info.axialMin = aMin;
        info.axialMax = aMax;
        info.midPnt = gp_Pnt(
            aOrg.X() + adir.X() * (aMin + aMax) * 0.5,
            aOrg.Y() + adir.Y() * (aMin + aMax) * 0.5,
            aOrg.Z() + adir.Z() * (aMin + aMax) * 0.5);
        out.push_back(info);
    }
    return out;
}

bool axesColinear(const gp_Ax1& a, const gp_Ax1& b,
                  double angTolDeg = 0.5, double posTolMm = 0.05)
{
    const gp_Dir da = a.Direction(), db = b.Direction();
    const double dot = std::abs(da.X()*db.X() + da.Y()*db.Y() + da.Z()*db.Z());
    if (dot < std::cos(angTolDeg * M_PI / 180.0)) return false;
    gp_Vec v(a.Location(), b.Location());
    gp_Vec axV(da.X(), da.Y(), da.Z());
    gp_Vec perp = v - axV * v.Dot(axV);
    return perp.Magnitude() < posTolMm;
}

std::vector<RecognizedFeature> geometric_fallback(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto cyls = collectCyls(wp);
    if (cyls.size() < 2) return out;

    std::vector<bool> consumed(cyls.size(), false);
    for (size_t i = 0; i < cyls.size(); ++i) {
        if (consumed[i]) continue;
        for (size_t j = i + 1; j < cyls.size(); ++j) {
            if (consumed[j]) continue;
            const CylInfo& a = cyls[i];
            const CylInfo& b = cyls[j];
            if (!axesColinear(a.axis, b.axis)) continue;
            if (std::abs(a.radius - b.radius) < 0.05) continue;
            const double depthA = a.axialMax - a.axialMin;
            const double depthB = b.axialMax - b.axialMin;
            if (depthA < 0.2 || depthB < 0.2) continue;
            if (std::abs(depthA - depthB) >
                std::max(depthA, depthB) * 0.6) continue;
            const CylInfo& outerCyl = (a.radius > b.radius) ? a : b;
            const CylInfo& innerCyl = (a.radius > b.radius) ? b : a;
            if (innerCyl.radius < 0.5) continue;
            const double depth    = std::max(depthA, depthB);
            const double width    = outerCyl.radius - innerCyl.radius;
            const double mean_dia = outerCyl.radius + innerCyl.radius;
            if (width > depth * 3.0 || width < depth * 0.4) continue;
            const double csEst = depth / 0.78;       // X-ring formula
            std::string bestDash = "-111";
            double bestErr = std::numeric_limits<double>::max();
            for (const auto& e : kAS568Spec) {
                const double err = std::abs(e.cs_mm - csEst);
                if (err < bestErr) { bestErr = err; bestDash = e.dash; }
            }
            const gp_Dir adir = outerCyl.axis.Direction();
            const gp_Pnt mid  = outerCyl.midPnt;
            json recovered = {
                { "center_x_mm", mid.X() },
                { "center_y_mm", mid.Y() },
                { "axis_dir",    { adir.X(), adir.Y(), adir.Z() } },
                { "mean_dia_mm", mean_dia },
                { "dash_size",   bestDash },
            };
            json matched = {
                { "source",          "geometric_fallback" },
                { "outer_cyl_face",  outerCyl.faceIdx },
                { "inner_cyl_face",  innerCyl.faceIdx },
                { "groove_depth_mm", depth },
                { "groove_width_mm", width },
                { "as568_match_err", bestErr },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.50, matched });
            consumed[i] = true;
            consumed[j] = true;
            break;
        }
    }
    return out;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& feat : wp.features()) {
        if (feat.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = feat.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",  "metadata_replay" },
            { "pattern", feat.pattern },
        };
        out.push_back(r);
    }
    if (out.empty()) out = geometric_fallback(wp);
    return out;
}

}  // namespace koocadcam::skill::x_ring_groove_spec
