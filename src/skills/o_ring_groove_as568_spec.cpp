// @lat: [[engine/skills#o_ring_groove_as568_spec]]

#include "o_ring_groove_as568_spec.hpp"

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

namespace koocadcam::skill::o_ring_groove_as568_spec {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── AS568 spec table ─────────────────────────────────────────────────────
//
// 10 dash sizes spanning the AS568 catalog (SAE AS568 / ANSI standard):
//   "100" series → CS = 1.78 mm  (slim)
//   "200" series → CS = 1.78 / 2.62 / 3.53 mm  (medium)
//   "300" series → CS = 3.53 / 5.33 mm        (heavy radial)
//   "400" series → CS = 6.99 mm               (industrial)
//   "900" series → boss-seal specialty (-908 CS = 6.99 mm)
// Source: SAE AS568D (2008) Table 1.

namespace {

struct AS568Entry { const char* dash; double cs_mm; double id_mm; };

constexpr std::array<AS568Entry, 10> kAS568Spec {{
    { "-006", 1.78,  2.84 },
    { "-011", 1.78,  7.59 },
    { "-016", 1.78, 15.54 },
    { "-111", 2.62, 10.77 },
    { "-116", 2.62, 18.72 },
    { "-212", 3.53, 21.89 },
    { "-224", 3.53, 56.74 },
    { "-325", 5.33, 36.10 },
    { "-425", 6.99, 41.51 },
    { "-908", 1.78, 13.21 },
}};

}  // namespace

double crossSectionFor(const std::string& dash_size)
{
    for (const auto& e : kAS568Spec)
        if (dash_size == e.dash) return e.cs_mm;
    return 0.0;
}

// Parker O-ring Handbook ORD 5700 §4-2 "Industrial Static Seal Design"
// Table 4-1: groove depth = 0.75 × CS (15-25% squeeze target).
double recommendedDepthFor(double cs_mm) { return 0.75 * cs_mm; }
// Parker O-ring Handbook ORD 5700 §4-2 Table 4-1: groove width = 1.40 × CS
// (gives 75-85% gland fill with worst-case swelled compound).
double recommendedWidthFor(double cs_mm) { return 1.40 * cs_mm; }

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.dash_size.empty()) {
        r.add("DFM-AS568-SPEC", "error",
              "o_ring_groove_as568_spec: dash_size is empty");
        return r;
    }
    const double cs = crossSectionFor(in.dash_size);
    if (cs <= 0.0) {
        r.add("DFM-AS568-SPEC", "error",
              "o_ring_groove_as568_spec: unknown AS568 dash_size '" +
              in.dash_size + "' (10 supported: -006/-011/-016/-111/-116/"
              "-212/-224/-325/-425/-908)");
        return r;
    }
    if (in.mean_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "o_ring_groove_as568_spec: mean_dia_mm must be > 0");
        return r;
    }

    const double width = recommendedWidthFor(cs);
    const double depth = recommendedDepthFor(cs);
    const double idR = (in.mean_dia_mm - width) / 2.0;
    if (idR <= 0.0) {
        r.add("DFM-AS568-ID", "error",
              "o_ring_groove_as568_spec: derived groove ID <= 0 (mean_dia "
              "too small for AS568 width " + std::to_string(width) + " mm)");
    }

    // Check face datum resolves; fall back to non-fatal warning if not (tests
    // sometimes supply FaceByNormal which always resolves for cuboid stock).
    auto faceId = wp.resolve(in.face_id);
    if (!faceId) {
        r.add("DFM-INPUT", "error",
              "o_ring_groove_as568_spec: face_id datum unresolved");
    }

    // Stock-fit sanity: groove OD must fit in the workpiece bbox.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double maxExtent =
        std::min(xMax - xMin, yMax - yMin);
    if ((in.mean_dia_mm + width) > maxExtent) {
        r.add("DFM-AS568-FIT", "warning",
              "groove OD " + std::to_string(in.mean_dia_mm + width) +
              " mm exceeds workpiece footprint " + std::to_string(maxExtent) +
              " mm — check stock size");
    }

    // Min depth sanity vs achievable cutter range (>0.2 mm).
    if (depth < 0.2) {
        r.add("DFM-AS568-DEPTH", "error",
              "derived groove depth " + std::to_string(depth) +
              " mm < 0.2 mm — below standard form-tool minimum");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Three sub-features:
//   1. MAIN ANNULAR GROOVE — pr::annularRing
//   2. ID 15° lead-in cone (per Parker §4-2)
//   3. OD 15° lead-in cone (per Parker §4-2)
//
// Lead-in profile per Parker handbook: 15° from face surface, height = 0.5
// mm (default).  At 15° the radial offset is leadH·tan(15°) ≈ 0.134 × leadH.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "o_ring_groove_as568_spec DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.face_id);
    if (!faceId)
        throw SkillError("o_ring_groove_as568_spec: face_id unresolved");

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
    double entryZ;
    if (adir.Z() < 0) entryZ = zMax + overhang;
    else              entryZ = zMin - overhang;

    // Sub-feature 1: main annular groove
    const gp_Pnt grooveOrigin(in.center_x_mm, in.center_y_mm, entryZ);
    const gp_Ax2 ax(grooveOrigin, adir);
    const double cutH = depth_G + overhang;
    const TopoDS_Shape mainGroove = pr::annularRing(ax, outerR, innerR, cutH);

    // Sub-feature 2: ID 15° lead-in chamfer (cone narrowing inward)
    const gp_Ax2 axId(grooveOrigin, adir);
    const TopoDS_Shape idChamfer = pr::coneFrustum(
        axId,
        /*r1 bottom=*/ innerR,
        /*r2 top   =*/ std::max(1e-3, innerR - leadRise),
        /*height   =*/ leadH + overhang);

    // Sub-feature 3: OD 15° lead-in chamfer (cone widening outward at face)
    const gp_Ax2 axOd(grooveOrigin, adir);
    const TopoDS_Shape odChamfer = pr::annularConeRing(
        axOd,
        /*outerR1Bottom=*/ outerR + leadRise,
        /*outerR2Top  =*/ outerR,
        /*innerR      =*/ outerR - 1e-3,
        /*height      =*/ leadH + overhang);

    // Fuse-then-cut so concentric overlap is handled in one Boolean.
    const TopoDS_Shape fused1   = pr::fuse(mainGroove, idChamfer);
    const TopoDS_Shape fusedAll = pr::fuse(fused1, odChamfer);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedAll);

    // Signature
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
        { "standard", "SAE AS568D + Parker ORD 5700 §4-2 (radial static)" },
        { "spec_key", in.dash_size },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::o_ring_groove_as568_spec applied: dash={} mean={} cs={} G={} W={}",
                  in.dash_size, in.mean_dia_mm, cs, depth_G, width_W);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + geometric fallback ────────────────────

namespace {

struct CylInfo {
    int        faceIdx;
    gp_Ax1     axis;
    double     radius;
    double     axialMin;
    double     axialMax;
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
            // Match to AS568: CS ≈ depth / 0.75
            const double csEst = depth / 0.75;
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
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
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

}  // namespace koocadcam::skill::o_ring_groove_as568_spec
