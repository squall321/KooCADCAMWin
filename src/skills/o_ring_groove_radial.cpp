// @lat: [[engine/skills#o_ring_groove_radial]]

#include "o_ring_groove_radial.hpp"

#include "_as568_table.hpp"
#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::o_ring_groove_radial {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── AS568 cross-section table ────────────────────────────────────────────
//
// CS lookup is delegated to the central AS568 table (_as568_table.hpp).
// Only the legacy dash sizes NOT in the central table remain here as
// extensions to preserve this skill's historical accept-set (-014, -110,
// -210, -220, -225, -228, -310).  -325 stays locally because central uses
// CS = 5.34 while this skill historically rounds to 5.33.
namespace {

struct AS568Override {
    const char* dash;
    double      cs_mm;     // cross-section (CS) in mm
};

constexpr std::array<AS568Override, 8> kAS568Extras {{
    { "-014", 1.78 },
    { "-110", 2.62 },
    { "-210", 3.53 },
    { "-220", 3.53 },
    { "-225", 3.53 },
    { "-228", 3.53 },
    { "-310", 5.33 },
    { "-325", 5.33 },
}};

}  // namespace

double crossSectionFor(const std::string& as568_dash)
{
    for (const auto& e : kAS568Extras)
        if (as568_dash == e.dash) return e.cs_mm;
    if (auto* p = as568::findDash(as568_dash)) return p->cross_section_mm;
    return 0.0;
}

double recommendedDepthFor(double cs_mm) { return 0.74 * cs_mm; }
double recommendedWidthFor(double cs_mm) { return 1.45 * cs_mm; }

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.o_ring_size.empty()) {
        r.add("DFM-AS568", "error",
              "o_ring_groove_radial: o_ring_size is empty");
        return r;
    }
    const double cs = crossSectionFor(in.o_ring_size);
    if (cs <= 0.0) {
        r.add("DFM-AS568", "error",
              "o_ring_groove_radial: unknown AS568 size '" + in.o_ring_size +
              "' (supported: -006/-014/-110/-114/-210/-220/-225/-228/-310/-325/-425)");
        return r;
    }
    if (in.bore_or_shaft_dia_mm < 2.0) {
        r.add("DFM-INPUT", "error",
              "o_ring_groove_radial: bore_or_shaft_dia_mm " +
              std::to_string(in.bore_or_shaft_dia_mm) +
              " < 2.0 mm (insufficient material for groove + 1 mm wall)");
    }
    const double depth = recommendedDepthFor(cs);
    const double shaftRadius = in.bore_or_shaft_dia_mm / 2.0;
    if (depth > 0.0 && shaftRadius - depth < 1.0) {
        r.add("DFM-AS568-WALL", "error",
              "o_ring_groove_radial: groove would leave wall < 1.0 mm "
              "(shaftR=" + std::to_string(shaftRadius) +
              ", groove depth=" + std::to_string(depth) + ")");
    }

    // Workpiece Z extent check.
    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double w = recommendedWidthFor(cs);
        const double zLow  = in.position_z_mm - w / 2.0;
        const double zHigh = in.position_z_mm + w / 2.0;
        if (zLow < zMin - 1e-3 || zHigh > zMax + 1e-3) {
            r.add("DFM-INPUT", "error",
                  "o_ring_groove_radial: groove zone outside stock Z extent");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "o_ring_groove_radial DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double cs       = crossSectionFor(in.o_ring_size);
    const double depth_G  = recommendedDepthFor(cs);
    const double width_W  = recommendedWidthFor(cs);
    const double bottomR  = 0.05 * width_W;          // 5%-W bottom fillet
    const double draftAng = 5.0 * M_PI / 180.0;      // 5° sidewall draft

    const double shaftR     = in.bore_or_shaft_dia_mm / 2.0;
    const double grooveR    = shaftR - depth_G;
    const double overhangR  = 0.5;                   // ensure clean Boolean
    const double zLow       = in.position_z_mm - width_W / 2.0;

    // ── Sub-feature 1: MAIN GROOVE (rect cross-section annular ring) ────
    const gp_Ax2 ax1(gp_Pnt(0.0, 0.0, zLow), gp::DZ());
    const TopoDS_Shape mainGroove = pr::annularRing(
        ax1, shaftR + overhangR, grooveR, width_W);

    // ── Sub-feature 2: BOTTOM RADIUS RELIEF ──────────────────────────────
    //
    // Approximate the 5%-W bottom fillet by an additional thin annular
    // ring at the groove bottom: inner_R = grooveR - bottomR, outer_R =
    // grooveR + ε, height = 2·bottomR, centered axially on position_z.
    const double zBotRing = in.position_z_mm - bottomR;
    const gp_Ax2 ax2(gp_Pnt(0.0, 0.0, zBotRing), gp::DZ());
    const TopoDS_Shape bottomFillet = pr::annularRing(
        ax2, grooveR + 1e-3, grooveR - bottomR, 2.0 * bottomR);

    // ── Sub-feature 3: SIDE DRAFT (lead-in chamfer rings) ──────────────
    //
    // Two short cone-frustum cuts at the upper and lower edges of the
    // groove — gives the 5° sidewall draft for O-ring entry.  Each chamfer
    // is a thin annular volume whose outer radius increases with axial
    // distance from groove-bottom plane.  We model each as an annular cone
    // ring of height C_draft and OD growing from groove ID to shaftR.
    const double draftH   = std::min(0.5, width_W * 0.15);
    const double draftRad = draftH * std::tan(draftAng);

    // Upper / lower side-draft chamfer.  Slice-9: original code passed both
    // outer radii equal to (shaftR + overhang), triggering OCCT's
    // "cone with two identic radii" error.  Since outer is constant, this
    // is really a cylindrical annular ring — use annularRing.
    const double zUpDraft = in.position_z_mm + width_W / 2.0 - draftH;
    const gp_Ax2 axUp(gp_Pnt(0.0, 0.0, zUpDraft), gp::DZ());
    const TopoDS_Shape upperDraft = pr::annularRing(
        axUp,
        /*outerR=*/ shaftR + overhangR,
        /*innerR=*/ grooveR + draftRad,
        /*height=*/ draftH);

    const double zDnDraft = in.position_z_mm - width_W / 2.0;
    const gp_Ax2 axDn(gp_Pnt(0.0, 0.0, zDnDraft), gp::DZ());
    const TopoDS_Shape lowerDraft = pr::annularRing(
        axDn,
        /*outerR=*/ shaftR + overhangR,
        /*innerR=*/ grooveR + draftRad,
        /*height=*/ draftH);

    // ── Fuse all sub-feature cutters then single boolean cut ────────────
    //
    // Concentric overlapping cutter pattern (see counterbore.cpp): fuse
    // the cutters into one tool to avoid lost-overlap issues with the
    // BRepAlgoAPI cut algorithm.
    const TopoDS_Shape fused1 = pr::fuse(mainGroove, bottomFillet);
    const TopoDS_Shape fused2 = pr::fuse(fused1, upperDraft);
    const TopoDS_Shape fusedAll = pr::fuse(fused2, lowerDraft);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedAll);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "bore_or_shaft_dia_mm", in.bore_or_shaft_dia_mm },
        { "position_z_mm",        in.position_z_mm },
        { "o_ring_size",          in.o_ring_size },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  3 },
        { "bore_or_shaft_dia_mm", in.bore_or_shaft_dia_mm },
        { "position_z_mm",        in.position_z_mm },
        { "o_ring_size",          in.o_ring_size },
        { "cs_mm",                cs },
        { "groove_depth_mm",      depth_G },
        { "groove_width_mm",      width_W },
        { "groove_radius_mm",     grooveR },
        { "bottom_radius_mm",     bottomR },
        { "draft_angle_deg",      5.0 },
        { "axis",                 { 0.0, 0.0, 1.0 } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "lathe_groove_insert;profile_form_tool";
    tooling.tool_dia_mm       = 0.0;   // form tool — width-controlled
    tooling.tool_length_mm    = depth_G + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 500.0;
    tooling.feed_per_tooth_mm = 0.03;  // slow finish on sealing geometry
    tooling.stock_removed_mm3 = M_PI * (shaftR * shaftR - grooveR * grooveR) * width_W;
    tooling.est_cycle_time_s  = std::max(2.0, depth_G / 5.0);
    tooling.extra = {
        { "subfeature_sequence", {
            { { "kind", "main_rect_groove" },
              { "depth_mm", depth_G }, { "width_mm", width_W } },
            { { "kind", "bottom_radius" },
              { "radius_mm", bottomR } },
            { { "kind", "side_draft" },
              { "angle_deg", 5.0 } },
        } },
        { "standard", "AS568 / Parker O-ring Handbook §4-1" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::o_ring_groove_radial applied: {} cs={} W={} G={} br={} faces {}→{}",
                  in.o_ring_size, cs, width_W, depth_G, bottomR,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + geometric fallback ────────────────────
//
// Geometric signature of a radial O-ring groove on a shaft:
//   - TWO coaxial cylindrical faces sharing the same axis line:
//       * "shaft" cylinder (larger radius = bore_or_shaft_dia/2);
//       * "groove bottom" cylinder (smaller radius = shaft_r − G);
//   - the GROOVE BOTTOM cylinder has narrow axial extent (≈ groove width
//     W) compared to the shaft cylinder which extends past the groove;
//   - axial position of the groove center matches position_z_mm.
// Cone sidewalls (5° draft) are tolerated but not required.

namespace {

struct CylInfo {
    int    faceIdx;
    gp_Ax1 axis;
    double radius;
    double axialMin;
    double axialMax;
    gp_Pnt midPnt;
};

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
            BRepAdaptor_Curve crv(TopoDS::Edge(exp.Current()));
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

std::vector<RecognizedFeature> geometric_fallback(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto cyls = collectCyls(wp);
    if (cyls.size() < 2) return out;

    for (size_t i = 0; i < cyls.size(); ++i) {
        for (size_t j = 0; j < cyls.size(); ++j) {
            if (i == j) continue;
            const CylInfo& shaft  = cyls[i];     // larger radius
            const CylInfo& bottom = cyls[j];     // smaller radius
            if (!axesColinear(shaft.axis, bottom.axis)) continue;
            if (bottom.radius + 0.05 >= shaft.radius) continue;
            const double depth = shaft.radius - bottom.radius;
            if (depth < 0.5 || depth > 8.0) continue;     // AS568 CS range
            const double widthB = bottom.axialMax - bottom.axialMin;
            const double widthS = shaft.axialMax - shaft.axialMin;
            if (widthB < 0.5 || widthB > 12.0) continue;
            if (widthS < widthB * 1.5) continue;          // shaft extends past groove
            // Reject if 1mm wall rule is broken (bottom radius too small).
            if (bottom.radius < 1.0) continue;
            // AS568 nominal: G ≈ 0.74·CS → CS ≈ G / 0.74; pick best dash.
            const double csEst = depth / 0.74;
            std::string bestSize = "-110";
            double bestErr = std::numeric_limits<double>::max();
            for (const auto& e : kAS568Extras) {
                const double err = std::abs(e.cs_mm - csEst);
                if (err < bestErr) { bestErr = err; bestSize = e.dash; }
            }
            for (const auto& e : as568::kAs568) {
                const double err = std::abs(e.cross_section_mm - csEst);
                if (err < bestErr) { bestErr = err; bestSize = e.dash_key; }
            }
            if (bestErr > 1.0) continue;

            const gp_Dir adir = shaft.axis.Direction();
            const gp_Pnt mid  = bottom.midPnt;
            const double position_z =
                std::abs(adir.Z()) > 0.5 ? mid.Z() :
                std::abs(adir.Y()) > 0.5 ? mid.Y() : mid.X();

            json recovered = {
                { "bore_or_shaft_dia_mm", 2.0 * shaft.radius },
                { "position_z_mm",        position_z },
                { "o_ring_size",          bestSize },
            };
            json matched = {
                { "source",               "geometric_fallback" },
                { "shaft_face_id",        shaft.faceIdx },
                { "groove_bottom_face_id", bottom.faceIdx },
                { "groove_depth_mm",      depth },
                { "groove_width_mm",      widthB },
                { "as568_match_err",      bestErr },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
            return out;
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
        r.skill_id = kSkillId;
        r.recovered_params = feat.params;
        r.confidence       = 1.0;     // metadata replay → ground truth
        r.matched_geometry = {
            { "source", "metadata_replay" },
            { "pattern", feat.pattern },
        };
        out.push_back(r);
    }
    if (out.empty()) {
        out = geometric_fallback(wp);
    }
    return out;
}

}  // namespace koocadcam::skill::o_ring_groove_radial
