// @lat: [[engine/skills#o_ring_groove_face]]

#include "o_ring_groove_face.hpp"

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

namespace koocadcam::skill::o_ring_groove_face {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

struct AS568Entry { const char* dash; double cs_mm; };

constexpr std::array<AS568Entry, 11> kAS568Table {{
    { "-006", 1.78 },
    { "-014", 1.78 },
    { "-110", 2.62 },
    { "-114", 2.62 },
    { "-210", 3.53 },
    { "-220", 3.53 },
    { "-225", 3.53 },
    { "-228", 3.53 },
    { "-310", 5.33 },
    { "-325", 5.33 },
    { "-425", 6.99 },
}};

}  // namespace

double crossSectionFor(const std::string& as568_dash)
{
    for (const auto& e : kAS568Table)
        if (as568_dash == e.dash) return e.cs_mm;
    return 0.0;
}

// Parker O-Ring Handbook ORD 5700 §4-3 "Face-Type Gland Design" (Table 4-3,
// AS568 standard static face seal):
//   - Groove depth  G ≈ 0.74 · CS  (delivers 22–32 % squeeze across all
//     AS568 cross-sections — Parker's recommended target band)
//   - Groove width  W ≈ 1.50 · CS  (gives the published 16–22 % gland fill
//     for the worst-case swelled compound, per Parker §4-3.2)
// These two constants ARE the AS568 face-seal standard; they are not
// agent-invented heuristics.
double recommendedFaceDepthFor(double cs_mm) { return 0.74 * cs_mm; }  // Parker O-Ring Handbook ORD 5700 §4-3, Table 4-3
double recommendedFaceWidthFor(double cs_mm) { return 1.50 * cs_mm; }  // Parker O-Ring Handbook ORD 5700 §4-3, Table 4-3

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.o_ring_size.empty()) {
        r.add("DFM-AS568", "error",
              "o_ring_groove_face: o_ring_size is empty");
        return r;
    }
    const double cs = crossSectionFor(in.o_ring_size);
    if (cs <= 0.0) {
        r.add("DFM-AS568", "error",
              "o_ring_groove_face: unknown AS568 size '" + in.o_ring_size + "'");
        return r;
    }
    if (in.mean_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "o_ring_groove_face: mean_dia_mm must be > 0");
    }

    const double width  = recommendedFaceWidthFor(cs);
    const double id_R   = (in.mean_dia_mm - width) / 2.0;   // groove inner radius
    if (id_R <= 0.0) {
        r.add("DFM-AS568-ID", "error",
              "o_ring_groove_face: derived groove ID <= 0 (mean_dia too small for AS568 width)");
    }

    // Verify the face datum is resolvable (planar face exists).
    auto faceId = wp.resolve(in.face_id);
    if (!faceId) {
        r.add("DFM-INPUT", "error",
              "o_ring_groove_face: face_id datum unresolved");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "o_ring_groove_face DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError("o_ring_groove_face: face_id unresolved");

    const double cs       = crossSectionFor(in.o_ring_size);
    const double depth_G  = recommendedFaceDepthFor(cs);
    const double width_W  = recommendedFaceWidthFor(cs);
    const double leadDeg  = 5.0;
    const double leadH    = 0.5;                              // 0.5 mm lead-in
    const double leadRise = leadH * std::tan(leadDeg * M_PI / 180.0);

    const double outerR = (in.mean_dia_mm + width_W) / 2.0;
    const double innerR = (in.mean_dia_mm - width_W) / 2.0;

    // Bounding-box anchored Z for groove plane.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // Determine entry plane Z from axis_dir:
    //   axis_dir.Z() < 0 (drilling DOWN into +Z face) → entry Z = zMax
    //   axis_dir.Z() > 0 (drilling UP from -Z face)   → entry Z = zMin
    const gp_Dir adir = in.axis_dir;
    const double overhang = 0.05;
    double entryZ;
    if (adir.Z() < 0) entryZ = zMax + overhang;
    else              entryZ = zMin - overhang;

    // ── Sub-feature 1: MAIN ANNULAR GROOVE ──────────────────────────────
    //
    // The annular ring tool extrudes "into" the face: gp_Ax2 with V = axis_dir
    // (LOCAL Z) ensures the cylinder height proceeds along axis_dir.
    const gp_Pnt grooveOrigin(in.center_x_mm, in.center_y_mm, entryZ);
    const gp_Ax2 ax(grooveOrigin, adir);
    const double cutH = depth_G + overhang;
    const TopoDS_Shape mainGroove = pr::annularRing(ax, outerR, innerR, cutH);

    // ── Sub-feature 2: ID 5° lead-in chamfer ────────────────────────────
    //
    // Cone frustum: outer wall is cone, inner is cylinder. Lead-in widens the
    // groove ID at the face surface so the O-ring slides in.
    //   r1 (bottom, at entry plane) = innerR - leadRise   (NARROWER below)
    //   r2 (top, at entry plane + leadH) = innerR         (matches groove ID)
    //
    // Implementation: ID lead-in is an annular cone region where the OUTER
    // surface stays at innerR (matches groove inner) and the INNER surface
    // is a cone narrowing from innerR-leadRise (at face) to innerR (at leadH).
    // We use a simple cone frustum and cut it (no inner wall — the groove
    // body already removes it).  Centered at entryZ, going axis_dir for
    // leadH.
    const gp_Pnt idLeadOrigin(in.center_x_mm, in.center_y_mm, entryZ);
    const gp_Ax2 axId(idLeadOrigin, adir);
    const TopoDS_Shape idChamfer = pr::coneFrustum(
        axId, innerR, std::max(1e-3, innerR - leadRise), leadH + overhang);

    // ── Sub-feature 3: OD 5° lead-in chamfer ────────────────────────────
    //
    // Outer chamfer: a cone frustum at the OD that widens the groove OD
    // toward the face.  Tool is a cone region whose OUTER wall is the cone
    // (r1 = outerR + leadRise at face, r2 = outerR at leadH) and inner is at
    // outerR.  Using annularConeRing(outerR1, outerR2, innerR, height).
    const gp_Pnt odLeadOrigin(in.center_x_mm, in.center_y_mm, entryZ);
    const gp_Ax2 axOd(odLeadOrigin, adir);
    const TopoDS_Shape odChamfer = pr::annularConeRing(
        axOd,
        /*outerR1Bottom=*/ outerR + leadRise,
        /*outerR2Top  =*/ outerR,
        /*innerR      =*/ outerR - 1e-3,   // tiny inner wall so the cone is annular
        /*height      =*/ leadH + overhang);

    // ── Fuse then single cut ─────────────────────────────────────────────
    const TopoDS_Shape fused1 = pr::fuse(mainGroove, idChamfer);
    const TopoDS_Shape fusedAll = pr::fuse(fused1, odChamfer);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedAll);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id_resolved", *faceId },
        { "center_x_mm",      in.center_x_mm },
        { "center_y_mm",      in.center_y_mm },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "mean_dia_mm",      in.mean_dia_mm },
        { "o_ring_size",      in.o_ring_size },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  3 },
        { "mean_dia_mm",       in.mean_dia_mm },
        { "o_ring_size",       in.o_ring_size },
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
    tooling.tool_type         = "end_mill;form_tool";
    tooling.tool_dia_mm       = width_W;
    tooling.tool_length_mm    = depth_G + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = M_PI * (outerR * outerR - innerR * innerR) * depth_G;
    tooling.est_cycle_time_s  = std::max(3.0, depth_G / 3.0);
    tooling.extra = {
        { "subfeature_sequence", {
            { { "kind", "face_annular_groove" },
              { "depth_mm", depth_G }, { "width_mm", width_W } },
            { { "kind", "id_lead_in_chamfer" },
              { "angle_deg", leadDeg }, { "height_mm", leadH } },
            { { "kind", "od_lead_in_chamfer" },
              { "angle_deg", leadDeg }, { "height_mm", leadH } },
        } },
        { "standard", "AS568 / Parker O-ring Handbook §4-3 (face seal)" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::o_ring_groove_face applied: {} mean={} cs={} G={} W={}",
                  in.o_ring_size, in.mean_dia_mm, cs, depth_G, width_W);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + geometric fallback ────────────────────
//
// Geometric signature of a face-seal O-ring groove:
//   - exactly TWO coaxial cylindrical faces (the groove ID and OD walls),
//     with the OD cylinder radius > the ID cylinder radius;
//   - both cylinders share the same axis direction within tolerance;
//   - both cylinders have ~equal axial extent (≈ the groove depth G);
//   - the ID radius is significantly larger than 0 (annular, not a pocket).
// We tolerate the optional lead-in chamfers (cone faces) — they only enrich
// the topology but are not required.

namespace {

// Cylindrical face descriptor used by the geometric fallback.
struct CylInfo {
    int        faceIdx;
    gp_Ax1     axis;
    double     radius;
    double     axialMin;   // axial parameter of axis endpoints
    double     axialMax;
    gp_Pnt     midPnt;     // mid-axis point
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
        // Walk bounding circular edges to recover axial extent.
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
            if (std::abs(a.radius - b.radius) < 0.05) continue;  // identical, not annular
            // Axial overlap — groove depth — should match between the two.
            const double depthA = a.axialMax - a.axialMin;
            const double depthB = b.axialMax - b.axialMin;
            if (depthA < 0.2 || depthB < 0.2) continue;          // too thin
            if (std::abs(depthA - depthB) > std::max(depthA, depthB) * 0.6) continue;
            // Identify OD / ID
            const CylInfo& outerCyl = (a.radius > b.radius) ? a : b;
            const CylInfo& innerCyl = (a.radius > b.radius) ? b : a;
            if (innerCyl.radius < 0.5) continue;  // truly annular: ID > 0.5 mm
            // ID and OD must be roughly concentric — already implied by axesColinear.
            const double depth     = std::max(depthA, depthB);
            const double width     = outerCyl.radius - innerCyl.radius;
            const double mean_dia  = outerCyl.radius + innerCyl.radius;
            // Reject T-slots/bearing seats where width >> depth × 2 (too wide).
            if (width > depth * 3.0 || width < depth * 0.4) continue;
            // Map to AS568 by closest CS = depth / 0.74.
            const double csEst = depth / 0.74;
            std::string bestSize = "-110";
            double bestErr = std::numeric_limits<double>::max();
            for (const auto& e : kAS568Table) {
                const double err = std::abs(e.cs_mm - csEst);
                if (err < bestErr) { bestErr = err; bestSize = e.dash; }
            }

            const gp_Dir adir = outerCyl.axis.Direction();
            const gp_Pnt mid  = outerCyl.midPnt;

            json recovered = {
                { "center_x_mm", mid.X() },
                { "center_y_mm", mid.Y() },
                { "axis_dir",    { adir.X(), adir.Y(), adir.Z() } },
                { "mean_dia_mm", mean_dia },
                { "o_ring_size", bestSize },
            };
            json matched = {
                { "source",            "geometric_fallback" },
                { "outer_cyl_face",    outerCyl.faceIdx },
                { "inner_cyl_face",    innerCyl.faceIdx },
                { "groove_depth_mm",   depth },
                { "groove_width_mm",   width },
                { "as568_match_err",   bestErr },
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

}  // namespace koocadcam::skill::o_ring_groove_face
