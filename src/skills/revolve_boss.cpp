// @lat: [[engine/skills#revolve_boss]]

#include "revolve_boss.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRep_Tool.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::revolve_boss {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Signed XZ-area of a 2D polyline (treat .first = x = r, .second = y = z).
double polygonAreaRZ(const std::vector<std::pair<double,double>>& poly)
{
    if (poly.size() < 3) return 0.0;
    double a = 0.0;
    const int n = static_cast<int>(poly.size());
    for (int i = 0; i < n; ++i) {
        const auto& A = poly[i];
        const auto& B = poly[(i + 1) % n];
        a += A.first * B.second - B.first * A.second;
    }
    return std::abs(a) * 0.5;
}

// Pappus' theorem centroid (r-coord) for the XZ profile polygon.
double polygonCentroidR(const std::vector<std::pair<double,double>>& poly)
{
    if (poly.size() < 3) return 0.0;
    double cx = 0.0, area = 0.0;
    const int n = static_cast<int>(poly.size());
    for (int i = 0; i < n; ++i) {
        const auto& A = poly[i];
        const auto& B = poly[(i + 1) % n];
        const double cross = A.first * B.second - B.first * A.second;
        area += cross;
        cx += (A.first + B.first) * cross;
    }
    area *= 0.5;
    if (std::abs(area) < 1e-12) return 0.0;
    return cx / (6.0 * area);
}

double minRofProfile(const std::vector<std::pair<double,double>>& poly)
{
    if (poly.empty()) return 0.0;
    double m = std::numeric_limits<double>::infinity();
    for (const auto& p : poly) m = std::min(m, p.first);
    return m;
}

double maxRofProfile(const std::vector<std::pair<double,double>>& poly)
{
    if (poly.empty()) return 0.0;
    double m = -std::numeric_limits<double>::infinity();
    for (const auto& p : poly) m = std::max(m, p.first);
    return m;
}

double zRangeOfProfile(const std::vector<std::pair<double,double>>& poly)
{
    if (poly.empty()) return 0.0;
    double zMin = 1e30, zMax = -1e30;
    for (const auto& p : poly) { zMin = std::min(zMin, p.second);
                                 zMax = std::max(zMax, p.second); }
    return zMax - zMin;
}

// A right-handed frame for an arbitrary revolution axis: the axis direction is
// the local Z (the profile's z runs along it), and a chosen perpendicular is
// the local X (the profile's r runs along it).  For the ±Z special case this
// reduces to the world XZ plane.
struct RevolveFrame {
    gp_Pnt origin;
    gp_Dir zAxis;   // axis direction (profile z)
    gp_Dir xAxis;   // radial direction (profile r)
};
RevolveFrame makeRevolveFrame(const gp_Pnt& origin, const gp_Dir& axisDir)
{
    // ±Z special case: reproduce the LEGACY placement EXACTLY.  The old
    // makeProfileWireXZ put the profile at world (r, 0, z) — z along world +Z —
    // regardless of whether the revolve axis was +Z or -Z (a 360° revolve about
    // either gives the same axisymmetric solid).  So the frame's zAxis must be
    // world +Z for BOTH ±Z axes; using axisDir directly would map (r,z)→(r,0,-z)
    // for a -Z axis and MIRROR the solid across Z=0 (a real regression, since
    // the struct/Executor default axis_dir is -Z).  xAxis = +X matches legacy.
    //
    // The fast-path gate must be EXACTLY ±Z: a loose |Z|>0.999 admits axes up to
    // ~2.56° off ±Z, then forces xAxis=(1,0,0) which is NOT perpendicular to such
    // an axis — the (xAxis,zAxis) frame is sheared and makeProfileWire measures r
    // along a direction with an axial component.  Off-axis (any tilt) falls
    // through to the cross-product branch, which builds a true perpendicular X.
    const double inPlaneSq = axisDir.X() * axisDir.X() + axisDir.Y() * axisDir.Y();
    if (inPlaneSq < 1e-18)   // axisDir is exactly ±Z
        return RevolveFrame{ origin, gp_Dir(0, 0, 1), gp_Dir(1, 0, 0) };
    const gp_Vec x = gp_Vec(0, 0, 1).Crossed(gp_Vec(axisDir));  // perpendicular to axis
    return RevolveFrame{ origin, axisDir, gp_Dir(x) };
}

// Build the closed wire of the (r,z) profile polygon placed in the given
// revolution frame: world point = origin + r*xAxis + z*zAxis.  For a ±Z axis
// with origin at 0 and xAxis = +X this is the legacy world XZ plane (y = 0).
TopoDS_Wire makeProfileWire(const std::vector<std::pair<double,double>>& poly,
                            const RevolveFrame& f)
{
    const gp_Vec ex(f.xAxis), ez(f.zAxis);
    std::vector<gp_Pnt> verts;
    verts.reserve(poly.size());
    for (const auto& p : poly) {
        const gp_Vec off = ex * p.first + ez * p.second;   // r along X, z along Z
        verts.emplace_back(f.origin.X() + off.X(),
                           f.origin.Y() + off.Y(),
                           f.origin.Z() + off.Z());
    }

    BRepBuilderAPI_MakeWire wireMk;
    for (size_t i = 0; i < verts.size(); ++i) {
        const gp_Pnt& a = verts[i];
        const gp_Pnt& b = verts[(i + 1) % verts.size()];
        if (a.Distance(b) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(a, b);
        if (!em.IsDone())
            throw SkillError("revolve_boss: edge build failed");
        wireMk.Add(em.Edge());
    }
    if (!wireMk.IsDone())
        throw SkillError("revolve_boss: wire build failed");
    return wireMk.Wire();
}

// Two faces are adjacent iff they share an edge.
bool facesShareEdge(const TopoDS_Face& a, const TopoDS_Face& b)
{
    for (TopExp_Explorer ea(a, TopAbs_EDGE); ea.More(); ea.Next())
        for (TopExp_Explorer eb(b, TopAbs_EDGE); eb.More(); eb.Next())
            if (ea.Current().IsSame(eb.Current())) return true;
    return false;
}

// Detect a CONE PROTRUSION: a single cone face FUSED to a host body (a watch
// crown knob, a tapered standoff), which the whole-part "solid of revolution"
// gate can never see because the host's walls abort it.  Signature: one cone
// face whose WIDE end ring is shared with the host and whose NARROW end points
// OUTWARD past the body boundary — a short tapered boss.  Emits a revolve_boss
// candidate (trapezoid meridian about the cone's own axis, use_world placement)
// that apply()'s arbitrary-axis revolve regenerates in place.
std::vector<RecognizedFeature> detectConeProtrusions(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Whole-body span, to tell a short boss from a full-depth turned cone.
    Bnd_Box bodyBox; BRepBndLib::Add(wp.shape(), bodyBox);
    double bx0, by0, bz0, bx1, by1, bz1;
    if (bodyBox.IsVoid()) return out;
    bodyBox.Get(bx0, by0, bz0, bx1, by1, bz1);
    const gp_Pnt corners[8] = {
        {bx0,by0,bz0},{bx1,by0,bz0},{bx0,by1,bz0},{bx1,by1,bz0},
        {bx0,by0,bz1},{bx1,by0,bz1},{bx0,by1,bz1},{bx1,by1,bz1} };

    for (int ci = 0; ci < wp.faceCount(); ++ci) {
        BRepAdaptor_Surface s(wp.face(ci));
        if (s.GetType() != GeomAbs_Cone) continue;
        const gp_Cone cone = s.Cone();
        const double refR = cone.RefRadius();
        const double semi = cone.SemiAngle();
        // Guard degenerate cones (a near-cylindrical SemiAngle≈0 or a near-flat
        // ≈90° one) whose tan() blows up or vanishes — they yield NaN/Inf radii.
        if (!std::isfinite(refR) || !std::isfinite(semi) ||
            std::abs(semi) < 1e-4 || std::abs(std::abs(semi) - M_PI / 2.0) < 1e-4)
            continue;
        const double tanA = std::tan(semi);
        if (!std::isfinite(tanA)) continue;
        const gp_Pnt cLoc = cone.Axis().Location();
        gp_Dir cAxis = cone.Axis().Direction();

        // Axial extent of THIS cone face along its own axis (face vertices, not a
        // world AABB — correct for any tilt).
        double aLo = 1e30, aHi = -1e30;
        for (TopExp_Explorer ve(wp.face(ci), TopAbs_VERTEX); ve.More(); ve.Next()) {
            const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(ve.Current()));
            const double a = gp_Vec(cLoc, p).Dot(gp_Vec(cAxis));
            aLo = std::min(aLo, a); aHi = std::max(aHi, a);
        }
        if (aHi - aLo < 0.3) continue;                  // too thin to be a knob
        double rLo = refR + aLo * tanA, rHi = refR + aHi * tanA;
        if (rLo < -1e-6 || rHi < -1e-6) continue;       // apex crossing — not a frustum
        // Orient the axis so it points from the WIDE end to the NARROW end (the
        // protrusion direction): the narrow end is the free tip.
        double aWide = aLo, aNarrow = aHi, rWide = rLo, rNarrow = rHi;
        if (rLo < rHi) { aWide = aHi; aNarrow = aLo; rWide = rHi; rNarrow = rLo; }
        if (rWide < 0.4 || std::abs(rWide - rNarrow) < 1e-3) continue;  // not tapered
        // Outward tip direction along the cone axis (sign that increases toward
        // the narrow end).
        gp_Dir tipDir = (aNarrow >= aWide) ? cAxis
                                           : gp_Dir(-cAxis.X(), -cAxis.Y(), -cAxis.Z());
        const gp_Pnt wideCtr(cLoc.X() + cAxis.X() * aWide,
                             cLoc.Y() + cAxis.Y() * aWide,
                             cLoc.Z() + cAxis.Z() * aWide);
        const gp_Pnt narrowCtr(cLoc.X() + cAxis.X() * aNarrow,
                               cLoc.Y() + cAxis.Y() * aNarrow,
                               cLoc.Z() + cAxis.Z() * aNarrow);

        // PROTRUSION gates:
        //  (1) the cone is SHORT (a boss rises a little; a turned taper spans the
        //      whole part).  Compare its axial length to the body span along tip.
        const double height = aHi - aLo;
        double bMin = 1e30, bMax = -1e30;
        for (const auto& c : corners) {
            const double t = gp_Vec(narrowCtr, c).Dot(gp_Vec(tipDir));
            bMin = std::min(bMin, t); bMax = std::max(bMax, t);
        }
        const double bodySpan = bMax - bMin;
        if (height > 0.6 * bodySpan) continue;          // a full-depth turned cone, not a boss
        //  (2) the NARROW tip is at the OUTER boundary along tipDir (it protrudes,
        //      i.e. no body material beyond it).
        double narrowAxial = gp_Vec(narrowCtr, narrowCtr).Dot(gp_Vec(tipDir));  // 0 by defn
        (void)narrowAxial;
        double bodyMaxAlongTip = -1e30;
        for (const auto& c : corners)
            bodyMaxAlongTip = std::max(bodyMaxAlongTip,
                                       gp_Vec(narrowCtr, c).Dot(gp_Vec(tipDir)));
        if (bodyMaxAlongTip > 0.3) continue;            // material beyond the tip → not a tip
        //  (3) the boss FOOTPRINT must sit INSIDE the host cross-section: a real
        //      knob rises from the middle of a larger face, so the host extends
        //      well beyond the wide ring in the radial directions.  A CHAMFER (or
        //      any rim bevel) instead has its wide end at the host's OWN edge —
        //      rWide ≈ the host half-extent — leaving no host material around it.
        //      Reject when the wide ring reaches the host's radial boundary.
        double radialHalfExtent = 0.0;   // host half-size perpendicular to tipDir
        for (const auto& c : corners) {
            const gp_Vec d(narrowCtr, c);
            const gp_Vec axial = gp_Vec(tipDir) * d.Dot(gp_Vec(tipDir));
            radialHalfExtent = std::max(radialHalfExtent, (d - axial).Magnitude());
        }
        if (rWide > 0.7 * radialHalfExtent) continue;   // wide ring at host edge → a bevel, not a boss
        //  (4) the WIDE end ring must be SHARED with the host (fused), else it is a
        //      free-standing cone the whole-part path already handles.
        bool wideShared = false;
        for (int oj = 0; oj < wp.faceCount() && !wideShared; ++oj) {
            if (oj == ci) continue;
            if (facesShareEdge(wp.face(ci), wp.face(oj))) wideShared = true;
        }
        if (!wideShared) continue;

        // Final finiteness guard — never emit a candidate carrying NaN/Inf (a
        // degenerate cone face slipped through would poison the JSON downstream).
        if (!std::isfinite(rWide) || !std::isfinite(rNarrow) ||
            !std::isfinite(wideCtr.X()) || !std::isfinite(wideCtr.Y()) ||
            !std::isfinite(wideCtr.Z()) || !std::isfinite(height))
            continue;

        // Recover the meridian in the cone's own frame, axis pointing along tip so
        // apply() places (r,z) with z growing toward the tip.  z=0 at the wide end.
        const double zTip = aHi - aLo;
        json meridian = json::array({
            { { "r", 0.0   }, { "z", 0.0  } }, { { "r", rWide   }, { "z", 0.0  } },
            { { "r", rNarrow }, { "z", zTip } }, { { "r", 0.0  }, { "z", zTip } },
        });
        json recovered = {
            { "profile_polyline",     meridian },
            { "axis_origin",          { wideCtr.X(), wideCtr.Y(), wideCtr.Z() } },
            { "axis_dir",             { tipDir.X(), tipDir.Y(), tipDir.Z() } },
            { "revolution_angle_deg", 360.0 },
            { "max_radius_mm",        rWide },
            { "axial_span_mm",        height },
        };
        json matched = {
            { "source",        "geometry" },
            { "cone_protrusion", true },
            { "cone_face_id",  ci },
            { "wide_radius_mm", rWide },
            { "narrow_radius_mm", rNarrow },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.85, matched });
    }
    return out;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.profile_polyline.size() < 3) {
        r.add("DFM-INPUT", "error",
              "revolve_boss: profile needs >= 3 vertices, got " +
              std::to_string(in.profile_polyline.size()));
    }
    if (in.revolution_angle_deg <= 0.0 || in.revolution_angle_deg > 360.0) {
        r.add("DFM-INPUT", "error",
              "revolve_boss: revolution_angle_deg " +
              std::to_string(in.revolution_angle_deg) +
              " out of (0, 360] range");
    }
    if (!in.profile_polyline.empty() && minRofProfile(in.profile_polyline) < 0.0) {
        r.add("DFM-INPUT", "error",
              "revolve_boss: profile has r < 0 — would cross revolution axis");
    }
    const double zSpan = zRangeOfProfile(in.profile_polyline);
    if (zSpan > 0.0 && zSpan < 0.4) {
        r.add("DFM-001", "error",
              "revolve_boss: profile axial span " + std::to_string(zSpan) +
              " mm < 0.4 mm (min wall thickness)");
    }
    if (in.profile_polyline.size() >= 3 &&
        polygonAreaRZ(in.profile_polyline) < 1e-6) {
        r.add("DFM-INPUT", "error",
              "revolve_boss: profile area ~0 (collinear?)");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "revolve_boss DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Revolve about an ARBITRARY axis: the profile (r,z) is placed in a frame
    // whose Z is the axis direction and whose X is a perpendicular radial.  A
    // ±Z axis with origin at 0 reproduces the legacy world-XZ behaviour, so
    // existing straight/ring revolves are unchanged; a radial axis (e.g. a
    // watch crown knob pointing ±X/±Y) now regenerates too.
    const RevolveFrame frame = makeRevolveFrame(in.axis_origin, in.axis_dir);
    const TopoDS_Wire wire = makeProfileWire(in.profile_polyline, frame);

    BRepBuilderAPI_MakeFace faceMk(wire, true);
    if (!faceMk.IsDone())
        throw SkillError("revolve_boss: face build failed");

    const gp_Ax1 axis(in.axis_origin, in.axis_dir);
    const double angRad = in.revolution_angle_deg * M_PI / 180.0;

    BRepPrimAPI_MakeRevol revol(faceMk.Face(), axis, angRad);
    revol.Build();
    if (!revol.IsDone())
        throw SkillError("revolve_boss: revol build failed");
    const TopoDS_Shape solid = revol.Shape();

    const TopoDS_Shape newShape = wp.shape().IsNull()
                                  ? solid
                                  : pr::fuse(wp.shape(), solid);

    // Volume by Pappus: V = 2π · R_centroid · A · (angle / 360)
    const double area = polygonAreaRZ(in.profile_polyline);
    const double rC   = polygonCentroidR(in.profile_polyline);
    const double vol  = 2.0 * M_PI * rC * area *
                        (in.revolution_angle_deg / 360.0);

    json polyJson = json::array();
    for (const auto& p : in.profile_polyline)
        polyJson.push_back({ { "r", p.first }, { "z", p.second } });

    json params = {
        { "profile_polyline",     polyJson },
        { "axis_origin",          { in.axis_origin.X(),
                                    in.axis_origin.Y(),
                                    in.axis_origin.Z() } },
        { "axis_dir",             { in.axis_dir.X(),
                                    in.axis_dir.Y(),
                                    in.axis_dir.Z() } },
        { "revolution_angle_deg", in.revolution_angle_deg },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      1 },
        { "sketch_vertex_count",   static_cast<int>(in.profile_polyline.size()) },
        { "revolution_angle_deg",  in.revolution_angle_deg },
        { "derived_volume_mm3",    vol },
        { "max_radius_mm",         maxRofProfile(in.profile_polyline) },
        { "axial_span_mm",         zRangeOfProfile(in.profile_polyline) },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "lathe;revolve_add";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = zRangeOfProfile(in.profile_polyline) + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = -vol;
    tooling.est_cycle_time_s  = std::max(5.0,
        in.profile_polyline.size() * 0.3 +
        in.revolution_angle_deg / 60.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::revolve_boss applied: n={} angle={} vol={} faces {}->{}",
                  in.profile_polyline.size(),
                  in.revolution_angle_deg, vol,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        // "metadata_replay" is the cap-exempt source (see analyze()): a
        // same-session revolve replay keeps full confidence and round-trips.
        // "feature_history" was capped to 0.5 and dropped below 0.7.
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric path B-prime: a CONE PROTRUSION (crown knob, tapered standoff)
    // fused to a host body.  The whole-part "solid of revolution" gate below
    // can never see it (the host's walls abort that gate), so detect a host-
    // fused, outward-pointing tapered cone independently.  If found, return it —
    // the whole-part path would only mis-read the host.
    {
        auto cones = detectConeProtrusions(wp);
        if (!cones.empty()) return cones;
    }

    // Geometric path B: detect a SOLID OF REVOLUTION (a lathe/revolve part) by
    // grouping COAXIAL revolution faces (cylinder/cone/torus/...) and confirming
    // every planar face is a CAP (perpendicular to the axis).  Recovers the axis
    // + radial/axial envelope.  The full meridian profile (for exact
    // regeneration) is a follow-up — left empty here.
    struct RevFace { int id; gp_Ax1 axis; };
    std::vector<RevFace> revFaces;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceRevolution(i)) continue;
        if (auto ax = wp.faceRevolutionAxis(i)) revFaces.push_back({ i, *ax });
    }
    if (revFaces.empty()) return out;

    // Coaxial test (parallel directions + same axis line) — mirrors the helper
    // in terminal_block_post.cpp.
    auto coaxial = [](const gp_Ax1& a, const gp_Ax1& b) {
        if (!a.Direction().IsParallel(b.Direction(), 1e-2)) return false;
        const gp_Vec v(b.Location(), a.Location());
        const gp_Vec d(b.Direction());
        return (v - d * v.Dot(d)).Magnitude() < 0.5;
    };
    int bestCount = 0;
    gp_Ax1 axis;
    for (const auto& rf : revFaces) {
        int cnt = 0;
        for (const auto& o : revFaces) if (coaxial(rf.axis, o.axis)) ++cnt;
        if (cnt > bestCount) { bestCount = cnt; axis = rf.axis; }
    }
    if (bestCount < 1) return out;
    const gp_Dir aDir = axis.Direction();
    const gp_Pnt aLoc = axis.Location();

    // SPECIFICITY gate: a pure solid of revolution's planar faces are all CAPS
    // (normal PARALLEL to the axis).  A drilled/pocketed block has WALLS (normal
    // perpendicular to the axis), so reject any planar face that is not a cap —
    // this keeps a machined part from being mis-read as a revolution.
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { return out; }
        if (std::abs(gp_Vec(n).Dot(gp_Vec(aDir))) < 0.9) return out;  // a wall
    }

    // Envelope: max radius from the revolution faces' radii; axial span from the
    // solid bbox projected onto the axis.
    double maxR = 0.0;
    for (const auto& rf : revFaces) {
        BRepAdaptor_Surface s(wp.face(rf.id));
        if (s.GetType() == GeomAbs_Cylinder)   maxR = std::max(maxR, s.Cylinder().Radius());
        else if (s.GetType() == GeomAbs_Cone)  maxR = std::max(maxR, s.Cone().RefRadius());
    }
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    double aMin = 1e30, aMax = -1e30;
    const double cx[2] = { xMin, xMax }, cy[2] = { yMin, yMax }, cz[2] = { zMin, zMax };
    for (int ix = 0; ix < 2; ++ix)
        for (int iy = 0; iy < 2; ++iy)
            for (int iz = 0; iz < 2; ++iz) {
                const double t = gp_Vec(aLoc, gp_Pnt(cx[ix], cy[iy], cz[iz])).Dot(gp_Vec(aDir));
                aMin = std::min(aMin, t);
                aMax = std::max(aMax, t);
            }
    const double axialSpan = aMax - aMin;
    if (maxR < 0.1 || axialSpan < 0.1) return out;

    // MERIDIAN recovery: for the simplest revolution — a SINGLE solid cylinder
    // about ±Z (what apply() can regenerate) — the meridian is the rectangle
    // {(0,zMin),(R,zMin),(R,zMax),(0,zMax)}, which revolves back to the same
    // cylinder (a full (r,z) round-trip).  A SINGLE cone frustum (the watch
    // crown knob) recovers as a trapezoid {(0,z0),(r0,z0),(r1,z1),(0,z1)} with
    // the two end radii read from the cone surface.  Stepped/hollow meridians
    // (multiple distinct revolution faces) are still a follow-up.
    json meridian = json::array();
    int cylCount = 0, coneCount = 0;
    int cylId = -1, coneId = -1;
    for (const auto& rf : revFaces) {
        const GeomAbs_SurfaceType st = BRepAdaptor_Surface(wp.face(rf.id)).GetType();
        if (st == GeomAbs_Cylinder) { ++cylCount; cylId = rf.id; }
        else if (st == GeomAbs_Cone) { ++coneCount; coneId = rf.id; }
    }
    // Recovered axis defaults to the legacy ±Z-through-origin convention; the
    // cone path (which supports any axis) overrides it with the real cone axis.
    gp_Pnt recAxisOrigin(0.0, 0.0, 0.0);
    gp_Dir recAxisDir(0.0, 0.0, aDir.Z() > 0 ? 1.0 : -1.0);

    if (cylCount == 1 && coneCount == 0) {
        // A single solid cylinder about ANY axis.  The meridian is a rectangle
        // {(0,zLo),(R,zLo),(R,zHi),(0,zHi)} where zLo/zHi are the cylinder face's
        // axial extent measured ALONG ITS OWN AXIS (project the face vertices —
        // this is orientation-robust, unlike a world-Z bbox which only works for a
        // ±Z axis and is fragile at the |aDir.Z|≈1 boundary).  The recovered axis
        // is the cylinder axis itself, normalized to +Z when it is ~±Z so apply()'s
        // legacy ±Z frame keeps the profile on world +Z (consistent with the cone
        // path and the -Z mirror fix).
        const gp_Cylinder cyl = BRepAdaptor_Surface(wp.face(cylId)).Cylinder();
        const double R = cyl.Radius();
        const gp_Pnt cLoc = cyl.Axis().Location();
        gp_Dir cDir = cyl.Axis().Direction();
        if (std::abs(cDir.Z()) > 0.999 && cDir.Z() < 0.0)
            cDir = gp_Dir(0.0, 0.0, 1.0);
        // Project the cylinder face's OWN bounding box onto the axis for the axial
        // extent.  (A full cylindrical face has no corner VERTICES — only seam
        // edges — so a vertex scan can come up empty; its tight bbox always has
        // the two end caps' extent.)  Orientation-robust for any axis.
        Bnd_Box cfb;
        BRepBndLib::AddOptimal(wp.face(cylId), cfb);
        double zLo = 1e30, zHi = -1e30;
        if (!cfb.IsVoid()) {
            double q0, q1, q2, q3, q4, q5; cfb.Get(q0, q1, q2, q3, q4, q5);
            const double qx[2] = { q0, q3 }, qy[2] = { q1, q4 }, qz[2] = { q2, q5 };
            for (int ix = 0; ix < 2; ++ix)
                for (int iy = 0; iy < 2; ++iy)
                    for (int iz = 0; iz < 2; ++iz) {
                        const double a = gp_Vec(cLoc, gp_Pnt(qx[ix], qy[iy], qz[iz]))
                                             .Dot(gp_Vec(cDir));
                        zLo = std::min(zLo, a); zHi = std::max(zHi, a);
                    }
        }
        if (zHi - zLo > 1e-3 && std::isfinite(R) && R > 0.0) {
            meridian = json::array({
                { { "r", 0.0 }, { "z", zLo } }, { { "r", R }, { "z", zLo } },
                { { "r", R   }, { "z", zHi } }, { { "r", 0.0 }, { "z", zHi } },
            });
            recAxisOrigin = cLoc;
            recAxisDir    = cDir;
        }
    } else if (coneCount == 1 && cylCount == 0) {
        // A single solid cone frustum (the watch crown knob) about an ARBITRARY
        // axis.  The radius varies linearly along the cone axis:
        //   r(a) = |refR + a*tan(semiAngle)|,  a = axial offset from cone Location.
        // The meridian (r,z) is expressed in the cone's OWN frame (z = signed
        // distance along the cone axis), and the recovered axis is the cone axis
        // itself — so apply() revolves it back in place regardless of orientation.
        const gp_Cone cone = BRepAdaptor_Surface(wp.face(coneId)).Cone();
        const double refR  = cone.RefRadius();
        const double tanA  = std::tan(cone.SemiAngle());
        const gp_Pnt cLoc  = cone.Axis().Location();
        // If the cone axis is ~±Z, apply()'s frame forces the profile z onto
        // world +Z (legacy parity), so a -Z cone axis would mismatch the meridian
        // z (which is measured ALONG cDir).  Normalize a near-±Z cone axis to +Z
        // here; the axial offsets below are then measured along +Z too, keeping
        // the recovered axis and meridian consistent.  Off-axis cones keep cDir.
        gp_Dir cDir = cone.Axis().Direction();
        if (std::abs(cDir.Z()) > 0.999 && cDir.Z() < 0.0)
            cDir = gp_Dir(0.0, 0.0, 1.0);
        // Axial extent of the cone FACE along its OWN axis.  Projecting the WORLD
        // axis-aligned bounding box corners (the old approach) is correct ONLY
        // when the axis is world-aligned: for a tilted axis the AABB extends past
        // the true face ends and over-states the span (e.g. a +X/+Y cone's span
        // 5mm reads as 12.4mm).  Instead project the cone face's OWN vertices onto
        // the axis — those are the real end rings, valid for any orientation.
        double aLoF = 1e30, aHiF = -1e30;
        for (TopExp_Explorer ve(wp.face(coneId), TopAbs_VERTEX); ve.More(); ve.Next()) {
            const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(ve.Current()));
            const double a = gp_Vec(cLoc, p).Dot(gp_Vec(cDir));
            aLoF = std::min(aLoF, a); aHiF = std::max(aHiF, a);
        }
        // A radius that varies linearly with axial position.  Guard the APEX: if
        // refR + a*tanA goes negative anywhere in [aLoF,aHiF] the frustum would
        // cross its own apex, and abs()-folding it would emit a bogus positive
        // radius.  In that (degenerate / mis-measured) case leave the meridian
        // empty so the candidate stays sub-threshold instead of mis-regenerating.
        const double rLoSigned = refR + aLoF * tanA;
        const double rHiSigned = refR + aHiF * tanA;
        const bool apexCrossed = (rLoSigned < -1e-6) || (rHiSigned < -1e-6);
        const double r0 = rLoSigned;
        const double r1 = rHiSigned;
        if (!apexCrossed && std::max(r0, r1) > 0.1 && std::abs(r0 - r1) > 1e-3 &&
            (aHiF - aLoF) > 1e-3) {
            // Trapezoid meridian in the cone frame: z runs along the cone axis.
            meridian = json::array({
                { { "r", 0.0 }, { "z", aLoF } }, { { "r", r0 }, { "z", aLoF } },
                { { "r", r1  }, { "z", aHiF } }, { { "r", 0.0 }, { "z", aHiF } },
            });
            recAxisOrigin = cLoc;
            recAxisDir    = cDir;
        }
    }

    json recovered = {
        { "profile_polyline",     meridian },
        { "axis_origin",          { recAxisOrigin.X(), recAxisOrigin.Y(), recAxisOrigin.Z() } },
        { "axis_dir",             { recAxisDir.X(), recAxisDir.Y(), recAxisDir.Z() } },
        { "revolution_angle_deg", 360.0 },
        { "max_radius_mm",        maxR },
        { "axial_span_mm",        axialSpan },
    };
    json matched = {
        { "source",                "geometry" },
        { "revolution_face_count", bestCount },
        { "max_radius_mm",         maxR },
        { "axial_span_mm",         axialSpan },
        { "meridian_recovered",    !meridian.empty() },
    };
    // Surface at full confidence ONLY when the meridian was recovered (a single
    // cylinder), so a regeneratable revolve reaches the Executor.  A cone /
    // stepped / hollow revolution has an EMPTY profile that apply() would reject
    // (profile.size() < 3) — emit it as a weak, sub-threshold detection so it is
    // NOT replayed into a failing Executor step.  (Full cone/stepped meridian
    // recovery is a follow-up.)
    const double conf = meridian.empty() ? 0.5 : 0.9;
    out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    return out;
}

}  // namespace koocadcam::skill::revolve_boss
