// @lat: [[engine/skills#revolve_boss]]

#include "revolve_boss.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <TopoDS.hxx>
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

// Build the closed wire of the profile polygon on the world XZ plane
// (y = 0).  Caller is responsible for the orientation matching axis_dir =
// +Z, which is the default; non-Z axes are not supported in slice-10.
TopoDS_Wire makeProfileWireXZ(const std::vector<std::pair<double,double>>& poly)
{
    std::vector<gp_Pnt> verts;
    verts.reserve(poly.size());
    for (const auto& p : poly)
        verts.emplace_back(p.first, 0.0, p.second);

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

    // For slice-10 we only support axis_dir = +Z about an arbitrary origin
    // (the most common SolidWorks usage).  Non-Z axes are TODO.
    if (std::abs(std::abs(in.axis_dir.Z()) - 1.0) > 1e-3) {
        throw SkillError("revolve_boss: only axis_dir = +/-Z supported in slice-10");
    }

    const TopoDS_Wire wire = makeProfileWireXZ(in.profile_polyline);

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

    // Geometric: a revolve produces axisymmetric cylindrical / toroidal faces.
    // Count cylindrical faces whose axis is along +Z.
    int axialCylCount = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        ++axialCylCount;
    }
    if (axialCylCount < 1) return out;

    json recovered = {
        { "profile_polyline",     json::array() },
        { "axis_origin",          { 0.0, 0.0, 0.0 } },
        { "axis_dir",             { 0.0, 0.0, 1.0 } },
        { "revolution_angle_deg", 360.0 },
    };
    json matched = {
        { "axial_cyl_face_count", axialCylCount },
    };
    const double conf = std::clamp(0.40 + 0.05 * axialCylCount, 0.40, 0.75);
    out.push_back(RecognizedFeature{ kSkillId, recovered, conf, matched });
    return out;
}

}  // namespace koocadcam::skill::revolve_boss
