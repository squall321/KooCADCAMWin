// @lat: [[engine/skills#swarf_milling]]

#include "swarf_milling.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRep_Builder.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
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

namespace koocadcam::skill::swarf_milling {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.tool_dia_mm < 1.5) {
        r.add("DFM-002", "error",
              "swarf_milling tool_dia_mm " + std::to_string(in.tool_dia_mm) +
              " < 1.5 mm (5-axis end-mill min)");
    }
    if (in.tool_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "tool_length_mm must be > 0");
    }
    if (in.tilt_angle_deg < 0.0 || in.tilt_angle_deg > 60.0) {
        r.add("DFM-SWARF-TILT", "error",
              "swarf_milling tilt_angle_deg " + std::to_string(in.tilt_angle_deg) +
              " outside [0, 60] — beyond this the machine kinematics or "
              "tool flute exposure become impractical");
    }
    if (in.guide_polyline.size() < 2) {
        r.add("DFM-INPUT", "error",
              "swarf_milling guide_polyline requires ≥ 2 points (got " +
              std::to_string(in.guide_polyline.size()) + ")");
    }

    // Always emit the 5-axis advisory — geometry indicates tilt.
    r.add("DFM-005", "info",
          "swarf_milling requires 5-axis machine kinematics — simultaneous "
          "translation along the guide and tool-axis rotation");
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Compute the tilted tool axis at a waypoint:
//   - Polyline tangent T = unit(prev → next).  At endpoints, use the
//     adjacent segment.
//   - Untilted axis Z₀ = -Z (straight down into stock).
//   - Tilt axis  R = unit(Z₀ × T) — perpendicular to both, the rotation axis.
//   - Tilted axis = rotate(Z₀, R, tilt_angle).
//
// If T is parallel to Z₀ (vertical polyline segment), R is undefined; we
// fall back to tilting around +X by tilt_angle.
gp_Dir computeTiltedToolAxis(const std::array<double, 3>& prev,
                             const std::array<double, 3>& next,
                             double tiltDeg)
{
    const gp_Vec tangent(next[0] - prev[0],
                         next[1] - prev[1],
                         next[2] - prev[2]);
    const double tLen = tangent.Magnitude();
    const gp_Dir z0(0.0, 0.0, -1.0);

    if (tLen < 1e-9 || tiltDeg < 1e-3) {
        return z0;
    }
    const gp_Dir tDir(tangent);

    gp_Vec rotAxis = gp_Vec(z0).Crossed(gp_Vec(tDir));
    if (rotAxis.Magnitude() < 1e-6) {
        // Tangent parallel to vertical — fall back to a perpendicular axis.
        rotAxis = gp_Vec(1.0, 0.0, 0.0);
    }
    rotAxis.Normalize();
    const gp_Dir R(rotAxis);

    // Rodrigues rotation of z0 around R by tiltDeg.
    const double a = tiltDeg * M_PI / 180.0;
    const double c = std::cos(a);
    const double s = std::sin(a);
    const gp_Vec z0v(z0);
    const gp_Vec Rv (R);
    // v' = v cos + (R × v) sin + R (R · v)(1 - cos)
    const gp_Vec v_rot =
        z0v * c + Rv.Crossed(z0v) * s + Rv * (Rv.Dot(z0v) * (1.0 - c));
    return gp_Dir(v_rot);
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "swarf_milling DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face_datum);
    if (!entryId) throw SkillError("swarf_milling: entry_face datum unresolved");

    // Build one cutter cylinder per waypoint.  Each cylinder is centered on
    // the waypoint, tilted by tilt_angle_deg about the polyline-tangent ×
    // vertical axis, and has length = tool_length_mm.
    //
    // We position the cylinder so its TIP (the side that cuts) starts at
    // the waypoint and extends `tool_length_mm` AWAY from the surface
    // (along the tool axis).  For the approximation we use the cylinder
    // center as the waypoint and let it extend in both directions; this
    // gives a sweep-like cut without precise CL-point control.
    const double radius   = in.tool_dia_mm / 2.0;
    const double halfLen  = in.tool_length_mm / 2.0;
    const size_t N        = in.guide_polyline.size();
    std::vector<TopoDS_Shape> cylinders;
    cylinders.reserve(N);

    for (size_t i = 0; i < N; ++i) {
        const auto& pt = in.guide_polyline[i];
        // Tangent at waypoint i: use the segment that exists.
        const auto& prev = (i == 0) ? in.guide_polyline[i] : in.guide_polyline[i - 1];
        const auto& next = (i + 1 == N) ? in.guide_polyline[i] : in.guide_polyline[i + 1];
        const gp_Dir toolAxis = computeTiltedToolAxis(prev, next, in.tilt_angle_deg);

        // Start the cylinder half_len above the waypoint (along -toolAxis)
        // so the cylinder straddles the waypoint along its full length.
        const gp_Pnt base(
            pt[0] - toolAxis.X() * halfLen,
            pt[1] - toolAxis.Y() * halfLen,
            pt[2] - toolAxis.Z() * halfLen);
        const gp_Ax2 ax(base, toolAxis);
        try {
            cylinders.push_back(pr::cylinder(ax, radius, in.tool_length_mm));
        } catch (const Standard_Failure&) {
            // Skip degenerate cylinders silently — better than aborting the
            // whole skill.
            continue;
        }
    }
    if (cylinders.empty())
        throw SkillError("swarf_milling: no valid cylinders built from guide_polyline");

    // Combine into a TopoDS_Compound and apply ONE cut.  Per the cookbook,
    // overlapping concentric cylinders fused into a compound, then cut as
    // a single Boolean, performs better and avoids History accumulation.
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    for (const auto& c : cylinders) builder.Add(compound, c);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), compound);

    // Estimate path length and stock removed.
    double pathLen = 0.0;
    for (size_t i = 1; i < N; ++i) {
        const auto& a = in.guide_polyline[i - 1];
        const auto& b = in.guide_polyline[i];
        pathLen += std::sqrt(
            (b[0] - a[0]) * (b[0] - a[0]) +
            (b[1] - a[1]) * (b[1] - a[1]) +
            (b[2] - a[2]) * (b[2] - a[2]));
    }

    json wpJson = json::array();
    for (const auto& w : in.guide_polyline)
        wpJson.push_back({ w[0], w[1], w[2] });

    json params = {
        { "entry_face_id",  *entryId },
        { "guide_polyline", wpJson },
        { "tool_dia_mm",    in.tool_dia_mm },
        { "tool_length_mm", in.tool_length_mm },
        { "tilt_angle_deg", in.tilt_angle_deg },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "cylindrical_face_count",     N },     // approx, may be less after Boolean
        { "is_5axis_cut",               true },
        { "tilt_angle_deg",             in.tilt_angle_deg },
        { "guide_polyline_length",      pathLen },
        { "waypoint_count",             N },
        { "bottom_planar_face_present", false },
        { "geometry",                   "tilted_cylinder_chain_approximation" },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = in.tool_dia_mm;
    tooling.tool_length_mm    = in.tool_length_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;     // 5-axis swarf is slower
    tooling.feed_per_tooth_mm = 0.025;
    // Stock removed approx = π r² × pathLen (cylindrical sweep volume).
    tooling.stock_removed_mm3 = M_PI * radius * radius * pathLen;
    tooling.est_cycle_time_s  = std::max(5.0, pathLen / 25.0);  // ~ 25 mm/s
    tooling.extra["dfm_findings"] = findings;
    tooling.extra["machining_constraint"] =
        "5-axis swarf milling — tool axis tilts continuously along guide";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::swarf_milling applied: N={} dia={} tilt={}° faces {}→{}",
                  N, in.tool_dia_mm, in.tilt_angle_deg,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Low-confidence heuristic: a chain of ≥ 2 cylindrical faces whose axes are
// NOT parallel to any global axis (the giveaway of a tilted 5-axis tool).
// We collect such cylinders; if the centers form a roughly linear sequence
// in 3D, we claim a swarf_milling candidate.

namespace {

bool isAxisAligned(const gp_Dir& d)
{
    const double ax = std::abs(d.X());
    const double ay = std::abs(d.Y());
    const double az = std::abs(d.Z());
    return ax > 0.99 || ay > 0.99 || az > 0.99;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    struct TiltedCyl {
        int    faceIdx;
        gp_Pnt center;
        gp_Dir axis;
        double radius;
    };
    std::vector<TiltedCyl> tilted;

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir adir = cyl.Axis().Direction();
        if (isAxisAligned(adir)) continue;    // standard drills / pockets
        const gp_Pnt center = wp.faceCenter(i);
        tilted.push_back({ i, center, adir, cyl.Radius() });
    }
    if (tilted.size() < 2) return out;

    // Reconstruct an approximate polyline by sorting centers along the
    // direction of greatest spread.  Simple heuristic — find the pair of
    // points with the largest distance, project all others onto that line,
    // sort by projection parameter.
    int bestI = 0, bestJ = 1;
    double bestD = 0.0;
    for (size_t a = 0; a < tilted.size(); ++a)
        for (size_t b = a + 1; b < tilted.size(); ++b) {
            const double d = tilted[a].center.Distance(tilted[b].center);
            if (d > bestD) { bestD = d; bestI = static_cast<int>(a); bestJ = static_cast<int>(b); }
        }
    const gp_Pnt& A = tilted[bestI].center;
    const gp_Pnt& B = tilted[bestJ].center;
    const gp_Vec lineDir(A, B);
    const double lineLen = lineDir.Magnitude();
    if (lineLen < 1e-6) return out;
    const gp_Vec lineUnit = lineDir.Normalized();

    struct Sortable {
        double  t;
        int     faceIdx;
        gp_Pnt  center;
    };
    std::vector<Sortable> sortable;
    for (const auto& c : tilted) {
        const gp_Vec v(A, c.center);
        sortable.push_back({ v.Dot(lineUnit), c.faceIdx, c.center });
    }
    std::sort(sortable.begin(), sortable.end(),
              [](const Sortable& a, const Sortable& b) { return a.t < b.t; });

    json poly = json::array();
    json faceIds = json::array();
    for (const auto& s : sortable) {
        poly.push_back({ s.center.X(), s.center.Y(), s.center.Z() });
        faceIds.push_back(s.faceIdx);
    }

    // Average tool radius across tilted cylinders.
    double rSum = 0.0;
    for (const auto& c : tilted) rSum += c.radius;
    const double avgR = rSum / static_cast<double>(tilted.size());

    // Average tilt: angle of axes from vertical (-Z).
    double angleSum = 0.0;
    for (const auto& c : tilted) {
        const double cosT = std::clamp(c.axis.Dot(gp_Dir(0.0, 0.0, -1.0)), -1.0, 1.0);
        angleSum += std::acos(std::abs(cosT)) * 180.0 / M_PI;
    }
    const double avgTilt = angleSum / static_cast<double>(tilted.size());

    json recovered = {
        { "guide_polyline", poly },
        { "tool_dia_mm",    2.0 * avgR },
        { "tool_length_mm", 0.0 },          // unrecoverable from final geom
        { "tilt_angle_deg", avgTilt },
    };
    json matched = {
        { "cylinder_face_ids", faceIds },
        { "linear_extent",     bestD },
    };
    out.push_back(RecognizedFeature{
        kSkillId, recovered, /*confidence*/ 0.40, matched
    });
    return out;
}

}  // namespace koocadcam::skill::swarf_milling
