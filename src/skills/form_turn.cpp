// @lat: [[engine/skills#form_turn]]

#include "form_turn.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::form_turn {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.profile.size() < 2) {
        r.add("DFM-INPUT", "error",
              "form_turn requires ≥ 2 profile waypoints (got " +
              std::to_string(in.profile.size()) + ")");
        return r;
    }

    // Monotonic Z (strictly increasing).
    for (size_t i = 1; i < in.profile.size(); ++i) {
        if (in.profile[i].z_mm <= in.profile[i - 1].z_mm) {
            r.add("DFM-INPUT", "error",
                  "form_turn profile z must be strictly increasing "
                  "(violation at index " + std::to_string(i) + ")");
            break;
        }
    }

    // Minimum radius for cutter contact.
    constexpr double kMinR = 0.4;
    for (size_t i = 0; i < in.profile.size(); ++i) {
        if (in.profile[i].r_mm < kMinR) {
            r.add("DFM-INPUT", "error",
                  "form_turn profile radius at index " + std::to_string(i) +
                  " (" + std::to_string(in.profile[i].r_mm) +
                  ") < min " + std::to_string(kMinR) + " mm");
            break;
        }
    }

    if (!wp.shape().IsNull()) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        const double outerR = bb.outerRadiusXY();

        // Every profile radius must be strictly less than stock outer.
        for (size_t i = 0; i < in.profile.size(); ++i) {
            if (in.profile[i].r_mm >= outerR - 1e-4) {
                r.add("DFM-LATHE", "error",
                      "form_turn profile r at index " + std::to_string(i) +
                      " (" + std::to_string(in.profile[i].r_mm) +
                      ") must be < stock outer radius (" +
                      std::to_string(outerR) + ") — nothing to remove there");
                break;
            }
        }

        // Z range must overlap the stock.
        const double zFirst = in.profile.front().z_mm;
        const double zLast  = in.profile.back().z_mm;
        if (zFirst > bb.zMax + 1e-3 || zLast < bb.zMin - 1e-3) {
            r.add("DFM-LATHE", "error",
                  "form_turn profile Z range [" + std::to_string(zFirst) +
                  "," + std::to_string(zLast) + "] does not overlap stock Z [" +
                  std::to_string(bb.zMin) + "," + std::to_string(bb.zMax) + "]");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Build a closed wire in the XZ plane (Y=0):
//   - Trace the profile from (r_0, z_0) to (r_N-1, z_N-1) at constant Y=0.
//   - Close it out to the envelope: up to (R_env, z_N-1), top to (R_env, z_0),
//     back down to (r_0, z_0).
// Then create a planar face on that wire and revolve it 360° about the Z axis.
// The revolved solid is the material to REMOVE.

namespace {

TopoDS_Shape buildFormCutter(const std::vector<ProfilePoint>& profile,
                             double envelopeR)
{
    if (profile.size() < 2)
        throw Standard_Failure("form_turn::buildFormCutter: ≥ 2 points required");

    BRepBuilderAPI_MakeWire wire;

    // Convert (z, r) → (X=r, Y=0, Z=z) for the half-section.
    auto P = [](double r, double z) { return gp_Pnt(r, 0.0, z); };

    // 1) profile path along the (varying r, z).
    for (size_t i = 1; i < profile.size(); ++i) {
        const gp_Pnt p1 = P(profile[i - 1].r_mm, profile[i - 1].z_mm);
        const gp_Pnt p2 = P(profile[i    ].r_mm, profile[i    ].z_mm);
        if (p1.Distance(p2) < 1e-9) continue;
        wire.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
    }

    const double zLow  = profile.front().z_mm;
    const double zHigh = profile.back().z_mm;
    const double rLow  = profile.front().r_mm;
    const double rHigh = profile.back().r_mm;

    // 2) End-cap from last profile point to envelope at zHigh.
    wire.Add(BRepBuilderAPI_MakeEdge(
        P(rHigh,     zHigh),
        P(envelopeR, zHigh)).Edge());

    // 3) Envelope top edge: (R_env, zHigh) → (R_env, zLow).
    wire.Add(BRepBuilderAPI_MakeEdge(
        P(envelopeR, zHigh),
        P(envelopeR, zLow)).Edge());

    // 4) Close back to first profile point.
    wire.Add(BRepBuilderAPI_MakeEdge(
        P(envelopeR, zLow),
        P(rLow,      zLow)).Edge());

    if (!wire.IsDone())
        throw Standard_Failure("form_turn::buildFormCutter: wire incomplete");

    BRepBuilderAPI_MakeFace face(wire.Wire(), /*OnlyPlane=*/true);
    if (!face.IsDone())
        throw Standard_Failure("form_turn::buildFormCutter: face build failed");

    // Revolve the profile face 360° about the global Z axis.
    const gp_Ax1 axis(gp_Pnt(0.0, 0.0, 0.0), gp::DZ());
    BRepPrimAPI_MakeRevol rev(face.Face(), axis, 2.0 * M_PI, true);
    rev.Build();
    if (!rev.IsDone())
        throw Standard_Failure("form_turn::buildFormCutter: revolve failed");
    return rev.Shape();
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "form_turn DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double envelopeR = bb.outerRadiusXY() + 0.5;

    const TopoDS_Shape cutter = buildFormCutter(in.profile, envelopeR);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // Estimate removed volume by trapezoidal annular slabs from the profile.
    double removedVol = 0.0;
    {
        const double R = bb.outerRadiusXY();
        for (size_t i = 1; i < in.profile.size(); ++i) {
            const double dz = in.profile[i].z_mm - in.profile[i - 1].z_mm;
            const double r1 = in.profile[i - 1].r_mm;
            const double r2 = in.profile[i    ].r_mm;
            // Mean cross-sectional annulus over this slab.
            const double aArea = M_PI * (R * R - (r1 * r1 + r2 * r2) / 2.0);
            removedVol += std::max(0.0, aArea) * std::max(0.0, dz);
        }
    }

    // Serialize the profile.
    json profileJ = json::array();
    for (const auto& p : in.profile)
        profileJ.push_back({ { "z_mm", p.z_mm }, { "r_mm", p.r_mm } });

    json params = {
        { "profile", profileJ },
    };
    json pattern = {
        { "kind",            kSkillId },
        { "axis",            { 0.0, 0.0, 1.0 } },
        { "profile",         profileJ },
        { "point_count",     static_cast<int>(in.profile.size()) },
        { "envelope_radius_mm", envelopeR },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "lathe_form_tool";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    =
        in.profile.back().z_mm - in.profile.front().z_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 450.0;            // form tools run slower than OD
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(2.0,
        (in.profile.back().z_mm - in.profile.front().z_mm) / 25.0);
    tooling.extra = json{ { "profile_points", static_cast<int>(in.profile.size()) } };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::form_turn applied: {} profile points, faces {}→{}",
                  in.profile.size(), wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Two paths:
//   (a) Metadata replay  — high confidence (1.0).  Returns the exact
//       polyline that was applied.
//   (b) Geometric fallback — low confidence (0.4).  Detects ANY Z-axis
//       cylindrical face with radius < stock outer and emits a 2-point
//       degenerate profile so consumers know SOMETHING was form-turned.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // (a) metadata replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // (b) geometric fallback — produce a coarse profile from any Z-axis
    //     cylindrical face on the part with radius < stock outer.
    if (wp.shape().IsNull()) return out;
    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double outerR = bb.outerRadiusXY();
    if (outerR <= 1e-6) return out;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface s(wp.face(fIdx));
        const gp_Cylinder cyl = s.Cylinder();
        if (std::abs(std::abs(cyl.Axis().Direction().Z()) - 1.0) > 1e-3)
            continue;
        const double rad = cyl.Radius();
        if (rad >= outerR - 1e-3) continue;

        // Emit a degenerate 2-point profile bracketing the bbox Z range.
        json profileJ = json::array({
            { { "z_mm", bb.zMin }, { "r_mm", rad } },
            { { "z_mm", bb.zMax }, { "r_mm", rad } },
        });
        json recovered = { { "profile", profileJ } };
        json matched = {
            { "face_id", fIdx },
            { "approx_radius_mm", rad },
            { "source", "geometric_fallback" },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.40, matched });
        // One candidate is enough for the fallback — avoid flooding.
        break;
    }
    return out;
}

}  // namespace koocadcam::skill::form_turn
