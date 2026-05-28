// @lat: [[engine/skills#contour_turn]]

#include "contour_turn.hpp"

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
#include <gp.hxx>
#include <gp_Ax1.hxx>
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

namespace koocadcam::skill::contour_turn {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

TopoDS_Shape buildContourCutter(const std::vector<ProfilePoint>& profile,
                                double envelopeR)
{
    if (profile.size() < 5)
        throw Standard_Failure("contour_turn::buildContourCutter: ≥ 5 points required");

    BRepBuilderAPI_MakeWire wire;
    auto P = [](double r, double z) { return gp_Pnt(r, 0.0, z); };

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

    wire.Add(BRepBuilderAPI_MakeEdge(P(rHigh,     zHigh),
                                     P(envelopeR, zHigh)).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(P(envelopeR, zHigh),
                                     P(envelopeR, zLow )).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(P(envelopeR, zLow),
                                     P(rLow,      zLow )).Edge());

    if (!wire.IsDone())
        throw Standard_Failure("contour_turn: wire incomplete");

    BRepBuilderAPI_MakeFace face(wire.Wire(), true);
    if (!face.IsDone())
        throw Standard_Failure("contour_turn: face build failed");

    const gp_Ax1 axis(gp_Pnt(0.0, 0.0, 0.0), gp::DZ());
    BRepPrimAPI_MakeRevol rev(face.Face(), axis, 2.0 * M_PI, true);
    rev.Build();
    if (!rev.IsDone())
        throw Standard_Failure("contour_turn: revolve failed");
    return rev.Shape();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.profile.size() < 5) {
        r.add("DFM-INPUT", "error",
              "contour_turn requires ≥ 5 profile waypoints (got " +
              std::to_string(in.profile.size()) +
              ") — use form_turn for sparse profiles");
    }
    if (in.pass_count < 1) {
        r.add("DFM-INPUT", "error",
              "contour_turn pass_count must be ≥ 1 (got " +
              std::to_string(in.pass_count) + ")");
    }

    for (size_t i = 1; i < in.profile.size(); ++i) {
        if (in.profile[i].z_mm <= in.profile[i - 1].z_mm) {
            r.add("DFM-INPUT", "error",
                  "contour_turn profile z must be strictly increasing "
                  "(violation at index " + std::to_string(i) + ")");
            break;
        }
    }
    constexpr double kMinR = 0.4;
    for (size_t i = 0; i < in.profile.size(); ++i) {
        if (in.profile[i].r_mm < kMinR) {
            r.add("DFM-INPUT", "error",
                  "contour_turn profile r at index " + std::to_string(i) +
                  " < " + std::to_string(kMinR) + " mm");
            break;
        }
    }

    if (!wp.shape().IsNull()) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        const double outerR = bb.outerRadiusXY();
        for (size_t i = 0; i < in.profile.size(); ++i) {
            if (in.profile[i].r_mm >= outerR - 1e-4) {
                r.add("DFM-LATHE", "error",
                      "contour_turn profile r at index " + std::to_string(i) +
                      " (" + std::to_string(in.profile[i].r_mm) +
                      ") ≥ stock outer (" + std::to_string(outerR) +
                      ") — nothing to remove");
                break;
            }
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "contour_turn DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double envelopeR = bb.outerRadiusXY() + 0.5;

    const TopoDS_Shape cutter = buildContourCutter(in.profile, envelopeR);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    // Estimate removed volume (same trapezoidal-annulus rule as form_turn).
    double removedVol = 0.0;
    {
        const double R = bb.outerRadiusXY();
        for (size_t i = 1; i < in.profile.size(); ++i) {
            const double dz = in.profile[i].z_mm - in.profile[i - 1].z_mm;
            const double r1 = in.profile[i - 1].r_mm;
            const double r2 = in.profile[i    ].r_mm;
            const double aArea = M_PI * (R * R - (r1 * r1 + r2 * r2) / 2.0);
            removedVol += std::max(0.0, aArea) * std::max(0.0, dz);
        }
    }

    json profileJ = json::array();
    for (const auto& p : in.profile)
        profileJ.push_back({ { "z_mm", p.z_mm }, { "r_mm", p.r_mm } });

    json params = {
        { "profile",    profileJ },
        { "pass_count", in.pass_count },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "axis",                  { 0.0, 0.0, 1.0 } },
        { "profile",               profileJ },
        { "point_count",           static_cast<int>(in.profile.size()) },
        { "pass_count",            in.pass_count },
        { "envelope_radius_mm",    envelopeR },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "lathe_contour_insert";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    =
        in.profile.back().z_mm - in.profile.front().z_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 850.0;        // contour finishing is fast
    tooling.feed_per_tooth_mm = 0.06;         // fine finish
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(2.0,
        in.pass_count *
        (in.profile.back().z_mm - in.profile.front().z_mm) / 40.0);
    tooling.extra = json{
        { "pass_count",     in.pass_count },
        { "point_count",    static_cast<int>(in.profile.size()) },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::contour_turn applied: {} points, {} passes, faces {}→{}",
                  in.profile.size(), in.pass_count,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata replay first; then geometric fallback (low confidence).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

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

    // Geometric fallback — coarse 2-point synthesis from any inner cylinder.
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
        if (cyl.Radius() >= outerR - 1e-3) continue;

        // Build a degenerate 5-point profile so recovered params validate.
        json profileJ = json::array();
        const double r = cyl.Radius();
        const double zMin = bb.zMin, zMax = bb.zMax;
        for (int k = 0; k < 5; ++k) {
            const double t = static_cast<double>(k) / 4.0;
            profileJ.push_back({
                { "z_mm", zMin + t * (zMax - zMin) },
                { "r_mm", r },
            });
        }
        json recovered = {
            { "profile",    profileJ },
            { "pass_count", 1 },
        };
        json matched = {
            { "face_id",            fIdx },
            { "approx_radius_mm",   r },
            { "source",             "geometric_fallback" },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.35, matched });
        break;
    }
    return out;
}

}  // namespace koocadcam::skill::contour_turn
