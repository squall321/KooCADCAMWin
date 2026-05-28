// @lat: [[engine/skills#metal_spin]]

#include "metal_spin.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"

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

namespace koocadcam::skill::metal_spin {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── helpers ──────────────────────────────────────────────────────────────

namespace {

// Estimate wall thinning percentage based on the spread of the profile.
// A flat profile (r constant) keeps original thickness; a steep cone or
// re-entrant profile thins the wall.  We use the cosine-of-flange-angle
// approximation: t_final ≈ t_blank · sin(α) where α is the half-angle of
// the cone between successive points.  Cumulative thinning is the mean
// of segment thinning weighted by segment length.
double estimateThinningPct(const std::vector<ProfilePoint>& profile)
{
    if (profile.size() < 2) return 0.0;
    double totalLen   = 0.0;
    double totalThin  = 0.0;
    for (size_t i = 1; i < profile.size(); ++i) {
        const double dz = profile[i].z_mm - profile[i - 1].z_mm;
        const double dr = profile[i].r_mm - profile[i - 1].r_mm;
        const double len = std::sqrt(dz * dz + dr * dr);
        if (len < 1e-9) continue;
        // Flange half-angle α from the +Z direction.
        // sin(α) = dr / len ; thinning_factor = 1 − cos(α) = 1 − dz / len.
        const double cosA = std::abs(dz) / len;
        const double thinFrac = 1.0 - cosA;     // 0 = no thin (axial), 1 = full thin (radial)
        totalThin += thinFrac * len;
        totalLen  += len;
    }
    if (totalLen <= 0.0) return 0.0;
    return (totalThin / totalLen) * 100.0;
}

// Estimate the maximum flange angle in the profile (degrees from Z).
double maxFlangeAngleDeg(const std::vector<ProfilePoint>& profile)
{
    double maxAng = 0.0;
    for (size_t i = 1; i < profile.size(); ++i) {
        const double dz = profile[i].z_mm - profile[i - 1].z_mm;
        const double dr = profile[i].r_mm - profile[i - 1].r_mm;
        const double len = std::sqrt(dz * dz + dr * dr);
        if (len < 1e-9) continue;
        const double ang = std::atan2(std::abs(dr), std::abs(dz)) * 180.0 / M_PI;
        if (ang > maxAng) maxAng = ang;
    }
    return maxAng;
}

TopoDS_Shape buildSpunShell(const std::vector<ProfilePoint>& profile,
                            double blankThicknessMm)
{
    if (profile.size() < 2)
        throw Standard_Failure("metal_spin::buildSpunShell: ≥ 2 points required");

    BRepBuilderAPI_MakeWire wire;

    // Outer profile in the XZ plane (Y=0).
    auto P = [](double r, double z) { return gp_Pnt(r, 0.0, z); };

    for (size_t i = 1; i < profile.size(); ++i) {
        const gp_Pnt p1 = P(profile[i - 1].r_mm, profile[i - 1].z_mm);
        const gp_Pnt p2 = P(profile[i    ].r_mm, profile[i    ].z_mm);
        if (p1.Distance(p2) < 1e-9) continue;
        wire.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
    }

    // Now offset inward by the blank thickness to form the inner profile.
    // For a simple slice-1 model we offset radially inward (smaller r) at
    // each waypoint by `blankThicknessMm` and trace the return path.
    const double t = std::max(0.05, blankThicknessMm);
    for (size_t k = profile.size(); k > 0; --k) {
        const size_t i = k - 1;
        const double rIn = std::max(0.05, profile[i].r_mm - t);
        if (k == profile.size()) {
            // Connect outer top to inner top.
            wire.Add(BRepBuilderAPI_MakeEdge(
                P(profile[i].r_mm, profile[i].z_mm),
                P(rIn,             profile[i].z_mm)).Edge());
        }
        if (i > 0) {
            const double rInPrev = std::max(0.05, profile[i - 1].r_mm - t);
            wire.Add(BRepBuilderAPI_MakeEdge(
                P(rIn,     profile[i    ].z_mm),
                P(rInPrev, profile[i - 1].z_mm)).Edge());
        }
    }

    // Close from inner bottom back to outer bottom.
    const double rInFirst = std::max(0.05, profile.front().r_mm - t);
    wire.Add(BRepBuilderAPI_MakeEdge(
        P(rInFirst,             profile.front().z_mm),
        P(profile.front().r_mm, profile.front().z_mm)).Edge());

    if (!wire.IsDone())
        throw Standard_Failure("metal_spin::buildSpunShell: wire incomplete");

    BRepBuilderAPI_MakeFace face(wire.Wire(), true);
    if (!face.IsDone())
        throw Standard_Failure("metal_spin::buildSpunShell: face build failed");

    const gp_Ax1 axis(gp_Pnt(0.0, 0.0, 0.0), gp::DZ());
    BRepPrimAPI_MakeRevol rev(face.Face(), axis, 2.0 * M_PI, true);
    rev.Build();
    if (!rev.IsDone())
        throw Standard_Failure("metal_spin::buildSpunShell: revolve failed");
    return rev.Shape();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.profile.size() < 2) {
        r.add("DFM-INPUT", "error",
              "metal_spin requires ≥ 2 profile waypoints (got " +
              std::to_string(in.profile.size()) + ")");
        return r;
    }

    for (size_t i = 1; i < in.profile.size(); ++i) {
        if (in.profile[i].z_mm <= in.profile[i - 1].z_mm) {
            r.add("DFM-INPUT", "error",
                  "metal_spin profile z must be strictly increasing "
                  "(violation at index " + std::to_string(i) + ")");
            break;
        }
    }

    constexpr double kMinR = 0.5;
    for (size_t i = 0; i < in.profile.size(); ++i) {
        if (in.profile[i].r_mm < kMinR) {
            r.add("DFM-INPUT", "error",
                  "metal_spin profile radius at index " + std::to_string(i) +
                  " (" + std::to_string(in.profile[i].r_mm) +
                  ") < min " + std::to_string(kMinR) + " mm");
            break;
        }
    }

    if (in.blank_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "metal_spin blank_thickness_mm must be > 0 (got " +
              std::to_string(in.blank_thickness_mm) + ")");
    }

    // Spin-specific checks.
    const double thinPct = estimateThinningPct(in.profile);
    if (thinPct > 70.0) {
        r.add("DFM-SPIN-THIN", "error",
              "metal_spin implied wall_thinning_pct " +
              std::to_string(thinPct) +
              " > 70% — risk of tearing; multi-stage spin required");
    } else if (thinPct > 50.0) {
        r.add("DFM-SPIN-THIN", "warning",
              "metal_spin implied wall_thinning_pct " +
              std::to_string(thinPct) +
              " > 50% — verify lubrication & roller geometry");
    }

    const double maxAng = maxFlangeAngleDeg(in.profile);
    if (maxAng > 80.0) {
        r.add("DFM-SPIN-ANG", "warning",
              "metal_spin max flange angle " + std::to_string(maxAng) +
              " ° > 80° — shear-spinning preferred over conventional spin");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "metal_spin DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const TopoDS_Shape result = buildSpunShell(in.profile, in.blank_thickness_mm);

    const double thinPct = estimateThinningPct(in.profile);
    const double maxAng  = maxFlangeAngleDeg(in.profile);

    json profileJ = json::array();
    for (const auto& p : in.profile)
        profileJ.push_back({ { "z_mm", p.z_mm }, { "r_mm", p.r_mm } });

    json params = {
        { "profile",            profileJ },
        { "blank_thickness_mm", in.blank_thickness_mm },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "axis",               { 0.0, 0.0, 1.0 } },
        { "profile",            profileJ },
        { "point_count",        static_cast<int>(in.profile.size()) },
        { "blank_thickness_mm", in.blank_thickness_mm },
        { "wall_thinning_pct",  thinPct },
        { "max_flange_angle_deg", maxAng },
        { "geometry_changed",   true },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "spin_roller";
    tooling.tool_dia_mm       = 40.0;            // typical roller diameter
    tooling.tool_length_mm    =
        in.profile.back().z_mm - in.profile.front().z_mm + 5.0;
    tooling.tool_material     = "hardened_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;             // forming, not cutting
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;             // forming preserves volume
    tooling.est_cycle_time_s  = std::max(15.0,
        (in.profile.back().z_mm - in.profile.front().z_mm) * 0.4 +
        in.profile.size() * 2.0);
    tooling.extra = json{
        { "process",            "metal_spinning" },
        { "wall_thinning_pct",  thinPct },
        { "max_flange_angle_deg", maxAng },
        { "blank_thickness_mm", in.blank_thickness_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::metal_spin applied: {} profile points, thin={:.1f}%",
                  in.profile.size(), thinPct);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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

    // (b) geometric fallback — any Z-axis cylindrical face implies a spun
    //     shell candidate.
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
        if (rad <= 0.0) continue;

        json profileJ = json::array({
            { { "z_mm", bb.zMin }, { "r_mm", rad } },
            { { "z_mm", bb.zMax }, { "r_mm", rad } },
        });
        json recovered = {
            { "profile",            profileJ },
            { "blank_thickness_mm", 0.0 },
        };
        json matched = {
            { "face_id", fIdx },
            { "approx_radius_mm", rad },
            { "source", "geometric_fallback" },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.35, matched });
        break;
    }
    return out;
}

}  // namespace koocadcam::skill::metal_spin
