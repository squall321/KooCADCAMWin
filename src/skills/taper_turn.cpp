// @lat: [[engine/skills#taper_turn]]

#include "taper_turn.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cone.hxx>
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

namespace koocadcam::skill::taper_turn {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Half-angle of the taper relative to the rotation axis (Z), in radians.
double taperHalfAngle(double r0, double r1, double dz)
{
    if (dz <= 0.0) return 0.0;
    return std::atan(std::abs(r1 - r0) / dz);
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.z1_mm <= in.z0_mm) {
        r.add("DFM-INPUT", "error",
              "taper_turn z1 (" + std::to_string(in.z1_mm) +
              ") must be > z0 (" + std::to_string(in.z0_mm) + ")");
    }
    constexpr double kMinR = 0.4;
    if (in.r0_mm < kMinR || in.r1_mm < kMinR) {
        r.add("DFM-INPUT", "error",
              "taper_turn r0 / r1 must be ≥ " + std::to_string(kMinR) + " mm");
    }

    if (!wp.shape().IsNull()) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        const double outerR = bb.outerRadiusXY();
        if (in.r0_mm >= outerR - 1e-4 || in.r1_mm >= outerR - 1e-4) {
            r.add("DFM-LATHE", "error",
                  "taper_turn r0/r1 (" + std::to_string(in.r0_mm) + "/" +
                  std::to_string(in.r1_mm) +
                  ") must be < stock outer (" + std::to_string(outerR) +
                  ") — nothing to remove");
        }
        if (in.z1_mm > in.z0_mm && (in.z0_mm > bb.zMax + 1e-3 ||
                                    in.z1_mm < bb.zMin - 1e-3)) {
            r.add("DFM-LATHE", "error",
                  "taper_turn Z range does not overlap stock");
        }
    }

    // Taper half-angle limit: 30° (≈ 0.5236 rad).
    if (in.z1_mm > in.z0_mm) {
        const double half = taperHalfAngle(in.r0_mm, in.r1_mm,
                                           in.z1_mm - in.z0_mm);
        if (half > 30.0 * M_PI / 180.0) {
            r.add("DFM-TAPER", "error",
                  "taper_turn half-angle " +
                  std::to_string(half * 180.0 / M_PI) +
                  "° > 30° — use a face/groove op instead");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "taper_turn DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double outerR = bb.outerRadiusXY();

    constexpr double kOverhangR = 0.5;
    constexpr double kOverhangZ = 0.01;
    const double startZ = in.z0_mm - kOverhangZ;
    const double height = (in.z1_mm + kOverhangZ) - startZ;

    // Build a ring whose outer wall is a straight cylinder at (outerR + overhang)
    // and whose inner wall is a cone frustum (r0 at z0 → r1 at z1).  Cutting that
    // ring from the workpiece leaves the cone-frustum shape over [z0, z1].
    // The cone height matches the cylinder height (kOverhangZ << dz, so the
    // small extra extends past z0/z1 negligibly).
    const gp_Ax2 ax(gp_Pnt(0.0, 0.0, startZ), gp::DZ());
    const TopoDS_Shape outer = pr::cylinder(ax, outerR + kOverhangR, height);
    const TopoDS_Shape inner = pr::coneFrustum(ax, in.r0_mm, in.r1_mm, height);
    const TopoDS_Shape ring  = pr::cut(outer, inner);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), ring);

    const double dz = in.z1_mm - in.z0_mm;
    const double halfAngleDeg =
        taperHalfAngle(in.r0_mm, in.r1_mm, dz) * 180.0 / M_PI;

    json params = {
        { "z0_mm", in.z0_mm },
        { "r0_mm", in.r0_mm },
        { "z1_mm", in.z1_mm },
        { "r1_mm", in.r1_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "axis",                       { 0.0, 0.0, 1.0 } },
        { "z0_mm",                      in.z0_mm },
        { "r0_mm",                      in.r0_mm },
        { "z1_mm",                      in.z1_mm },
        { "r1_mm",                      in.r1_mm },
        { "half_angle_deg",             halfAngleDeg },
        { "conical_face_count",         1 },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "lathe_taper_insert";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = dz + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 700.0;
    tooling.feed_per_tooth_mm = 0.12;
    {
        // Annular volume = π·(R² · dz − ∫r(z)²dz).  Frustum integral is
        // π·dz·(r0²+r0·r1+r1²)/3.
        const double R2 = (outerR + kOverhangR) * (outerR + kOverhangR);
        const double cone =
            (in.r0_mm * in.r0_mm + in.r0_mm * in.r1_mm + in.r1_mm * in.r1_mm) / 3.0;
        tooling.stock_removed_mm3 = std::max(0.0, M_PI * dz * (R2 - cone));
    }
    tooling.est_cycle_time_s = std::max(1.0, dz / 30.0);

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::taper_turn applied: r0={} → r1={} over Z[{},{}] half={} deg faces {}→{}",
                  in.r0_mm, in.r1_mm, in.z0_mm, in.z1_mm,
                  halfAngleDeg, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    if (wp.shape().IsNull()) return out;

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double outerR = bb.outerRadiusXY();

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        const TopoDS_Face& f = wp.face(fIdx);
        BRepAdaptor_Surface surf(f);
        if (surf.GetType() != GeomAbs_Cone) continue;
        const gp_Cone cone = surf.Cone();
        if (std::abs(std::abs(cone.Axis().Direction().Z()) - 1.0) > 1e-3)
            continue;
        // Concentric with stock axis.
        const gp_Pnt loc = cone.Axis().Location();
        if (std::sqrt(loc.X() * loc.X() + loc.Y() * loc.Y()) > 0.5) continue;

        const double halfAngle = std::abs(cone.SemiAngle());
        if (halfAngle > 30.0 * M_PI / 180.0 + 1e-3) continue;

        // Collect circular bounding edges (perpendicular to Z).
        std::vector<std::pair<double, double>> zr;  // (z, r) of each circle
        for (TopExp_Explorer exp(f, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Z()) - 1.0) > 1e-3)
                continue;
            zr.emplace_back(c.Location().Z(), c.Radius());
        }
        if (zr.size() < 2) continue;
        std::sort(zr.begin(), zr.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        const double z0 = zr.front().first, r0 = zr.front().second;
        const double z1 = zr.back ().first, r1 = zr.back ().second;
        if (z1 - z0 < 1e-3) continue;
        if (r0 >= outerR - 1e-3 && r1 >= outerR - 1e-3) continue;

        json recovered = {
            { "z0_mm", z0 }, { "r0_mm", r0 },
            { "z1_mm", z1 }, { "r1_mm", r1 },
        };
        json matched = {
            { "face_id",        fIdx },
            { "half_angle_deg", halfAngle * 180.0 / M_PI },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.88, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::taper_turn
