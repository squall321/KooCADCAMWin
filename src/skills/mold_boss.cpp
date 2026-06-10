// @lat: [[engine/skills#mold_boss]]

#include "mold_boss.hpp"

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
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::mold_boss {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.outer_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "mold_boss outer_dia_mm must be > 0");
    } else if (in.outer_dia_mm < 1.5) {
        r.add("DFM-BOSS-DIA", "error",
              "mold_boss outer_dia " + std::to_string(in.outer_dia_mm) +
              " mm < 1.5 mm minimum");
    }

    if (in.height_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "mold_boss height_mm must be > 0");
    }

    if (in.inner_dia_mm < 0.0) {
        r.add("DFM-INPUT", "error", "mold_boss inner_dia_mm must be ≥ 0");
    }

    if (in.inner_dia_mm > 0.0) {
        if (in.inner_dia_mm >= in.outer_dia_mm) {
            r.add("DFM-BOSS-WALL", "error",
                  "mold_boss inner_dia ≥ outer_dia");
        } else {
            const double wallTotal = in.outer_dia_mm - in.inner_dia_mm;
            if (wallTotal < 1.2) {
                r.add("DFM-BOSS-WALL", "error",
                      "mold_boss wall (outer-inner) " + std::to_string(wallTotal) +
                      " mm < 1.2 mm (each side wall must be ≥ 0.6 mm)");
            }
        }
    }

    if (in.draft_angle_deg < 0.5) {
        r.add("DFM-BOSS-DRAFT", "warning",
              "mold_boss draft_angle " + std::to_string(in.draft_angle_deg) +
              " deg < 0.5 — mould release risk");
    }

    if (in.outer_dia_mm > 0.0 && in.height_mm / in.outer_dia_mm > 4.0) {
        r.add("DFM-BOSS-HEIGHT", "warning",
              "mold_boss height/outer_dia ratio " +
              std::to_string(in.height_mm / in.outer_dia_mm) +
              " > 4 — tall narrow boss is unstable");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mold_boss DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("mold_boss: entry_face datum unresolved");

    BRepAdaptor_Surface entrySurf(wp.face(*entryId));
    if (entrySurf.GetType() != GeomAbs_Plane) {
        throw SkillError("mold_boss: entry_face must be planar");
    }
    const gp_Pln entryPlane = entrySurf.Plane();
    gp_Dir outward = entryPlane.Axis().Direction();
    if (wp.face(*entryId).Orientation() == TopAbs_REVERSED) outward.Reverse();

    // Project (position_x, position_y, 0) onto entry plane.
    auto projectOntoPlane = [&](double x, double y) {
        const gp_Pnt P0 = entryPlane.Location();
        const gp_Dir n  = entryPlane.Axis().Direction();
        if (std::abs(n.Z()) < 1e-9) {
            return gp_Pnt(x, y, P0.Z());
        }
        const double z = P0.Z() - (n.X() * (x - P0.X()) + n.Y() * (y - P0.Y())) / n.Z();
        return gp_Pnt(x, y, z);
    };
    const gp_Pnt basePnt = projectOntoPlane(in.position_x_mm, in.position_y_mm);

    // Build outer boss.  If draft_angle > 0, use a cone frustum (wider at
    // base, narrower at top).  Otherwise use a straight cylinder.
    //
    // The boss base sits INSIDE the workpiece by `kEmbed` so that the fuse
    // produces a clean welded shape (avoids coplanar-face Boolean issues).
    const double draftRad = in.draft_angle_deg * M_PI / 180.0;
    const double outerR  = in.outer_dia_mm / 2.0;
    const double outerRTop = std::max(outerR - in.height_mm * std::tan(draftRad), 0.1);

    constexpr double kEmbed = 0.02;
    const gp_Pnt embeddedBase(
        basePnt.X() - outward.X() * kEmbed,
        basePnt.Y() - outward.Y() * kEmbed,
        basePnt.Z() - outward.Z() * kEmbed);
    const gp_Ax2 bossAx(embeddedBase, outward);
    TopoDS_Shape boss;
    if (in.draft_angle_deg > 1e-3 && std::abs(outerR - outerRTop) > 1e-4) {
        boss = pr::coneFrustum(bossAx, outerR, outerRTop, in.height_mm + kEmbed);
    } else {
        boss = pr::cylinder(bossAx, outerR, in.height_mm + kEmbed);
    }

    // Fuse boss onto workpiece.
    TopoDS_Shape afterFuse = pr::fuse(wp.shape(), boss);

    // If hollow, drill an inner cylinder from the top.
    if (in.inner_dia_mm > 0.0) {
        const double innerR = in.inner_dia_mm / 2.0;
        // Start the inner cutter from slightly above the boss top, extend
        // down to within a small wall floor (depth = height - 0.5 mm) for a
        // capped hole.  For slice-1, leave a floor of 0.5 mm (typical
        // self-tap pilot hole leaves a small webbed bottom for solidity).
        const double kFloor = std::min(0.5, in.height_mm * 0.2);
        const double cutDepth = in.height_mm - kFloor;
        const double kOverhang = 0.05;
        // Top of boss = basePnt + outward * height.
        const gp_Pnt topPnt(
            basePnt.X() + outward.X() * (in.height_mm + kOverhang),
            basePnt.Y() + outward.Y() * (in.height_mm + kOverhang),
            basePnt.Z() + outward.Z() * (in.height_mm + kOverhang));
        // Inner cutter axis goes from top down (-outward) into the boss.
        const gp_Dir inward(-outward.X(), -outward.Y(), -outward.Z());
        const gp_Ax2 innerAx(topPnt, inward);
        const TopoDS_Shape innerCut = pr::cylinder(innerAx, innerR, cutDepth + kOverhang);
        afterFuse = pr::cut(afterFuse, innerCut);
    }

    json params = {
        { "entry_face_kind",  "resolved_id" },
        { "entry_face_id",    *entryId },
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
        { "height_mm",        in.height_mm },
        { "outer_dia_mm",     in.outer_dia_mm },
        { "inner_dia_mm",     in.inner_dia_mm },
        { "draft_angle_deg",  in.draft_angle_deg },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "outer_dia_mm",     in.outer_dia_mm },
        { "inner_dia_mm",     in.inner_dia_mm },
        { "height_mm",        in.height_mm },
        { "is_hollow",        in.inner_dia_mm > 0.0 },
        { "axis_dir",         { outward.X(), outward.Y(), outward.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "mold_feature";
    const double outerArea = M_PI * outerR * outerR;
    const double innerArea = (in.inner_dia_mm > 0.0)
        ? M_PI * (in.inner_dia_mm / 2.0) * (in.inner_dia_mm / 2.0)
        : 0.0;
    tooling.stock_removed_mm3 = -(outerArea - innerArea) * in.height_mm;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(afterFuse, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::mold_boss applied: dia={} (inner={}) height={} faces {}→{}",
                  in.outer_dia_mm, in.inner_dia_mm, in.height_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Boss = a cylindrical face whose axis is parallel to a planar base face's
// normal, with a planar top cap.  We replay history first (highest fidelity)
// and fall back to topology-based heuristics.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "position_x_mm",   f.params.value("position_x_mm",   0.0) },
            { "position_y_mm",   f.params.value("position_y_mm",   0.0) },
            { "height_mm",       f.params.value("height_mm",       0.0) },
            { "outer_dia_mm",    f.params.value("outer_dia_mm",    0.0) },
            { "inner_dia_mm",    f.params.value("inner_dia_mm",    0.0) },
            { "draft_angle_deg", f.params.value("draft_angle_deg", 0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.95,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Topology-based: pair each vertical cylindrical face with a planar
    // disc cap.  Heuristic: cylinder axis (almost) vertical, has at least
    // one circular bounding edge that bounds a small circular planar face
    // strictly above the workpiece's main top.
    const auto wpBb = pr::optimalBbox(wp.shape());

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface surf(wp.face(fIdx));
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir adir = cyl.Axis().Direction();
        if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-2) continue;
        const double radius = cyl.Radius();
        if (radius < 0.75 || radius > 25.0) continue;       // boss-sized

        // Find circular edges and the highest one — if it's strictly above
        // wpBb.zMax of the un-bossed stock, this is likely a boss top.
        double zMax = -1e30, zMin = 1e30;
        for (TopExp_Explorer exp(wp.face(fIdx), TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Pnt midP = crv.Value((crv.FirstParameter() + crv.LastParameter()) / 2.0);
            zMax = std::max(zMax, midP.Z());
            zMin = std::min(zMin, midP.Z());
        }
        if (zMax < -1e29) continue;
        const double bossHeight = zMax - zMin;
        if (bossHeight < 0.5) continue;
        // The boss must protrude above the base — top of cylinder strictly
        // above some neighboring planar Z.  Approximate via wp bbox top:
        // boss must reach near workpiece zMax.
        if (zMax < wpBb.zMax - 1e-3) continue;

        // Position = axis location in XY at the base.
        const gp_Pnt axBase = cyl.Axis().Location();

        json rec = {
            { "position_x_mm",   axBase.X() },
            { "position_y_mm",   axBase.Y() },
            { "height_mm",       bossHeight },
            { "outer_dia_mm",    2.0 * radius },
            { "inner_dia_mm",    0.0 },
            { "draft_angle_deg", 0.0 },
        };
        json matched = {
            { "outer_cyl_face_id", fIdx },
            { "z_top",             zMax },
            { "z_bottom",          zMin },
        };
        out.push_back(RecognizedFeature{ kSkillId, rec, 0.7, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::mold_boss
