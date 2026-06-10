// @lat: [[engine/skills#undercut_relief]]

#include "undercut_relief.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::undercut_relief {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "undercut_relief dia_mm must be > 0");
    } else if (in.dia_mm < 0.5 || in.dia_mm > 6.0) {
        r.add("DFM-UC-DIA", "error",
              "undercut_relief dia " + std::to_string(in.dia_mm) +
              " mm outside [0.5, 6.0] mm typical range");
    }

    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "undercut_relief depth_mm must be > 0");
    } else if (in.depth_mm < 0.3 || in.depth_mm > 8.0) {
        r.add("DFM-UC-DEPTH", "error",
              "undercut_relief depth " + std::to_string(in.depth_mm) +
              " mm outside [0.3, 8.0] mm typical range");
    }

    // axis_xy must be in XY plane (slide direction is always parallel to
    // parting plane).
    if (std::abs(in.axis_xy.Z()) > 1e-3) {
        r.add("DFM-UC-AXIS", "error",
              "undercut_relief axis_xy must be in XY plane "
              "(Z component must be 0)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "undercut_relief DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Normalize axis (allow non-unit input, but keep within XY).
    gp_Vec axVec(in.axis_xy);
    if (axVec.Magnitude() < 1e-9) {
        throw SkillError("undercut_relief: axis_xy has zero magnitude");
    }
    axVec.Normalize();
    const gp_Dir axDir(axVec.X(), axVec.Y(), 0.0);

    // Corner point in world coordinates (the parting-line corner where the
    // side action engages).  Push start slightly OUTSIDE the workpiece along
    // -axDir so the Boolean is clean (avoids coplanar-face issues).
    constexpr double kOverhang = 0.05;
    const gp_Pnt start(
        in.corner_x_mm - axDir.X() * kOverhang,
        in.corner_y_mm - axDir.Y() * kOverhang,
        in.corner_z_mm - axDir.Z() * kOverhang);

    // gp_Ax2 V = local Z (pitfall): the cylinder grows along `axDir`.
    const gp_Ax2 ax(start, axDir);
    const TopoDS_Shape cutter = pr::cylinder(ax, in.dia_mm / 2.0,
                                              in.depth_mm + 2.0 * kOverhang);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "corner_x_mm", in.corner_x_mm },
        { "corner_y_mm", in.corner_y_mm },
        { "corner_z_mm", in.corner_z_mm },
        { "axis_xy",     { in.axis_xy.X(), in.axis_xy.Y(), in.axis_xy.Z() } },
        { "dia_mm",      in.dia_mm },
        { "depth_mm",    in.depth_mm },
    };
    json pattern = {
        { "kind",      kSkillId },
        { "dia_mm",    in.dia_mm },
        { "depth_mm",  in.depth_mm },
        { "axis_dir",  { axDir.X(), axDir.Y(), axDir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "side_action_relief";
    tooling.tool_dia_mm      = in.dia_mm;
    tooling.tool_length_mm   = in.depth_mm;
    const double r = in.dia_mm / 2.0;
    tooling.stock_removed_mm3 = M_PI * r * r * in.depth_mm;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::undercut_relief applied: dia={} depth={} faces {}→{}",
                  in.dia_mm, in.depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay (primary).
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "corner_x_mm", f.params.value("corner_x_mm", 0.0) },
            { "corner_y_mm", f.params.value("corner_y_mm", 0.0) },
            { "corner_z_mm", f.params.value("corner_z_mm", 0.0) },
            { "axis_xy",     f.params.value("axis_xy", json::array({1.0,0.0,0.0})) },
            { "dia_mm",      f.params.value("dia_mm",   0.0) },
            { "depth_mm",    f.params.value("depth_mm", 0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.95, json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Topology fallback: small horizontal-axis cylindrical faces in the
    // undercut dia range.  axis_xy → cylinder.Axis().Direction().Z() ≈ 0.
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface surf(wp.face(fIdx));
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir adir = cyl.Axis().Direction();
        if (std::abs(adir.Z()) > 0.1) continue;       // axis must be (~) in XY
        const double dia = 2.0 * cyl.Radius();
        if (dia < 0.5 || dia > 6.0) continue;
        const gp_Pnt loc = cyl.Axis().Location();
        json rec = {
            { "corner_x_mm", loc.X() },
            { "corner_y_mm", loc.Y() },
            { "corner_z_mm", loc.Z() },
            { "axis_xy",     json::array({ adir.X(), adir.Y(), 0.0 }) },
            { "dia_mm",      dia },
            { "depth_mm",    0.0 },     // unknown without edge analysis
        };
        json matched = { { "cyl_face_id", fIdx } };
        out.push_back(RecognizedFeature{ kSkillId, rec, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::undercut_relief
