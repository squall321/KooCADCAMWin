// @lat: [[engine/skills#rubber_grommet_seat]]

#include "rubber_grommet_seat.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <Standard_Failure.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::rubber_grommet_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.hole_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "rubber_grommet_seat hole_dia " + std::to_string(in.hole_dia_mm) +
              " mm < min 0.8 mm");
    }
    if (in.hole_dia_mm < 3.0) {
        r.add("DFM-GROMMET", "error",
              "rubber_grommet_seat hole_dia " + std::to_string(in.hole_dia_mm) +
              " mm < 3 mm — too small for a standard panel grommet");
    }
    if (in.plate_t_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "rubber_grommet_seat plate_t_mm must be > 0");
    } else if (in.plate_t_mm < 1.2) {
        r.add("DFM-GROMMET", "error",
              "rubber_grommet_seat plate_t " + std::to_string(in.plate_t_mm) +
              " mm < 1.2 mm — no room for retention groove at mid-depth");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "rubber_grommet_seat DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("rubber_grommet_seat: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("rubber_grommet_seat: only axis_dir = -Z supported in slice-1");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    // ── Sub-feature dims ──────────────────────────────────────────────
    const double chamfer_d   = in.hole_dia_mm * 0.08;   // top chamfer chord
    const double groove_w    = 1.0;                     // retention groove width along axis
    const double groove_dr   = 0.4;                     // groove radial depth
    const double groove_od   = in.hole_dia_mm + 2.0 * groove_dr;
    const double groove_z    = zMax - in.plate_t_mm / 2.0;  // groove center Z (mid-depth)

    const gp_Pnt axisTop(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
    const gp_Ax2 toolAx(axisTop, in.axis_dir);

    // 1) Through hole at hole_dia.
    const double pilotH = in.plate_t_mm + 2.0 * kOverhang;
    const TopoDS_Shape pilotTool = pr::cylinder(toolAx, in.hole_dia_mm / 2.0, pilotH);

    // 2) Top chamfer — cone frustum from (hole_dia + 2×chamfer_d) at top
    //    down to (hole_dia) at depth = chamfer_d.  Implemented with
    //    prim::coneFrustum.  axis_dir is -Z, so we build a cone going
    //    downward of total height = chamfer_d.
    const gp_Pnt chamferTop(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
    const gp_Ax2 chamferAx(chamferTop, in.axis_dir);
    // gp_Ax2 with V = -Z means the local +Z runs in -Z direction; height
    // extends from the apex Z(=zMax+kOverhang) into the material.
    // r1 = at apex (top), r2 = at end (chamfer_d below).
    const double chamferTopR = in.hole_dia_mm / 2.0 + chamfer_d;
    const double chamferBotR = in.hole_dia_mm / 2.0;
    const TopoDS_Shape chamferTool = pr::coneFrustum(
        chamferAx, chamferTopR, chamferBotR, chamfer_d + kOverhang);

    // 3) Retention groove — annular ring located at mid-depth.  We build
    //    the annular ring axis-aligned with the bore, positioned so the
    //    groove sits at groove_z.  The annular ring is hole_dia → groove_od,
    //    height = groove_w (along axis).
    // Place the ring's bottom at groove_z - groove_w/2.  With axis_dir = -Z,
    // the cylinder extends from apex into -Z.  Compute apex position:
    const gp_Pnt grooveAxisPos(
        in.position_x_mm, in.position_y_mm, groove_z + groove_w / 2.0);
    const gp_Ax2 grooveAx(grooveAxisPos, in.axis_dir);
    const TopoDS_Shape grooveTool = pr::annularRing(
        grooveAx, groove_od / 2.0, in.hole_dia_mm / 2.0, groove_w);

    // Fuse all three concentric tools, then cut.
    TopoDS_Shape fused;
    try {
        fused = pr::fuse(pilotTool, chamferTool);
        fused = pr::fuse(fused, grooveTool);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("rubber_grommet_seat: fuse failed: ") + e.what());
    }

    TopoDS_Shape newShape;
    try {
        newShape = pr::cut(wp.shape(), fused);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("rubber_grommet_seat: cut failed: ") + e.what());
    }

    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "hole_dia_mm",     in.hole_dia_mm },
        { "plate_t_mm",      in.plate_t_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    3 },
        { "hole_dia_mm",         in.hole_dia_mm },
        { "plate_t_mm",          in.plate_t_mm },
        { "chamfer_d_mm",        chamfer_d },
        { "groove_w_mm",         groove_w },
        { "groove_dr_mm",        groove_dr },
        { "groove_od_mm",        groove_od },
        { "groove_z_mm",         groove_z },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type   = "drill;chamfer;groove";
    tooling.tool_dia_mm = in.hole_dia_mm;
    tooling.tool_material = "carbide";
    tooling.flute_count   = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double pilotV  = M_PI * (in.hole_dia_mm/2.0) * (in.hole_dia_mm/2.0) * in.plate_t_mm;
    const double chamV   = M_PI / 3.0 * chamfer_d * (chamferTopR * chamferTopR
                                                     + chamferTopR * chamferBotR
                                                     + chamferBotR * chamferBotR)
                          - M_PI * (in.hole_dia_mm/2.0)*(in.hole_dia_mm/2.0) * chamfer_d;
    const double grooveV = M_PI * ((groove_od/2.0)*(groove_od/2.0)
                                   - (in.hole_dia_mm/2.0)*(in.hole_dia_mm/2.0)) * groove_w;
    tooling.stock_removed_mm3 = std::max(0.0, pilotV + std::max(0.0, chamV) + grooveV);
    tooling.est_cycle_time_s = std::max(3.0, in.plate_t_mm / 40.0 + 2.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },   { "tool_dia_mm", in.hole_dia_mm },
              { "depth_mm", in.plate_t_mm + kOverhang } },
            { { "tool_type", "chamfer" }, { "tool_dia_mm", chamferTopR * 2.0 },
              { "depth_mm", chamfer_d } },
            { { "tool_type", "groove" },  { "tool_od_mm", groove_od },
              { "tool_id_mm", in.hole_dia_mm }, { "depth_mm", groove_w },
              { "groove_z_mm", groove_z } },
        } },
        { "standard", "DIN 89280 / VDE 0470 panel grommet seat" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::rubber_grommet_seat applied: hole={} plate={} faces {}→{}",
                  in.hole_dia_mm, in.plate_t_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
// Metadata replay only — the internal groove + chamfer combo is not
// reliably distinguishable from a fancy single-step bore by B-rep alone.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "feature_history_replay" } };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::rubber_grommet_seat
