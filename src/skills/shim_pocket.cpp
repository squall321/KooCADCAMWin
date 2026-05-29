// @lat: [[engine/skills#shim_pocket]]

#include "shim_pocket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::shim_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.shim_thk_mm < 0.05 || in.shim_thk_mm > 5.0) {
        r.add("DFM-SHIM-THK", "error",
              "shim_pocket: shim_thk_mm " + std::to_string(in.shim_thk_mm) +
              " mm outside DIN 988 range [0.05, 5.0] mm");
    }
    if (in.shim_w_mm < 4.0 || in.shim_l_mm < 4.0) {
        r.add("DFM-SHIM-SIZE", "error",
              "shim_pocket: shim_w or shim_l < 4 mm "
              "(impractical for handling/seating)");
    }
    if (in.shim_w_mm <= 0.0 || in.shim_l_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "shim_pocket: shim_w_mm and shim_l_mm must be > 0");
    }
    if (in.retention_step_inset_mm * 4.0 >=
        std::min(in.shim_w_mm, in.shim_l_mm)) {
        r.add("DFM-SHIM-STEP", "error",
              "shim_pocket: retention_step_inset " +
              std::to_string(in.retention_step_inset_mm) +
              " mm ≥ min(shim_w, shim_l)/4 — step swallows pocket floor");
    }
    if (in.retention_step_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "shim_pocket: retention_step_depth must be > 0");
    }
    if (in.tab_notch_w_mm < 0.8) {
        r.add("DFM-002", "error",
              "shim_pocket: tab_notch_w " +
              std::to_string(in.tab_notch_w_mm) + " mm < 0.8 mm (min end-mill)");
    }
    if (in.tab_notch_d_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "shim_pocket: tab_notch_d must be > 0");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "shim_pocket DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError(
        "shim_pocket: entry_face datum unresolved");

    if (std::abs(in.axis_dir.X()) > 1e-6 || std::abs(in.axis_dir.Y()) > 1e-6
        || in.axis_dir.Z() >= 0) {
        throw SkillError("shim_pocket: only axis_dir = -Z supported in slice 1");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // Pocket depth is shim thickness + small radial clearance, but radial
    // clearance is on the XY not Z; effective Z pocket depth ≈ shim_thk +
    // 0.05 (axial slip-fit allowance).
    const double pocketDepth = in.shim_thk_mm + 0.05;

    const double pocketCenterX = in.position_x_mm;
    const double pocketCenterY = in.position_y_mm;

    // ── Sub-feature 1: main shim pocket (box) ──────────────────────────
    const double pocketL = in.shim_l_mm + 0.10;   // +0.10 mm slip fit
    const double pocketW = in.shim_w_mm + 0.10;

    const gp_Pnt mainOrigin(
        pocketCenterX - pocketL / 2.0,
        pocketCenterY - pocketW / 2.0,
        zMax - pocketDepth);
    const gp_Ax2 mainAx(mainOrigin, gp::DZ());
    const TopoDS_Shape mainCutter =
        pr::box(mainAx, pocketL, pocketW, pocketDepth + 0.05);

    // ── Sub-feature 2: retention edge step ─────────────────────────────
    // A shallower narrower pocket below the main pocket floor; depth =
    // retention_step_depth, footprint inset by retention_step_inset from
    // the main pocket.
    const double stepL = pocketL - 2.0 * in.retention_step_inset_mm;
    const double stepW = pocketW - 2.0 * in.retention_step_inset_mm;
    const gp_Pnt stepOrigin(
        pocketCenterX - stepL / 2.0,
        pocketCenterY - stepW / 2.0,
        zMax - pocketDepth - in.retention_step_depth_mm);
    const gp_Ax2 stepAx(stepOrigin, gp::DZ());
    const TopoDS_Shape stepCutter =
        pr::box(stepAx, stepL, stepW, in.retention_step_depth_mm + 0.05);

    // ── Sub-features 3 + 4: retention tabs (diametrically opposed) ─────
    // Two small notches on the short sides (X-min and X-max of the pocket),
    // centered along Y.  Each notch is tab_notch_w_mm wide × tab_notch_d_mm
    // deep × pocketDepth height (so it pierces the full pocket wall).
    const double tabH = pocketDepth + 0.05;

    // Tab A — on the -X side, extending FURTHER in -X by tab_notch_d.
    const gp_Pnt tabAOrigin(
        pocketCenterX - pocketL / 2.0 - in.tab_notch_d_mm,
        pocketCenterY - in.tab_notch_w_mm / 2.0,
        zMax - tabH);
    const gp_Ax2 tabAAx(tabAOrigin, gp::DZ());
    const TopoDS_Shape tabA =
        pr::box(tabAAx, in.tab_notch_d_mm + 0.05, in.tab_notch_w_mm, tabH);

    // Tab B — on the +X side.
    const gp_Pnt tabBOrigin(
        pocketCenterX + pocketL / 2.0 - 0.05,
        pocketCenterY - in.tab_notch_w_mm / 2.0,
        zMax - tabH);
    const gp_Ax2 tabBAx(tabBOrigin, gp::DZ());
    const TopoDS_Shape tabB =
        pr::box(tabBAx, in.tab_notch_d_mm + 0.05, in.tab_notch_w_mm, tabH);

    // Fuse all four cutters into a single tool, then single cut.
    const TopoDS_Shape fused1 = pr::fuse(mainCutter, stepCutter);
    const TopoDS_Shape fused2 = pr::fuse(fused1, tabA);
    const TopoDS_Shape fused3 = pr::fuse(fused2, tabB);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused3);

    json params = {
        { "entry_face_id",            *entryId },
        { "position_x_mm",            in.position_x_mm },
        { "position_y_mm",            in.position_y_mm },
        { "axis_dir",                 { in.axis_dir.X(), in.axis_dir.Y(),
                                         in.axis_dir.Z() } },
        { "shim_thk_mm",              in.shim_thk_mm },
        { "shim_w_mm",                in.shim_w_mm },
        { "shim_l_mm",                in.shim_l_mm },
        { "retention_step_depth_mm",  in.retention_step_depth_mm },
        { "retention_step_inset_mm",  in.retention_step_inset_mm },
        { "tab_notch_w_mm",           in.tab_notch_w_mm },
        { "tab_notch_d_mm",           in.tab_notch_d_mm },
    };
    json pattern = {
        { "kind",                     kSkillId },
        { "is_compound",              true },
        { "subfeature_count",         4 },
        { "shim_thk_mm",              in.shim_thk_mm },
        { "shim_w_mm",                in.shim_w_mm },
        { "shim_l_mm",                in.shim_l_mm },
        { "pocket_l_mm",              pocketL },
        { "pocket_w_mm",              pocketW },
        { "pocket_depth_mm",          pocketDepth },
        { "retention_step_depth_mm",  in.retention_step_depth_mm },
        { "retention_step_inset_mm",  in.retention_step_inset_mm },
        { "retention_step_l_mm",      stepL },
        { "retention_step_w_mm",      stepW },
        { "tab_notch_w_mm",           in.tab_notch_w_mm },
        { "tab_notch_d_mm",           in.tab_notch_d_mm },
        { "axis_dir",                 { in.axis_dir.X(), in.axis_dir.Y(),
                                         in.axis_dir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = std::min(in.tab_notch_w_mm, 3.0);
    tooling.tool_length_mm    = pocketDepth + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 =
        pocketL * pocketW * pocketDepth
        + stepL * stepW * in.retention_step_depth_mm
        + 2.0 * in.tab_notch_d_mm * in.tab_notch_w_mm * pocketDepth;
    tooling.est_cycle_time_s = std::max(5.0,
        pocketL * pocketW * 0.0008 + 4.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", std::min(in.tab_notch_w_mm, 3.0) },
              { "depth_mm",    pocketDepth },
              { "purpose",     "main shim pocket (DIN 988 slip fit)" } },
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", std::min(in.tab_notch_w_mm, 3.0) },
              { "depth_mm",    in.retention_step_depth_mm },
              { "purpose",     "retention edge step (rim seat relief)" } },
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", in.tab_notch_w_mm },
              { "depth_mm",    pocketDepth },
              { "purpose",     "retention tab A" } },
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", in.tab_notch_w_mm },
              { "depth_mm",    pocketDepth },
              { "purpose",     "retention tab B (opposed)" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::shim_pocket applied: shim {}x{}x{} step {}x{} tabs {}x{}",
                  in.shim_l_mm, in.shim_w_mm, in.shim_thk_mm,
                  stepL, stepW,
                  in.tab_notch_w_mm, in.tab_notch_d_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition — metadata replay ────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            { { "via", "metadata_replay" },
              { "subfeature_count", 4 } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::shim_pocket
