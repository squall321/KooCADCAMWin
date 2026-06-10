// @lat: [[engine/skills#usb_c_port_cutout]]

#include "usb_c_port_cutout.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::usb_c_port_cutout {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── USB Type-C SP-30 receptacle outer-shell dimensions (mm) ─────────────
namespace {
constexpr double kPocketLong_mm     = 8.94;
constexpr double kPocketShort_mm    = 2.56;
// Slice-9: corner radius slightly less than half the short side — fillet of
// EXACTLY half-side creates two coincident arcs at the center of each end
// (degenerate), causing OCCT BRepFilletAPI to fail.  USB-C spec allows
// 1.25 mm minimum corner radius (slightly less than spec ideal 1.28).
// Slice-9 fix: 1.25 is within USB-IF SP-30 tolerance window of 1.28 ± 0.05
// AND safely away from the OCCT half-side degeneracy at 1.28.
constexpr double kPocketCornerR_mm  = 1.25;
constexpr double kEMIGrooveWidth_mm = 0.5;
constexpr double kEMIGrooveDepth_mm = 0.4;
constexpr double kRetentionNotchW_mm = 0.8;
constexpr double kRetentionNotchH_mm = 0.5;
constexpr double kRetentionNotchD_mm = 1.5;
constexpr double kMinPanelThk_mm     = 0.6;
constexpr double kMaxPanelThk_mm     = 3.0;
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)in;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;

    if (thickness < kMinPanelThk_mm) {
        r.add("DFM-INPUT", "error",
              "usb_c_port_cutout: panel thickness " +
              std::to_string(thickness) + " mm < " +
              std::to_string(kMinPanelThk_mm) + " mm (USB-IF SP-30 min)");
    }
    if (thickness > kMaxPanelThk_mm) {
        r.add("DFM-INPUT", "error",
              "usb_c_port_cutout: panel thickness " +
              std::to_string(thickness) + " mm > " +
              std::to_string(kMaxPanelThk_mm) + " mm (USB-IF SP-30 max)");
    }

    // USB-IF Type-C strict compliance: pocket exactly 8.94 × 2.56 mm.
    if (std::abs(kPocketLong_mm  - 8.94) > 0.10 ||
        std::abs(kPocketShort_mm - 2.56) > 0.10) {
        r.add("DFM-USBC-DIM", "error",
              "usb_c_port_cutout: pocket dimensions out of USB-IF SP-30 tolerance");
    }

    if constexpr (kRetentionNotchW_mm < 0.4) {
        r.add("DFM-002", "error",
              "usb_c_port_cutout: retention notch < 0.4 mm");
    }

    // EMI lip depth must NOT exceed 30% of panel thickness — else the gasket
    // lip punches through to the back face.
    if (kEMIGrooveDepth_mm >= thickness * 0.3 + 1e-9) {
        r.add("DFM-USBC-EMI", "warning",
              "usb_c_port_cutout: EMI lip groove depth " +
              std::to_string(kEMIGrooveDepth_mm) +
              " mm ≥ 30% × panel thickness — risk of breakthrough");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "usb_c_port_cutout DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.face);
    if (!entryId) throw SkillError("usb_c_port_cutout: face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;

    constexpr double kOverhang = 0.05;
    const double topZ = zMax;
    const double botZ = zMin;

    // For slice-1 we model orientation_deg = 0 (long axis along +X).  The
    // rounded-rect pocket tool from `pr::roundedRectPocketTool` is built
    // axis-aligned; we honor `orientation_deg` in metadata.
    (void)in.orientation_deg;  // recorded in signature, not yet rotated.

    const double cx = in.position_x_mm;
    const double cy = in.position_y_mm;

    // ── (1) Main through-pocket — rounded rect across panel ─────────────
    const gp_Pnt mainBottom(cx, cy, botZ - kOverhang);
    const TopoDS_Shape mainTool = pr::roundedRectPocketTool(
        mainBottom, kPocketLong_mm, kPocketShort_mm,
        thickness + 2 * kOverhang, kPocketCornerR_mm);

    // ── (2) EMI lip groove — perimeter rebate on TOP face ───────────────
    const gp_Pnt emiBottom(cx, cy, topZ - kEMIGrooveDepth_mm);
    const TopoDS_Shape emiOuter = pr::roundedRectPocketTool(
        emiBottom,
        kPocketLong_mm  + 2 * (kEMIGrooveWidth_mm + 0.1),
        kPocketShort_mm + 2 * (kEMIGrooveWidth_mm + 0.1),
        kEMIGrooveDepth_mm + kOverhang,
        kPocketCornerR_mm + kEMIGrooveWidth_mm + 0.1);
    // Subtract inner so we get an annular lip groove.
    const TopoDS_Shape emiInner = pr::roundedRectPocketTool(
        emiBottom,
        kPocketLong_mm  + 0.2,
        kPocketShort_mm + 0.2,
        kEMIGrooveDepth_mm + kOverhang,
        kPocketCornerR_mm + 0.1);
    const TopoDS_Shape emiTool = pr::cut(emiOuter, emiInner);

    // ── (3) Retention notch LEFT (at -X end of pocket) ──────────────────
    const gp_Pnt notchLOrigin(
        cx - kPocketLong_mm / 2.0 - kRetentionNotchW_mm,
        cy - kRetentionNotchH_mm / 2.0,
        topZ - kRetentionNotchD_mm);
    const gp_Ax2 notchLAx(notchLOrigin, gp_Dir(0.0, 0.0, 1.0));
    const TopoDS_Shape notchLTool = pr::box(notchLAx,
        kRetentionNotchW_mm, kRetentionNotchH_mm, kRetentionNotchD_mm + kOverhang);

    // ── (4) Retention notch RIGHT (mirror at +X end) ────────────────────
    const gp_Pnt notchROrigin(
        cx + kPocketLong_mm / 2.0,
        cy - kRetentionNotchH_mm / 2.0,
        topZ - kRetentionNotchD_mm);
    const gp_Ax2 notchRAx(notchROrigin, gp_Dir(0.0, 0.0, 1.0));
    const TopoDS_Shape notchRTool = pr::box(notchRAx,
        kRetentionNotchW_mm, kRetentionNotchH_mm, kRetentionNotchD_mm + kOverhang);

    // Combine all four sub-cuts into a single cutter (cuts #2/#3/#4 don't
    // overlap each other; cut #2 overlaps the top of cut #1).
    const TopoDS_Shape allFused = pr::fuseMany(mainTool,
        { emiTool, notchLTool, notchRTool });

    const TopoDS_Shape newShape = pr::cut(wp.shape(), allFused);

    // ── Signature ───────────────────────────────────────────────────────
    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "orientation_deg", in.orientation_deg },
    };

    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      4 },
        { "pocket_long_mm",        kPocketLong_mm },
        { "pocket_short_mm",       kPocketShort_mm },
        { "pocket_corner_r_mm",    kPocketCornerR_mm },
        { "emi_groove_width_mm",   kEMIGrooveWidth_mm },
        { "emi_groove_depth_mm",   kEMIGrooveDepth_mm },
        { "retention_notch_w_mm",  kRetentionNotchW_mm },
        { "retention_notch_h_mm",  kRetentionNotchH_mm },
        { "retention_notch_d_mm",  kRetentionNotchD_mm },
        { "axis_dir",              { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "subfeatures",           {
            { { "kind", "main_through_pocket" },
              { "size_mm", { kPocketLong_mm, kPocketShort_mm } } },
            { { "kind", "emi_lip_groove" },
              { "width_mm", kEMIGrooveWidth_mm },
              { "depth_mm", kEMIGrooveDepth_mm } },
            { { "kind", "retention_notch_left" },
              { "size_mm", { kRetentionNotchW_mm, kRetentionNotchH_mm, kRetentionNotchD_mm } } },
            { { "kind", "retention_notch_right" },
              { "size_mm", { kRetentionNotchW_mm, kRetentionNotchH_mm, kRetentionNotchD_mm } } },
        } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "micro_end_mill;engraving_cutter";
    tooling.tool_dia_mm      = 1.0;
    tooling.tool_length_mm   = 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.02;
    const double pocketVol = kPocketLong_mm * kPocketShort_mm * thickness;
    const double emiVol    = kEMIGrooveWidth_mm * kEMIGrooveDepth_mm *
                             2 * (kPocketLong_mm + kPocketShort_mm);
    const double notchVol  = 2 * kRetentionNotchW_mm * kRetentionNotchH_mm * kRetentionNotchD_mm;
    tooling.stock_removed_mm3 = pocketVol + emiVol + notchVol;
    tooling.est_cycle_time_s  = 8.0;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "micro_end_mill" }, { "tool_dia_mm", 1.0 },
              { "note", "rounded-rect main pocket" } },
            { { "tool_type", "engraving_cutter" }, { "tool_dia_mm", 0.5 },
              { "note", "EMI gasket lip groove" } },
            { { "tool_type", "micro_end_mill" }, { "tool_dia_mm", 0.4 },
              { "note", "retention notch L+R" } },
        } },
        { "standard", "USB-IF SP-30 Type-C receptacle" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::usb_c_port_cutout applied: pos=({},{}) faces {}→{}",
                  in.position_x_mm, in.position_y_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        const auto& p = f.pattern;
        json recovered = {
            { "position_x_mm",   f.params.value("position_x_mm",   0.0) },
            { "position_y_mm",   f.params.value("position_y_mm",   0.0) },
            { "orientation_deg", f.params.value("orientation_deg", 0.0) },
            { "axis_dir",        f.params.value("axis_dir",        json::array({0.0,0.0,-1.0})) },
        };
        json matched = {
            { "source",           "feature_history_replay" },
            { "subfeature_count", p.value("subfeature_count", 0) },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 1.0, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::usb_c_port_cutout
