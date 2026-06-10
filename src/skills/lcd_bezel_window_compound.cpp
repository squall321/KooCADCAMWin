// @lat: [[engine/skills#lcd_bezel_window_compound]]

#include "lcd_bezel_window_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace koocadcam::skill::lcd_bezel_window_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!(in.lcd_window_w_mm > 0.0) || !(in.lcd_window_h_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "lcd_bezel_window_compound: lcd_window_w_mm and h_mm must be > 0");
    }
    if (!(in.lcd_recess_depth_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "lcd_bezel_window_compound: lcd_recess_depth_mm must be > 0");
    }
    if (!(in.lcd_window_corner_radius_mm >= 0.5)) {
        r.add("DFM-LCD-CORNER", "error",
              "lcd_bezel_window_compound: corner_radius " +
              std::to_string(in.lcd_window_corner_radius_mm) +
              " < 0.5 mm (cutter floor; rounded-corner wire needs R >= 0.5)");
    }
    if (in.lcd_window_w_mm > 0.0 && in.lcd_window_h_mm > 0.0 &&
        in.lcd_window_corner_radius_mm >
            0.5 * std::min(in.lcd_window_w_mm, in.lcd_window_h_mm)) {
        r.add("DFM-LCD-CORNER-MAX", "error",
              "lcd_bezel_window_compound: corner_radius exceeds half of min(w,h)");
    }
    if (in.vent_slot_count != 0 && in.vent_slot_count != 4) {
        r.add("DFM-LCD-VENTS", "error",
              "lcd_bezel_window_compound: vent_slot_count must be 0 or 4 (got " +
              std::to_string(in.vent_slot_count) + ")");
    }
    // wp bbox sanity
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    if (in.lcd_recess_depth_mm > (zMax - zMin)) {
        r.add("DFM-INPUT", "error",
              "lcd_bezel_window_compound: lcd_recess_depth > wp thickness");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "lcd_bezel_window_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double zTop = zMax;
    constexpr double kEmbed = 0.05;

    // ── Sub-feature 1: rounded-corner LCD recess pocket ──────────────────
    const gp_Pnt recessBottomCenter(
        in.center_x_mm, in.center_y_mm,
        zTop - in.lcd_recess_depth_mm);
    const TopoDS_Shape recess = pr::roundedRectPocketTool(
        recessBottomCenter,
        in.lcd_window_w_mm,
        in.lcd_window_h_mm,
        in.lcd_recess_depth_mm + kEmbed,
        in.lcd_window_corner_radius_mm);

    TopoDS_Shape afterRecess = pr::cut(wp.shape(), recess);

    // ── Sub-feature 2: optional vent slots above LCD area ────────────────
    double ventsVol = 0.0;
    int actualVentSlots = 0;
    if (in.vent_slot_count == 4) {
        // Place a row of 4 narrow rectangular slots ABOVE (in +Y) the LCD,
        // spanning ~80 % of LCD width.  Each slot is 2 mm wide (along Y),
        // (lcd_w * 0.8 / 4) - gap long (along X), cut THROUGH the wp.
        const double rowWidth  = in.lcd_window_w_mm * 0.8;
        const double slotW     = rowWidth / 4.0 - 1.5;   // 1.5 mm gap between
        const double slotH     = 2.0;                    // 2 mm thick (along Y)
        const double rowY      = in.center_y_mm + in.lcd_window_h_mm * 0.5 + 4.0;
        const double leftX     = in.center_x_mm - rowWidth * 0.5;
        const double thru      = (zMax - zMin) + 2.0 * kEmbed;

        std::vector<TopoDS_Shape> cutters;
        cutters.reserve(4);
        for (int i = 0; i < 4; ++i) {
            const double sx = leftX + i * (slotW + 1.5);
            const gp_Pnt origin(sx, rowY - slotH * 0.5, zMin - kEmbed);
            const gp_Ax2 ax(origin, gp::DZ());
            cutters.push_back(pr::box(ax, slotW, slotH, thru));
            ventsVol += slotW * slotH * (zMax - zMin);
        }
        afterRecess = pr::cutMany(afterRecess, cutters);
        actualVentSlots = 4;
    }

    const double recessVol = in.lcd_window_w_mm * in.lcd_window_h_mm *
                             in.lcd_recess_depth_mm;
    const double removedVol = recessVol + ventsVol;

    json params = {
        { "face_id",                      in.face_id },
        { "center_x_mm",                  in.center_x_mm },
        { "center_y_mm",                  in.center_y_mm },
        { "lcd_window_w_mm",              in.lcd_window_w_mm },
        { "lcd_window_h_mm",              in.lcd_window_h_mm },
        { "lcd_window_corner_radius_mm",  in.lcd_window_corner_radius_mm },
        { "lcd_recess_depth_mm",          in.lcd_recess_depth_mm },
        { "vent_slot_count",              in.vent_slot_count },
    };
    json pattern_js = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "electronics_feature_type", "lcd_bezel_window" },
        { "subfeature_count",       1 + actualVentSlots },
        { "lcd_window_w_mm",        in.lcd_window_w_mm },
        { "lcd_window_h_mm",        in.lcd_window_h_mm },
        { "lcd_window_corner_radius_mm", in.lcd_window_corner_radius_mm },
        { "lcd_recess_depth_mm",    in.lcd_recess_depth_mm },
        { "vent_slot_count",        actualVentSlots },
        { "derived_volume_removed_mm3", removedVol },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = std::max(2.0, in.lcd_window_corner_radius_mm * 1.8);
    tooling.tool_length_mm    = in.lcd_recess_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(10.0, removedVol / 200.0);
    tooling.extra = {
        { "feature_standard",   "LCD bezel rounded-corner recess + vents" },
        { "vent_slot_count",    actualVentSlots },
        { "removed_volume_mm3", removedVol },
    };

    FeatureSignature sig { kSkillId, params, pattern_js, tooling };

    auto wpNew = std::make_shared<Workpiece>(afterRecess, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::lcd_bezel_window_compound applied: {}x{} mm recess R{}, vents={}",
        in.lcd_window_w_mm, in.lcd_window_h_mm,
        in.lcd_window_corner_radius_mm, actualVentSlots);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",      "metadata_replay" },
            { "is_compound", true },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::lcd_bezel_window_compound
