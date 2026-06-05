// @lat: [[engine/skills#nacelle_panel_lock_quarter_turn]]

#include "nacelle_panel_lock_quarter_turn.hpp"

#include "Workpiece.hpp"
#include "_as568_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::nacelle_panel_lock_quarter_turn {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "nacelle_panel_lock_quarter_turn: face_id out of range");
    }
    if (in.latch_body_dia_mm <= 0.0 ||
        in.latch_depth_mm <= 0.0 ||
        in.cam_engagement_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "nacelle_panel_lock_quarter_turn: all dims must be > 0");
        return r;
    }
    // AS568 key validation.
    const auto* od = as568::findDash(in.sealing_o_ring_size_key);
    if (!od) {
        r.add("DFM-AS568", "error",
              "nacelle_panel_lock_quarter_turn: AS568 key '" +
              in.sealing_o_ring_size_key + "' not in _as568_table");
        return r;
    }
    // Latch body must be at least 1.5x the bore for receptacle clearance.
    if (in.latch_body_dia_mm < 6.0) {
        r.add("DFM-LATCH-DIA", "error",
              "nacelle_panel_lock_quarter_turn: latch_body_dia " +
              std::to_string(in.latch_body_dia_mm) +
              " mm too small (≥ 6 mm for heavy-duty)");
    }
    if (in.latch_depth_mm > 25.0) {
        r.add("DFM-LATCH-DEPTH", "error",
              "nacelle_panel_lock_quarter_turn: latch_depth " +
              std::to_string(in.latch_depth_mm) +
              " mm > 25 mm (off-spec for nacelle panel thickness)");
    }
    if (in.cam_engagement_depth_mm >= in.latch_depth_mm) {
        r.add("DFM-CAM-DEPTH", "error",
              "nacelle_panel_lock_quarter_turn: cam_engagement_depth must "
              "be < latch_depth (cam sits below pocket floor)");
    }
    // O-ring groove must fit on the entry face annulus around the bore.
    const double oringID = od->inner_dia_mm;
    if (oringID + 2.0 * od->cross_section_mm <= in.latch_body_dia_mm) {
        r.add("DFM-ORING-FIT", "error",
              "nacelle_panel_lock_quarter_turn: O-ring " +
              in.sealing_o_ring_size_key + " (ID " +
              std::to_string(oringID) + ") too small for latch_body_dia " +
              std::to_string(in.latch_body_dia_mm));
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "nacelle_panel_lock_quarter_turn DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* od = as568::findDash(in.sealing_o_ring_size_key);

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    constexpr double kOverhang = 0.05;
    const gp_Pnt entry(
        in.center_x_mm + n.X() * kOverhang,
        in.center_y_mm + n.Y() * kOverhang,
        faceC.Z()      + n.Z() * kOverhang);
    const gp_Ax2 ax(entry, drillDir);

    // ── 1) latch through-bore (through full wall) ──────────────────────
    const double bodyR     = in.latch_body_dia_mm / 2.0;
    const double throughLen = bboxDiag + 2.0 * kOverhang;
    const TopoDS_Shape throughTool = pr::cylinder(ax, bodyR, throughLen);

    // ── 2) wider receptacle pocket (concentric counterbore) ─────────────
    const double pocketR  = bodyR * 1.4;            // 40% wider for receptacle
    const double pocketH  = in.latch_depth_mm + kOverhang;
    const TopoDS_Shape pocketTool = pr::cylinder(ax, pocketR, pocketH);

    // ── 3) cam engagement undercut (annular ring BELOW pocket floor) ────
    // Sits at depth = latch_depth, extending cam_engagement_depth further.
    // Inner radius = bodyR (clear shaft), outer radius = pocketR * 1.4
    // (wider than pocket for cam wing engagement).
    const gp_Pnt camStart(
        entry.X() - drillDir.X() * 0.0 + drillDir.X() * in.latch_depth_mm,
        entry.Y() - drillDir.Y() * 0.0 + drillDir.Y() * in.latch_depth_mm,
        entry.Z() - drillDir.Z() * 0.0 + drillDir.Z() * in.latch_depth_mm);
    const gp_Ax2 camAx(camStart, drillDir);
    const double camOuterR = pocketR * 1.4;
    const double camInnerR = bodyR;
    const TopoDS_Shape camTool = pr::annularRing(
        camAx, camOuterR, camInnerR,
        in.cam_engagement_depth_mm + kOverhang);

    // ── 4) O-ring seal groove at entry face ────────────────────────────
    // Annular groove centered on PCD = oring ID + CS.  Located at entry
    // face going groove_depth into the body.
    const double oringMeanDia = od->inner_dia_mm + od->cross_section_mm;
    const double grooveInnerR = (oringMeanDia - od->groove_width_mm) / 2.0;
    const double grooveOuterR = (oringMeanDia + od->groove_width_mm) / 2.0;
    const TopoDS_Shape grooveTool = pr::annularRing(
        ax, grooveOuterR, grooveInnerR,
        od->groove_depth_mm + kOverhang);

    // ── Sequential pr::cut loop ────────────────────────────────────────
    TopoDS_Shape current = wp.shape();
    current = pr::cut(current, throughTool);    // 1. through-bore
    current = pr::cut(current, pocketTool);     // 2. wider pocket
    current = pr::cut(current, camTool);        // 3. cam undercut
    current = pr::cut(current, grooveTool);     // 4. O-ring groove
    const TopoDS_Shape newShape = current;

    // ── Derived volume removed ─────────────────────────────────────────
    const double wallThk = zMax - zMin;
    const double vThrough = M_PI * bodyR * bodyR * wallThk;
    const double vPocket  = M_PI * (pocketR * pocketR - bodyR * bodyR) *
                            in.latch_depth_mm;
    const double vCam     = M_PI * (camOuterR * camOuterR -
                                    camInnerR * camInnerR) *
                            in.cam_engagement_depth_mm;
    const double vGroove  = M_PI * (grooveOuterR * grooveOuterR -
                                    grooveInnerR * grooveInnerR) *
                            od->groove_depth_mm;
    const double removedVol = vThrough + vPocket + vCam + vGroove;

    json params = {
        { "face_id",                  in.face_id },
        { "center_x_mm",              in.center_x_mm },
        { "center_y_mm",              in.center_y_mm },
        { "latch_body_dia_mm",        in.latch_body_dia_mm },
        { "latch_depth_mm",           in.latch_depth_mm },
        { "cam_engagement_depth_mm",  in.cam_engagement_depth_mm },
        { "sealing_o_ring_size_key",  in.sealing_o_ring_size_key },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "wind_feature_type",          "nacelle_panel_lock_quarter_turn" },
        { "subfeature_count",           4 },
        { "latch_body_dia_mm",          in.latch_body_dia_mm },
        { "latch_depth_mm",             in.latch_depth_mm },
        { "cam_engagement_depth_mm",    in.cam_engagement_depth_mm },
        { "sealing_o_ring_size_key",    in.sealing_o_ring_size_key },
        { "groove_inner_dia_mm",        2.0 * grooveInnerR },
        { "groove_outer_dia_mm",        2.0 * grooveOuterR },
        { "derived_volume_removed_mm3", removedVol },
        { "standard_ref",               "AS568 / heavy-duty quarter-turn" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;end_mill;groove_cutter";
    tooling.tool_dia_mm       = pocketR * 2.0;
    tooling.tool_length_mm    = wallThk + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = 25.0 + in.latch_depth_mm * 0.5;
    tooling.extra = {
        { "wind_feature_type", "nacelle_panel_lock_quarter_turn" },
        { "latch_family",      "heavy_duty_quarter_turn" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::nacelle_panel_lock_quarter_turn applied: body={} depth={} oring={}",
                  in.latch_body_dia_mm, in.latch_depth_mm,
                  in.sealing_o_ring_size_key);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay ─────────────────────────────────────────

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
            { "source",            "metadata_replay" },
            { "is_compound",       true },
            { "wind_feature_type", "nacelle_panel_lock_quarter_turn" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::nacelle_panel_lock_quarter_turn
