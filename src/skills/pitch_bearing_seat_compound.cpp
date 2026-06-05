// @lat: [[engine/skills#pitch_bearing_seat_compound]]

#include "pitch_bearing_seat_compound.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::pitch_bearing_seat_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "pitch_bearing_seat_compound: face_id out of range");
    }
    if (in.bearing_inner_dia_mm <= 0.0 ||
        in.bearing_outer_dia_mm <= 0.0 ||
        in.bearing_thickness_mm <= 0.0 ||
        in.bolt_pcd_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pitch_bearing_seat_compound: all dims must be > 0");
        return r;
    }
    if (in.bearing_outer_dia_mm <= in.bearing_inner_dia_mm) {
        r.add("DFM-BEARING-GEOM", "error",
              "pitch_bearing_seat_compound: outer_dia must be > inner_dia");
    }
    if (in.bolt_count <= 4) {
        r.add("DFM-BOLT-COUNT", "error",
              "pitch_bearing_seat_compound: bolt_count " +
              std::to_string(in.bolt_count) +
              " must be > 4 (utility-scale pitch bearing)");
    }
    // bearing thickness / outer dia ratio sanity (0.02 ≤ ratio ≤ 0.30).
    const double thkRatio = in.bearing_thickness_mm / in.bearing_outer_dia_mm;
    if (thkRatio < 0.02 || thkRatio > 0.30) {
        r.add("DFM-BEARING-RATIO", "error",
              "pitch_bearing_seat_compound: thickness/outer_dia ratio " +
              std::to_string(thkRatio) +
              " outside 0.02..0.30 utility-scale window");
    }
    // Thread key validation.
    const auto* spec = thread_table::findMetric(in.bolt_thread_size_key);
    if (!spec) {
        r.add("DFM-THREAD", "error",
              "pitch_bearing_seat_compound: thread key '" +
              in.bolt_thread_size_key +
              "' not in _iso_thread_table");
    }
    // PCD must sit outside the bearing outer dia.
    if (in.bolt_pcd_mm <= in.bearing_outer_dia_mm) {
        r.add("DFM-PCD", "error",
              "pitch_bearing_seat_compound: bolt_pcd must be > bearing_outer_dia");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pitch_bearing_seat_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = thread_table::findMetric(in.bolt_thread_size_key);
    // spec guaranteed non-null by validate.
    const double boltHoleDia = spec->clearance_medium_mm;

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());

    constexpr double kOverhang = 0.05;

    // ── Sub-feature 1: annular bearing seat pocket (single cut) ─────────
    const gp_Pnt seatEntry(
        in.center_x_mm + n.X() * kOverhang,
        in.center_y_mm + n.Y() * kOverhang,
        faceC.Z()      + n.Z() * kOverhang);
    const gp_Ax2 seatAx(seatEntry, drillDir);
    const double outerR = in.bearing_outer_dia_mm / 2.0;
    const double innerR = in.bearing_inner_dia_mm / 2.0;
    const TopoDS_Shape bearingSeatTool = pr::annularRing(
        seatAx, outerR, innerR,
        in.bearing_thickness_mm + 2.0 * kOverhang);

    TopoDS_Shape current = pr::cut(wp.shape(), bearingSeatTool);

    // ── Sub-feature 2..N+1: N bolt holes via SEQUENTIAL gp_Trsf rotation ─
    // Build a "template" bolt cylinder at theta=0 then rotate it for each i.
    const double pcdR    = in.bolt_pcd_mm / 2.0;
    const double boltR   = boltHoleDia / 2.0;
    const double boltDepth = in.bearing_thickness_mm * 1.5;  // bolt anchors below seat

    const gp_Pnt boltEntryTpl(
        in.center_x_mm + pcdR + n.X() * kOverhang,
        in.center_y_mm        + n.Y() * kOverhang,
        faceC.Z()             + n.Z() * kOverhang);
    const gp_Ax2 boltAxTpl(boltEntryTpl, drillDir);
    const TopoDS_Shape boltTemplate = pr::cylinder(
        boltAxTpl, boltR, boltDepth + 2.0 * kOverhang);

    // Rotation axis = face normal at PCD center.
    const gp_Pnt rotCenter(in.center_x_mm, in.center_y_mm, faceC.Z());
    const gp_Ax1 rotAxis(rotCenter, n);

    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        gp_Trsf rot;
        rot.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(boltTemplate, rot, true);
        if (!xform.IsDone()) {
            throw SkillError(
                "pitch_bearing_seat_compound: bolt rotation failed");
        }
        const TopoDS_Shape boltTool = xform.Shape();
        current = pr::cut(current, boltTool);  // sequential — no compound
    }
    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ──────────────────────────────────────
    const double vSeat = M_PI * (outerR * outerR - innerR * innerR) *
                         in.bearing_thickness_mm;
    const double vBolts = static_cast<double>(in.bolt_count) *
                          M_PI * boltR * boltR * boltDepth;
    const double removedVol = vSeat + vBolts;

    // Bolt positions JSON
    json bolts_json = json::array();
    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        bolts_json.push_back({
            { "x_mm",      in.center_x_mm + pcdR * std::cos(theta) },
            { "y_mm",      in.center_y_mm + pcdR * std::sin(theta) },
            { "theta_rad", theta },
        });
    }

    json params = {
        { "face_id",              in.face_id },
        { "center_x_mm",          in.center_x_mm },
        { "center_y_mm",          in.center_y_mm },
        { "bearing_inner_dia_mm", in.bearing_inner_dia_mm },
        { "bearing_outer_dia_mm", in.bearing_outer_dia_mm },
        { "bearing_thickness_mm", in.bearing_thickness_mm },
        { "bolt_count",           in.bolt_count },
        { "bolt_pcd_mm",          in.bolt_pcd_mm },
        { "bolt_thread_size_key", in.bolt_thread_size_key },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "wind_feature_type",          "pitch_bearing_seat" },
        { "bolt_count",                 in.bolt_count },
        { "bearing_inner_dia_mm",       in.bearing_inner_dia_mm },
        { "bearing_outer_dia_mm",       in.bearing_outer_dia_mm },
        { "bearing_thickness_mm",       in.bearing_thickness_mm },
        { "bolt_pcd_mm",                in.bolt_pcd_mm },
        { "bolt_thread_size_key",       in.bolt_thread_size_key },
        { "bolt_hole_dia_mm",           boltHoleDia },
        { "subfeature_count",           1 + in.bolt_count },
        { "bolt_positions",             bolts_json },
        { "derived_volume_removed_mm3", removedVol },
        { "standard_ref",               "IEC 61400-1" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "annular_cutter;drill";
    tooling.tool_dia_mm       = in.bearing_outer_dia_mm;
    tooling.tool_length_mm    = boltDepth + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 180.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = 120.0 + 20.0 * static_cast<double>(in.bolt_count);
    tooling.extra = {
        { "wind_feature_type", "pitch_bearing_seat" },
        { "bearing_family",    "slewing_ring" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::pitch_bearing_seat_compound applied: ID={} OD={} bolts={}",
                  in.bearing_inner_dia_mm, in.bearing_outer_dia_mm,
                  in.bolt_count);

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
            { "wind_feature_type", "pitch_bearing_seat" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::pitch_bearing_seat_compound
