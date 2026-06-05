// @lat: [[engine/skills#pv_frame_clamp_tslot]]

#include "pv_frame_clamp_tslot.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::pv_frame_clamp_tslot {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.slot_width_mm <= 0.0 || in.slot_depth_mm <= 0.0 ||
        in.slot_length_mm <= 0.0 || in.tooth_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pv_frame_clamp_tslot: all dimensions must be > 0");
    }
    if (in.tooth_count < 1 || in.tooth_count > 8) {
        r.add("DFM-TEETH", "error",
              "pv_frame_clamp_tslot: tooth_count " +
              std::to_string(in.tooth_count) +
              " out of range [1, 8] (UL 2703 grounding-tooth practice)");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pv_frame_clamp_tslot DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    // ── 1) T-slot channel: box cut DOWN from the top face ───────────────
    const double sx = in.slot_length_mm;   // length along X
    const double sy = in.slot_width_mm;     // width along Y
    const double sd = in.slot_depth_mm;
    const gp_Pnt slotOrigin(cx - sx / 2.0, cy - sy / 2.0, topZ - sd);
    const gp_Ax2 slotAx(slotOrigin, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::box(slotAx, sx, sy, sd + kOver));

    // ── N) grounding teeth: small box notches along the slot floor ──────
    // Teeth are evenly spaced along the slot length (X), centred in Y,
    // each cutting `tooth_depth_mm` deeper than the slot floor.
    const double toothLen = std::min(sx / (in.tooth_count * 2.0), 2.0);
    const double toothWid = std::min(sy * 0.6, 4.0);
    const double toothDepth = in.tooth_depth_mm;
    const double pitch = sx / static_cast<double>(in.tooth_count);
    for (int i = 0; i < in.tooth_count; ++i) {
        const double tcx = (cx - sx / 2.0) + pitch * (i + 0.5);
        const gp_Pnt tOrigin(tcx - toothLen / 2.0,
                             cy - toothWid / 2.0,
                             topZ - sd - toothDepth);
        const gp_Ax2 tAx(tOrigin, gp::DZ());
        current = pr::cut(current,
                          pr::box(tAx, toothLen, toothWid, toothDepth + kOver));
    }

    const double vSlot = sx * sy * sd;
    const double vTeeth =
        static_cast<double>(in.tooth_count) * toothLen * toothWid * toothDepth;
    const double volRemoved = vSlot + vTeeth;

    json params = {
        { "center_xy",      { in.center_xy.X(), in.center_xy.Y(), in.center_xy.Z() } },
        { "slot_width_mm",  in.slot_width_mm },
        { "slot_depth_mm",  in.slot_depth_mm },
        { "slot_length_mm", in.slot_length_mm },
        { "tooth_count",    in.tooth_count },
        { "tooth_depth_mm", in.tooth_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "solar_feature_type",         "frame_clamp_tslot_grounding" },
        { "subfeature_count",           1 + in.tooth_count },
        { "tooth_count",                in.tooth_count },
        { "derived_slot_width_mm",      in.slot_width_mm },
        { "derived_slot_depth_mm",      in.slot_depth_mm },
        { "derived_tooth_len_mm",       toothLen },
        { "derived_tooth_wid_mm",       toothWid },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "UL 2703" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "slot_mill;engraving_cutter";
    tooling.tool_dia_mm       = in.slot_width_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(20.0, in.slot_length_mm / 2.0);
    tooling.extra = {
        { "solar_application", "module_clamp" },
        { "standard",          "UL 2703" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::pv_frame_clamp_tslot: slot {}x{} teeth={}",
                  in.slot_length_mm, in.slot_width_mm, in.tooth_count);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric: a clamp T-slot leaves several planar faces clustered in a
    // narrow Z band below the top.  Count planar faces near the top.
    int planarNearTop = 0;
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const gp_Pnt c = wp.faceCenter(i);
        if (c.Z() < zMax - 0.01 && c.Z() > zMax - 12.0) ++planarNearTop;
    }
    if (planarNearTop >= 6) {
        json recovered = { { "slot_width_mm",  10.0 },
                           { "slot_depth_mm",  6.0 },
                           { "slot_length_mm", 40.0 },
                           { "tooth_count",    4 },
                           { "tooth_depth_mm", 0.8 } };
        json matched = { { "source", "geometric_tslot_pattern" },
                         { "planar_near_top", planarNearTop } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::pv_frame_clamp_tslot
