// @lat: [[engine/skills#pv_junction_box_pottable_pocket]]

#include "pv_junction_box_pottable_pocket.hpp"

#include "_iso_thread_table.hpp"
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
#include <memory>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::pv_junction_box_pottable_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "pv_junction_box_pottable_pocket: face_id out of range");
    }
    if (!(in.pocket_w_mm > 0.0) || !(in.pocket_h_mm > 0.0) ||
        !(in.pocket_depth_mm > 0.0) || in.gland_count <= 0) {
        r.add("DFM-INPUT", "error",
              "pv_junction_box_pottable_pocket: all dimensions and counts > 0");
        return r;
    }

    const auto* spec = thread_table::findMetric(in.gland_thread_size_key);
    if (!spec) {
        r.add("DFM-THREAD-KEY", "error",
              "pv_junction_box_pottable_pocket: gland_thread_size_key '" +
              in.gland_thread_size_key + "' not in central ISO M-thread table");
        return r;
    }

    if (in.gland_count != 3 && in.gland_count != 4) {
        r.add("DFM-PV-GLAND-COUNT", "error",
              "pv_junction_box_pottable_pocket: gland_count must be 3 or 4 "
              "(got " + std::to_string(in.gland_count) + ")");
    }

    if (in.pocket_w_mm < 40.0 || in.pocket_w_mm > 120.0) {
        r.add("DFM-PV-POCKET-W", "warning",
              "pv_junction_box_pottable_pocket: pocket_w outside typical "
              "[40, 120] mm range");
    }
    if (in.pocket_h_mm < 20.0 || in.pocket_h_mm > 60.0) {
        r.add("DFM-PV-POCKET-H", "warning",
              "pv_junction_box_pottable_pocket: pocket_h outside typical "
              "[20, 60] mm range");
    }

    // Glands fit along the long wall: N glands of pilot_dia each with min
    // 1×pilot_dia clearance between them.
    const double pilot = spec->tap_pilot_dia_mm;
    const double required = in.gland_count * pilot
                           + (in.gland_count + 1) * pilot;  // gaps + edges
    if (required > in.pocket_w_mm) {
        r.add("DFM-PV-GLAND-FIT", "error",
              "pv_junction_box_pottable_pocket: " +
              std::to_string(in.gland_count) + " × " +
              in.gland_thread_size_key + " glands do not fit in pocket_w");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pv_junction_box_pottable_pocket DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = thread_table::findMetric(in.gland_thread_size_key);
    // (validated; not null here)

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const double baseZ = faceC.Z();
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());

    constexpr double kEmbed = 0.05;
    const double pocketDepth = in.pocket_depth_mm + kEmbed;

    // ── 1. CUT rectangular potting pocket ────────────────────────────────
    // Pocket centred on (center_x, center_y) on the top face, going INTO
    // housing along drillDir.  Origin = bottom-left corner of pocket on the
    // face plane (+ kEmbed above).
    const gp_Pnt pocketOrigin(
        in.center_x_mm - in.pocket_w_mm / 2.0,
        in.center_y_mm - in.pocket_h_mm / 2.0,
        baseZ + kEmbed);
    const gp_Ax2 pocketAx(pocketOrigin, drillDir);
    // box(ax, dx, dy, dz): dx along +X (pocket_w), dy along +Y (pocket_h),
    //                     dz along drillDir (pocket_depth)
    const TopoDS_Shape pocketTool = pr::box(
        pocketAx, in.pocket_w_mm, in.pocket_h_mm, pocketDepth);
    TopoDS_Shape current = pr::cut(wp.shape(), pocketTool);

    // ── 2..N+1. CUT N tapped cable-gland bores ───────────────────────────
    // Glands drilled THROUGH the -Y pocket wall (so axis = +Y).  Drill from
    // outside (y = center_y - pocket_h/2 - extra) inward by full pocket
    // wall depth.  We use the bottom-edge of the pocket — gland bores
    // enter at Z = baseZ - pocket_depth/2 (vertical middle of side wall).
    const double pilotR    = spec->tap_pilot_dia_mm / 2.0;
    const double wallY     = in.center_y_mm - in.pocket_h_mm / 2.0;
    const double drillStartY = wallY + 5.0;  // outside the pocket
    const double drillLen    = 5.0 + in.pocket_h_mm + 2.0 * kEmbed;
    const double glandZ      = baseZ - in.pocket_depth_mm / 2.0;
    const double margin      = spec->tap_pilot_dia_mm * 1.0;
    const double usableW     = in.pocket_w_mm - 2.0 * margin;
    const double step        = (in.gland_count > 1)
        ? usableW / (in.gland_count - 1) : 0.0;
    const double startX      = in.center_x_mm - usableW / 2.0;

    double totalGlandVol = 0.0;
    for (int k = 0; k < in.gland_count; ++k) {
        const double gx = startX + k * step;
        const gp_Pnt gOrigin(gx, drillStartY, glandZ);
        const gp_Ax2 gAx(gOrigin, gp_Dir(0.0, -1.0, 0.0));
        const TopoDS_Shape gland = pr::cylinder(gAx, pilotR, drillLen);
        current = pr::cut(current, gland);
        totalGlandVol += M_PI * pilotR * pilotR * drillLen;
    }

    const TopoDS_Shape newShape = current;

    // ── Volumes ──────────────────────────────────────────────────────────
    const double pocketVol = in.pocket_w_mm * in.pocket_h_mm
                           * in.pocket_depth_mm;
    const double removedVol = pocketVol + totalGlandVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",              in.face_id },
        { "center_x_mm",          in.center_x_mm },
        { "center_y_mm",          in.center_y_mm },
        { "pocket_w_mm",          in.pocket_w_mm },
        { "pocket_h_mm",          in.pocket_h_mm },
        { "pocket_depth_mm",      in.pocket_depth_mm },
        { "gland_count",          in.gland_count },
        { "gland_thread_size_key", in.gland_thread_size_key },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_solar_feature",    true },
        { "solar_feature_type",  "pv_junction_box_pocket" },
        { "subfeature_count",    1 + in.gland_count },
        { "gland_count",         in.gland_count },
        { "gland_thread_size_key", in.gland_thread_size_key },
        { "gland_pilot_dia_mm",  spec->tap_pilot_dia_mm },
        { "derived_volume_removed_mm3", removedVol },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill;tap";
    tooling.tool_dia_mm       = spec->tap_pilot_dia_mm;
    tooling.tool_length_mm    = std::max(in.pocket_depth_mm, in.pocket_h_mm) + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(8.0, removedVol / 200.0);
    tooling.extra = {
        { "feature_standard", "PV junction box potting pocket (IEC 62790)" },
        { "gland_thread_size_key", in.gland_thread_size_key },
        { "removed_volume_mm3", removedVol },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::pv_junction_box_pottable_pocket applied: w={} h={} glands={}",
        in.pocket_w_mm, in.pocket_h_mm, in.gland_count);

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
            { "source",             "metadata_replay" },
            { "is_compound",        true },
            { "solar_feature_type", "pv_junction_box_pocket" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::pv_junction_box_pottable_pocket
