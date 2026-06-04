// @lat: [[engine/skills#busbar_terminal_post_compound]]

#include "busbar_terminal_post_compound.hpp"

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

namespace koocadcam::skill::busbar_terminal_post_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "busbar_terminal_post_compound: face_id out of range");
    }
    if (!(in.post_dia_mm > 0.0) || !(in.post_height_mm > 0.0) ||
        !(in.insulator_recess_dia_mm > 0.0) ||
        !(in.insulator_recess_depth_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "busbar_terminal_post_compound: all dimensions must be > 0");
        return r;
    }

    // Central ISO thread table lookup
    const auto* spec = thread_table::findMetric(in.thread_size_key);
    if (!spec) {
        r.add("DFM-THREAD-KEY", "error",
              "busbar_terminal_post_compound: thread_size_key '" +
              in.thread_size_key + "' not in central ISO M-thread table");
        return r;
    }

    // Thread tap pilot must fit inside the post
    if (spec->tap_pilot_dia_mm >= in.post_dia_mm - 1.0) {
        r.add("DFM-POST-WALL", "error",
              "busbar_terminal_post_compound: post wall " +
              std::to_string((in.post_dia_mm - spec->tap_pilot_dia_mm) / 2.0) +
              " mm too thin for " + in.thread_size_key + " thread");
    }

    // Insulator gap check
    if (in.insulator_recess_dia_mm <= in.post_dia_mm + 0.5) {
        r.add("DFM-INSULATOR-GAP", "error",
              "busbar_terminal_post_compound: insulator_recess_dia must exceed "
              "post_dia by ≥ 0.5 mm (annular gap for insulator ring)");
    }

    if (in.insulator_recess_depth_mm > 0.5 * in.insulator_recess_dia_mm) {
        r.add("DFM-INSULATOR-DEPTH", "warning",
              "busbar_terminal_post_compound: recess depth/dia ratio > 0.5 — "
              "unusually deep insulator pocket");
    }

    if (in.post_height_mm < 1.5 * spec->nominal_dia_mm) {
        r.add("DFM-POST-HEIGHT", "warning",
              "busbar_terminal_post_compound: post_height " +
              std::to_string(in.post_height_mm) + " < 1.5 × thread nominal");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "busbar_terminal_post_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = thread_table::findMetric(in.thread_size_key);
    // (validated; not null here)

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const double baseZ = faceC.Z();

    const double postR        = in.post_dia_mm / 2.0;
    const double pilotR       = spec->tap_pilot_dia_mm / 2.0;
    const double insR_outer   = in.insulator_recess_dia_mm / 2.0;
    const double insR_inner   = postR + 0.1;  // hug the post base
    constexpr double kEmbed   = 0.05;

    // ── Sub-feature 1: FUSE cylindrical post on face ─────────────────────
    // Post extrudes ALONG +n (outward normal) from face plane.
    const gp_Pnt postOrigin(in.center_x_mm, in.center_y_mm,
                            baseZ - kEmbed);  // embed slightly so fuse is clean
    const gp_Ax2 postAx(postOrigin, n);
    const TopoDS_Shape post = pr::cylinder(
        postAx, postR, in.post_height_mm + kEmbed);

    const TopoDS_Shape withPost = pr::fuse(wp.shape(), post);

    // ── Sub-feature 2: CUT threaded pilot through post top ───────────────
    // Goes from above post top down through full post + into housing 1 mm.
    const double drillLen = in.post_height_mm + 1.5 + kEmbed;
    const gp_Pnt drillOrigin(in.center_x_mm, in.center_y_mm,
                             baseZ + in.post_height_mm + kEmbed);
    // Drill points opposite the face normal (into the post).
    const gp_Dir drillDir(-n.X(), -n.Y(), -n.Z());
    const gp_Ax2 drillAx(drillOrigin, drillDir);
    const TopoDS_Shape pilot = pr::cylinder(drillAx, pilotR, drillLen);

    // ── Sub-feature 3: CUT annular insulator recess at post base ─────────
    // Annular ring on face plane, going INTO the housing (along -n).
    const gp_Pnt recessOrigin(in.center_x_mm, in.center_y_mm,
                              baseZ + kEmbed);
    const gp_Ax2 recessAx(recessOrigin, drillDir);  // into housing
    const TopoDS_Shape insRecess = pr::annularRing(
        recessAx, insR_outer, insR_inner,
        in.insulator_recess_depth_mm + kEmbed);

    // Bundle the 2 cuts.
    const TopoDS_Shape afterPilot  = pr::cut(withPost, pilot);
    const TopoDS_Shape newShape    = pr::cut(afterPilot, insRecess);

    // ── Volumes ──────────────────────────────────────────────────────────
    const double postVol   = M_PI * postR * postR * in.post_height_mm;
    const double pilotVol  = M_PI * pilotR * pilotR * drillLen;
    const double recessVol = M_PI *
        (insR_outer * insR_outer - insR_inner * insR_inner) *
        in.insulator_recess_depth_mm;
    const double addedVol   = postVol;
    const double removedVol = pilotVol + recessVol;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",                  in.face_id },
        { "center_x_mm",              in.center_x_mm },
        { "center_y_mm",              in.center_y_mm },
        { "post_dia_mm",              in.post_dia_mm },
        { "post_height_mm",           in.post_height_mm },
        { "thread_size_key",          in.thread_size_key },
        { "insulator_recess_dia_mm",  in.insulator_recess_dia_mm },
        { "insulator_recess_depth_mm", in.insulator_recess_depth_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_ev_feature",       true },
        { "ev_feature_type",     "busbar_terminal_post" },
        { "subfeature_count",    3 },
        { "post_dia_mm",         in.post_dia_mm },
        { "post_height_mm",      in.post_height_mm },
        { "thread_size_key",     in.thread_size_key },
        { "thread_pilot_dia_mm", spec->tap_pilot_dia_mm },
        { "insulator_recess_dia_mm",   in.insulator_recess_dia_mm },
        { "insulator_recess_depth_mm", in.insulator_recess_depth_mm },
        { "derived_volume_added_mm3",   addedVol },
        { "derived_volume_removed_mm3", removedVol },
        { "subfeatures", json::array({
            { { "op", "fuse_cyl_post" },
              { "dia_mm",  in.post_dia_mm },
              { "height_mm", in.post_height_mm },
              { "added_vol_mm3", postVol } },
            { { "op", "cut_thread_pilot_through_post" },
              { "pilot_dia_mm", spec->tap_pilot_dia_mm },
              { "length_mm", drillLen },
              { "removed_vol_mm3", pilotVol } },
            { { "op", "cut_insulator_annular_recess" },
              { "outer_dia_mm", in.insulator_recess_dia_mm },
              { "inner_dia_mm", in.post_dia_mm + 0.2 },
              { "depth_mm",     in.insulator_recess_depth_mm },
              { "removed_vol_mm3", recessVol } },
        }) },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;tap;end_mill";
    tooling.tool_dia_mm       = spec->tap_pilot_dia_mm;
    tooling.tool_length_mm    = drillLen + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(5.0, (addedVol + removedVol) / 80.0);
    tooling.extra = {
        { "feature_standard", "EV busbar terminal post (ISO metric)" },
        { "thread_size_key",  in.thread_size_key },
        { "added_volume_mm3", addedVol },
        { "removed_volume_mm3", removedVol },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::busbar_terminal_post_compound applied: postD={} postH={} thread={}",
        in.post_dia_mm, in.post_height_mm, in.thread_size_key);

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
            { "ev_feature_type",   "busbar_terminal_post" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::busbar_terminal_post_compound
