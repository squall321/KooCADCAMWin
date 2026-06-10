// @lat: [[engine/skills#pcb_lock_tab_compound]]

#include "pcb_lock_tab_compound.hpp"

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

namespace koocadcam::skill::pcb_lock_tab_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "pcb_lock_tab_compound: face_id out of range");
    }
    if (!(in.tab_width_mm > 0.0) || !(in.tab_length_mm > 0.0) ||
        !(in.tab_height_above_face_mm > 0.0) ||
        !(in.slot_engagement_depth_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "pcb_lock_tab_compound: all dimensions must be > 0");
        return r;
    }

    if (in.tab_height_above_face_mm < 0.5) {
        r.add("DFM-TAB-HEIGHT", "error",
              "pcb_lock_tab_compound: tab_height_above_face " +
              std::to_string(in.tab_height_above_face_mm) +
              " mm < 0.5 mm sheet-metal forming minimum");
    }
    if (in.tab_height_above_face_mm > 4.0) {
        r.add("DFM-TAB-HEIGHT", "warning",
              "pcb_lock_tab_compound: tab_height_above_face > 4 mm — "
              "treat as boss, not sheet tab");
    }
    if (in.slot_engagement_depth_mm < 0.4) {
        r.add("DFM-MIN-WALL", "error",
              "pcb_lock_tab_compound: slot_engagement_depth " +
              std::to_string(in.slot_engagement_depth_mm) +
              " mm < 0.4 mm minimum (PCB edge contact)");
    }
    if (in.slot_engagement_depth_mm >= in.tab_height_above_face_mm) {
        r.add("DFM-SLOT-VS-TAB", "error",
              "pcb_lock_tab_compound: slot_engagement_depth must be < "
              "tab_height_above_face (undercut bounded by tab)");
    }
    if (in.tab_width_mm < 3.0 || in.tab_width_mm > 5.0) {
        r.add("DFM-TAB-WIDTH", "warning",
              "pcb_lock_tab_compound: tab_width " +
              std::to_string(in.tab_width_mm) +
              " mm outside typical [3, 5] mm");
    }
    if (in.tab_length_mm < 5.0 || in.tab_length_mm > 10.0) {
        r.add("DFM-TAB-LEN", "warning",
              "pcb_lock_tab_compound: tab_length " +
              std::to_string(in.tab_length_mm) +
              " mm outside typical [5, 10] mm");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pcb_lock_tab_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const double baseZ = faceC.Z();

    constexpr double kEmbed = 0.05;

    const double cx = in.tab_position_x_mm;
    const double cy = in.tab_position_y_mm;
    const double W  = in.tab_width_mm;       // tangential (Y)
    const double L  = in.tab_length_mm;      // along X (long axis)
    const double H  = in.tab_height_above_face_mm;
    const double sD = in.slot_engagement_depth_mm;
    // PCB engagement slot height = thin (typ 1.2 mm = a thin PCB edge).
    constexpr double kSlotHeight = 1.2;

    // ── Sub-feature 1: FUSE rectangular tab box on face ─────────────────
    // Tab origin: corner at (cx - L/2, cy - W/2, baseZ - kEmbed).
    // box(ax, dx, dy, dz) with V = +Z, Vx = +X: DX along X, DY along Y, DZ along Z.
    // We use the outward face normal as V; assume +Z up.
    const gp_Pnt tabOrigin(cx - L / 2.0, cy - W / 2.0, baseZ - kEmbed);
    const gp_Ax2 tabAx(tabOrigin, n, gp::DX());
    const TopoDS_Shape tab = pr::box(tabAx, L, W, H + kEmbed);

    const TopoDS_Shape withTab = pr::fuse(wp.shape(), tab);

    // ── Sub-feature 2: CUT engagement slot underneath the tab ────────────
    // The slot is a HORIZONTAL undercut on the face plane, located at the
    // tab's "front" edge (PCB edge slides under from -X direction).  We cut
    // a thin slot of (L * 0.5) length along X, (W) wide along Y, and depth
    // = sD INTO the face downward… but actually it's a slot that engages
    // BENEATH the tab.  We cut a slot at face level on the tab footprint:
    // Position: centred at (cx - L * 0.25, cy), spanning L*0.5 × W × sD
    // sitting AT baseZ ↑ to (baseZ + kSlotHeight).
    //
    // To make it a true "undercut beneath the tab," the slot box must sit
    // INSIDE the fused tab body, at baseZ to baseZ + kSlotHeight (i.e.
    // cutting through the tab horizontally from its open end).
    //
    // Tab base sits at baseZ; cap at baseZ + H.  Slot interior runs from
    // baseZ to baseZ + kSlotHeight.  Length sD along X (the engagement
    // depth), starting from the tab's -X edge moving inward.
    const gp_Pnt slotOrigin(
        cx - L / 2.0 - kEmbed,         // start at tab front (-X edge)
        cy - W / 2.0,
        baseZ + kEmbed);                // just above face plane
    const gp_Ax2 slotAx(slotOrigin, n, gp::DX());
    // Slot dimensions: DX = sD + kEmbed (engagement depth, along X into tab),
    //                  DY = W (full tab tangential width),
    //                  DZ = kSlotHeight (PCB thickness).
    const TopoDS_Shape slot = pr::box(
        slotAx, sD + 2.0 * kEmbed, W, kSlotHeight);

    const TopoDS_Shape newShape = pr::cut(withTab, slot);

    // ── Volumes ──────────────────────────────────────────────────────────
    const double addedVol   = L * W * H;
    const double removedVol = sD * W * kSlotHeight;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",                  in.face_id },
        { "tab_position_x_mm",        in.tab_position_x_mm },
        { "tab_position_y_mm",        in.tab_position_y_mm },
        { "tab_width_mm",             in.tab_width_mm },
        { "tab_length_mm",            in.tab_length_mm },
        { "tab_height_above_face_mm", in.tab_height_above_face_mm },
        { "slot_engagement_depth_mm", in.slot_engagement_depth_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_ev_feature",       true },
        { "ev_feature_type",     "pcb_lock_tab" },
        { "subfeature_count",    2 },
        { "tab_width_mm",        in.tab_width_mm },
        { "tab_length_mm",       in.tab_length_mm },
        { "tab_height_mm",       in.tab_height_above_face_mm },
        { "slot_engagement_depth_mm", in.slot_engagement_depth_mm },
        { "derived_volume_added_mm3",   addedVol },
        { "derived_volume_removed_mm3", removedVol },
        { "subfeatures", json::array({
            { { "op", "fuse_tab_box" },
              { "L_mm", L }, { "W_mm", W }, { "H_mm", H },
              { "added_vol_mm3", addedVol } },
            { { "op", "cut_pcb_engagement_slot" },
              { "depth_mm", sD }, { "height_mm", kSlotHeight },
              { "removed_vol_mm3", removedVol } },
        }) },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "stamp;end_mill";
    tooling.tool_dia_mm       = kSlotHeight;
    tooling.tool_length_mm    = sD + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.025;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(3.0, (addedVol + removedVol) / 50.0);
    tooling.extra = {
        { "feature_standard", "EV PCB retainer lock tab (sheet metal)" },
        { "added_volume_mm3", addedVol },
        { "removed_volume_mm3", removedVol },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::pcb_lock_tab_compound applied: L={} W={} H={} slot={}",
        L, W, H, sD);

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
            { "source",          "metadata_replay" },
            { "is_compound",     true },
            { "ev_feature_type", "pcb_lock_tab" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::pcb_lock_tab_compound
