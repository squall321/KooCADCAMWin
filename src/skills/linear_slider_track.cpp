// @lat: [[engine/skills#linear_slider_track]]

#include "linear_slider_track.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::linear_slider_track {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec table (ASME-Y14.5 sliding fits H7/g6, H8/f7, H9/d10) ────────────

namespace {

constexpr std::array<TrackSpec, 3> kTrackTable {{
    { "PRECISION_H7", 0.020, 1.5 },
    { "GENERAL_H8",   0.040, 1.0 },
    { "LOOSE_H9",     0.080, 0.8 },
}};

}  // namespace

const TrackSpec* lookupTrackSpec(const std::string& key)
{
    for (const auto& s : kTrackTable) {
        if (key == s.key) return &s;
    }
    return nullptr;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const TrackSpec* spec = lookupTrackSpec(in.track_class);
    if (!spec) {
        r.add("DFM-SPEC", "error",
              "linear_slider_track: unknown track_class '" + in.track_class +
              "' (expected PRECISION_H7|GENERAL_H8|LOOSE_H9)");
        return r;
    }

    if (in.slot_width_mm < 0.8) {
        r.add("DFM-002", "error",
              "linear_slider_track: slot_width " +
              std::to_string(in.slot_width_mm) + " mm < min 0.8 mm");
    }
    if (in.slot_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "linear_slider_track: slot_depth_mm must be > 0");
    }
    if (spec->rib_height_mm < 0.5) {
        r.add("DFM-RIB", "error",
              "linear_slider_track: derived rib_height " +
              std::to_string(spec->rib_height_mm) + " mm < 0.5 mm");
    }
    if (in.end_stop_thk_mm < 1.0) {
        r.add("DFM-STOP", "error",
              "linear_slider_track: end_stop_thk " +
              std::to_string(in.end_stop_thk_mm) +
              " mm < 1.0 mm (compression strength)");
    }
    const double slotLen = std::hypot(in.end_x_mm - in.start_x_mm,
                                      in.end_y_mm - in.start_y_mm);
    if (slotLen <= 1e-6) {
        r.add("DFM-INPUT", "error",
              "linear_slider_track: start/end coincide (slot length 0)");
    }
    if (in.slot_depth_mm <= spec->rib_height_mm) {
        r.add("DFM-RIB", "error",
              "linear_slider_track: slot_depth must exceed rib_height "
              "(rib would be entirely above slot floor)");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "linear_slider_track DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const TrackSpec* spec = lookupTrackSpec(in.track_class);
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("linear_slider_track: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    const gp_Vec dir2D(in.end_x_mm - in.start_x_mm,
                       in.end_y_mm - in.start_y_mm, 0.0);
    const double slotLen = dir2D.Magnitude();
    const gp_Dir xLoc(dir2D.X() / slotLen, dir2D.Y() / slotLen, 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    // Shrink the slot length by end_stop_thk_mm at each end so the end-stop
    // walls remain in place.
    const double effLen = std::max(0.1, slotLen - 2.0 * in.end_stop_thk_mm);
    const gp_Pnt midPt(
        (in.start_x_mm + in.end_x_mm) / 2.0,
        (in.start_y_mm + in.end_y_mm) / 2.0,
        zMax);

    // SUBFEATURE 1: main slot cutter — a box of width × effLen × (depth + overhang)
    // Centred on midPt, with local frame: V (up Z) is local Z (extrusion), Vx = xLoc.
    const gp_Pnt slotOrigin(
        midPt.X() - effLen / 2.0 * xLoc.X() - in.slot_width_mm / 2.0 * yLoc.X(),
        midPt.Y() - effLen / 2.0 * xLoc.Y() - in.slot_width_mm / 2.0 * yLoc.Y(),
        zMax - in.slot_depth_mm);
    const gp_Ax2 slotAx(slotOrigin, gp::DZ(), xLoc);
    const TopoDS_Shape mainSlot = pr::box(
        slotAx, effLen, in.slot_width_mm, in.slot_depth_mm + kOverhang);

    // SUBFEATURE 2: retention RIB (fused box at slot floor, runs along slot
    // centerline, height = spec->rib_height_mm).  We CUT the slot first,
    // then FUSE the rib back at the floor so it remains as a tongue.
    const double ribWidth = in.slot_width_mm * 0.4;       // 40% of slot width
    const gp_Pnt ribOrigin(
        midPt.X() - effLen / 2.0 * xLoc.X() - ribWidth / 2.0 * yLoc.X(),
        midPt.Y() - effLen / 2.0 * xLoc.Y() - ribWidth / 2.0 * yLoc.Y(),
        zMax - in.slot_depth_mm);
    const gp_Ax2 ribAx(ribOrigin, gp::DZ(), xLoc);
    const TopoDS_Shape ribBox = pr::box(
        ribAx, effLen, ribWidth, spec->rib_height_mm);

    // SUBFEATURE 3 & 4: end-stop walls are implicit in the shrunk slot
    // (we left end_stop_thk_mm of material at each extremity).
    // SUBFEATURE 5: limit walls are the slot sidewalls (also implicit).

    // Boolean composition:
    //   workpiece' = (workpiece − mainSlot) ∪ ribBox
    const TopoDS_Shape slotted    = pr::cut(wp.shape(), mainSlot);
    const TopoDS_Shape withRib    = pr::fuse(slotted, ribBox);

    // Build signature
    json params = {
        { "entry_face_id",   *entryId },
        { "start_x_mm",      in.start_x_mm },
        { "start_y_mm",      in.start_y_mm },
        { "end_x_mm",        in.end_x_mm },
        { "end_y_mm",        in.end_y_mm },
        { "axis_dir",        { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "slot_width_mm",   in.slot_width_mm },
        { "slot_depth_mm",   in.slot_depth_mm },
        { "end_stop_thk_mm", in.end_stop_thk_mm },
        { "track_class",     in.track_class },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      5 },
        { "track_class",           in.track_class },
        { "slot_clearance_mm",     spec->slot_clearance_mm },
        { "rib_height_mm",         spec->rib_height_mm },
        { "slot_width_mm",         in.slot_width_mm },
        { "slot_depth_mm",         in.slot_depth_mm },
        { "end_stop_thk_mm",       in.end_stop_thk_mm },
        { "length_mm",             slotLen },
        { "effective_slot_len_mm", effLen },
    };
    ToolingMeta tooling;
    tooling.tool_type = "end_mill";
    tooling.tool_dia_mm = std::min(in.slot_width_mm, 3.0);
    tooling.tool_length_mm = in.slot_depth_mm + 5.0;
    tooling.tool_material  = "carbide";
    tooling.flute_count    = 3;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.04;
    // Volume removed = slot volume − rib volume.
    const double slotVol = effLen * in.slot_width_mm * in.slot_depth_mm;
    const double ribVol  = effLen * ribWidth * spec->rib_height_mm;
    tooling.stock_removed_mm3 = std::max(0.0, slotVol - ribVol);
    tooling.est_cycle_time_s = std::max(3.0, slotLen * 0.2 + in.slot_depth_mm * 0.6);
    tooling.extra = {
        { "spec_table_ref", "ASME-Y14.5 sliding fits H7/g6, H8/f7, H9/d10" },
        { "compound_chain", {
            { { "step", "cut_main_slot" },   { "tool", "end_mill" } },
            { { "step", "fuse_retention_rib" }, { "tool", "end_mill" } },
            { { "step", "implicit_end_stops" }, { "note", "left in place by shrunk slot" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(withRib, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::linear_slider_track applied: class={} len={} w={} d={}",
                  in.track_class, slotLen, in.slot_width_mm, in.slot_depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Strategy: find a horizontal (normal=+Z) flat-bottom face strictly below the
// workpiece top that has an internal rib (a second smaller flat-top face
// strictly above bottom but below top).  The rib presence is the key
// disambiguator from a plain rectangular slot.
//
// Metadata replay: if the live workpiece has features() listing
// "linear_slider_track", we replay from there.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // ── Metadata replay path ────────────────────────────────────────────
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        if (!f.pattern.contains("track_class")) continue;
        RecognizedFeature r;
        r.skill_id = kSkillId;
        r.confidence = 0.99;
        r.recovered_params = f.params;
        r.matched_geometry = {
            { "source", "metadata_replay" },
            { "subfeature_count", f.pattern.value("subfeature_count", 0) },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // ── Geometric heuristic ─────────────────────────────────────────────
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // Find horizontal floor faces strictly below top, then verify a second
    // horizontal face exists at an intermediate Z (the rib top).
    int floorId = -1;
    double floorZ = -1e30;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(n.Z() - 1.0) > 1e-2) continue;
        const gp_Pnt c = wp.faceCenter(i);
        if (c.Z() >= zMax - 1e-3) continue;
        if (c.Z() <= zMin + 1e-3) continue;
        if (c.Z() > floorZ) {
            // pick the LOWEST horizontal floor (likely slot floor)
        }
        if (floorId < 0 || c.Z() < floorZ) { floorId = i; floorZ = c.Z(); }
    }
    if (floorId < 0) return out;

    // Look for a rib top — a second horizontal face above floorZ but below
    // workpiece top.
    int ribTopId = -1;
    double ribTopZ = -1e30;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (i == floorId) continue;
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(n.Z() - 1.0) > 1e-2) continue;
        const gp_Pnt c = wp.faceCenter(i);
        if (c.Z() <= floorZ + 1e-3) continue;
        if (c.Z() >= zMax  - 1e-3) continue;
        if (c.Z() > ribTopZ) { ribTopId = i; ribTopZ = c.Z(); }
    }
    if (ribTopId < 0) return out;

    const double depth = zMax - floorZ;
    const double ribH  = ribTopZ - floorZ;
    // Match best spec by rib height.
    const TrackSpec* best = nullptr;
    double bestErr = 1e9;
    for (const auto& s : kTrackTable) {
        const double err = std::abs(s.rib_height_mm - ribH);
        if (err < bestErr) { bestErr = err; best = &s; }
    }
    const std::string klass = best ? std::string(best->key) : std::string("GENERAL_H8");

    // Estimate length/width from the floor face center & bbox.
    const gp_Pnt fc = wp.faceCenter(floorId);

    json recovered = {
        { "start_x_mm",      fc.X() - 5.0 },
        { "start_y_mm",      fc.Y() },
        { "end_x_mm",        fc.X() + 5.0 },
        { "end_y_mm",        fc.Y() },
        { "axis_dir",        { 0.0, 0.0, -1.0 } },
        { "slot_width_mm",   std::max(0.8, wp.faceArea(floorId) / 10.0) },
        { "slot_depth_mm",   depth },
        { "end_stop_thk_mm", 1.0 },
        { "track_class",     klass },
    };
    json matched = {
        { "floor_face_id",   floorId },
        { "rib_top_face_id", ribTopId },
        { "rib_height_mm",   ribH },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.70, matched });
    return out;
}

}  // namespace koocadcam::skill::linear_slider_track
