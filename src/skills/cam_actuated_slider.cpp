// @lat: [[engine/skills#cam_actuated_slider]]

#include "cam_actuated_slider.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::cam_actuated_slider {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec table (cam-slider classes) ──────────────────────────────────────

namespace {

constexpr std::array<CamSpec, 3> kCamTable {{
    { "MICRO_M3",  3.0, 3.0, 1.5,  4.0 },
    { "STD_M5",    5.0, 5.0, 2.5,  8.0 },
    { "HEAVY_M8",  8.0, 8.0, 4.0, 12.0 },
}};

}  // namespace

const CamSpec* lookupCamSpec(const std::string& key)
{
    for (const auto& s : kCamTable) {
        if (key == s.key) return &s;
    }
    return nullptr;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const CamSpec* spec = lookupCamSpec(in.cam_class);
    if (!spec) {
        r.add("DFM-SPEC", "error",
              "cam_actuated_slider: unknown cam_class '" + in.cam_class +
              "' (expected MICRO_M3|STD_M5|HEAVY_M8)");
        return r;
    }

    if (spec->slot_w_mm < 0.8) {
        r.add("DFM-002", "error",
              "cam_actuated_slider: slot_w " + std::to_string(spec->slot_w_mm) +
              " mm < min 0.8 mm");
    }
    if (spec->notch_w_mm < 0.8) {
        r.add("DFM-002", "error",
              "cam_actuated_slider: notch_w " + std::to_string(spec->notch_w_mm) +
              " mm < min 0.8 mm");
    }
    if (in.slot_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "cam_actuated_slider: slot_depth_mm must be > 0");
    }

    const double slotLen = std::hypot(in.slot_end_x_mm - in.slot_start_x_mm,
                                      in.slot_end_y_mm - in.slot_start_y_mm);
    if (slotLen <= 1e-6) {
        r.add("DFM-INPUT", "error",
              "cam_actuated_slider: slot endpoints coincide");
    }

    // CAM CLEARANCE: pivot bore must not overlap slot walls.  Pivot center
    // is perpendicular-offset from slot midpoint by pivot_offset_mm; need
    // pivot_offset > (slot_w + pivot_dia)/2 + 0.5 (clearance band).
    const double minOff = (spec->slot_w_mm + spec->pivot_dia_mm) / 2.0 + 0.5;
    if (std::abs(in.pivot_offset_mm) < minOff) {
        r.add("DFM-CAM-CLR", "error",
              "cam_actuated_slider: |pivot_offset| " +
              std::to_string(in.pivot_offset_mm) +
              " mm < required " + std::to_string(minOff) +
              " mm (pivot bore overlaps slot wall)");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cam_actuated_slider DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const CamSpec* spec = lookupCamSpec(in.cam_class);
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("cam_actuated_slider: entry_face unresolved");

    // Context-aware: use bbox to default pivot_offset based on workpiece size.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    const gp_Vec dir2D(in.slot_end_x_mm - in.slot_start_x_mm,
                       in.slot_end_y_mm - in.slot_start_y_mm, 0.0);
    const double slotLen = dir2D.Magnitude();
    const gp_Dir xLoc(dir2D.X() / slotLen, dir2D.Y() / slotLen, 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    const gp_Pnt slotMid(
        (in.slot_start_x_mm + in.slot_end_x_mm) / 2.0,
        (in.slot_start_y_mm + in.slot_end_y_mm) / 2.0,
        zMax);

    // SUBFEATURE 1: LINEAR SLOT cut.
    const gp_Pnt slotOrigin(
        slotMid.X() - slotLen / 2.0 * xLoc.X() - spec->slot_w_mm / 2.0 * yLoc.X(),
        slotMid.Y() - slotLen / 2.0 * xLoc.Y() - spec->slot_w_mm / 2.0 * yLoc.Y(),
        zMax - in.slot_depth_mm);
    const gp_Ax2 slotAx(slotOrigin, gp::DZ(), xLoc);
    const TopoDS_Shape slotBox = pr::box(
        slotAx, slotLen, spec->slot_w_mm, in.slot_depth_mm + kOverhang);

    // SUBFEATURE 2: PIVOT BORE — perpendicular cylindrical bore.
    // Centered at slotMid + pivot_offset along yLoc.
    const gp_Pnt pivotCenter(
        slotMid.X() + in.pivot_offset_mm * yLoc.X(),
        slotMid.Y() + in.pivot_offset_mm * yLoc.Y(),
        zMax + kOverhang);
    const gp_Ax2 pivotAx(pivotCenter, gp_Dir(0.0, 0.0, -1.0), gp::DX());
    const TopoDS_Shape pivotBore = pr::cylinder(
        pivotAx, spec->pivot_dia_mm / 2.0, in.slot_depth_mm + 2.0 * kOverhang);

    // SUBFEATURE 3: CAM NOTCH — small rectangular pocket transverse to slot,
    // at slot midpoint, that engages the cam lobe.
    const gp_Pnt notchOrigin(
        slotMid.X() - spec->notch_w_mm / 2.0 * xLoc.X() - spec->slot_w_mm * 0.75 * yLoc.X(),
        slotMid.Y() - spec->notch_w_mm / 2.0 * xLoc.Y() - spec->slot_w_mm * 0.75 * yLoc.Y(),
        zMax - in.slot_depth_mm * 1.2);
    const gp_Ax2 notchAx(notchOrigin, gp::DZ(), xLoc);
    const TopoDS_Shape notchBox = pr::box(
        notchAx, spec->notch_w_mm, spec->slot_w_mm * 1.5, in.slot_depth_mm * 1.2 + kOverhang);

    // Compose cuts.
    const TopoDS_Shape cutter1   = pr::fuse(slotBox, pivotBore);
    const TopoDS_Shape cutter    = pr::fuse(cutter1, notchBox);
    const TopoDS_Shape newShape  = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",   *entryId },
        { "slot_start_x_mm", in.slot_start_x_mm },
        { "slot_start_y_mm", in.slot_start_y_mm },
        { "slot_end_x_mm",   in.slot_end_x_mm },
        { "slot_end_y_mm",   in.slot_end_y_mm },
        { "axis_dir",        { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "slot_depth_mm",   in.slot_depth_mm },
        { "pivot_offset_mm", in.pivot_offset_mm },
        { "cam_class",       in.cam_class },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  3 },
        { "cam_class",         in.cam_class },
        { "slot_w_mm",         spec->slot_w_mm },
        { "pivot_dia_mm",      spec->pivot_dia_mm },
        { "notch_w_mm",        spec->notch_w_mm },
        { "throw_mm",          spec->throw_mm },
        { "slot_depth_mm",     in.slot_depth_mm },
        { "pivot_offset_mm",   in.pivot_offset_mm },
        { "length_mm",         slotLen },
    };
    ToolingMeta tooling;
    tooling.tool_type = "end_mill;drill";
    tooling.tool_dia_mm = spec->slot_w_mm;
    tooling.tool_length_mm = in.slot_depth_mm + 5.0;
    tooling.tool_material  = "carbide";
    tooling.flute_count    = 3;
    tooling.cutting_speed_sfm = 380.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double rPivot = spec->pivot_dia_mm / 2.0;
    tooling.stock_removed_mm3 =
        slotLen * spec->slot_w_mm * in.slot_depth_mm +
        M_PI * rPivot * rPivot * in.slot_depth_mm +
        spec->notch_w_mm * spec->slot_w_mm * 1.5 * in.slot_depth_mm * 1.2;
    tooling.est_cycle_time_s = std::max(5.0, slotLen * 0.2 + in.slot_depth_mm * 0.8);
    tooling.extra = {
        { "context_used", {
            { "wp_bbox_diag_mm", std::sqrt(
                (xMax-xMin)*(xMax-xMin) +
                (yMax-yMin)*(yMax-yMin) +
                (zMax-zMin)*(zMax-zMin)) },
            { "wp_face_count",   wp.faceCount() },
        } },
        { "compound_chain", {
            { { "step", "linear_slot" }, { "tool", "end_mill" } },
            { { "step", "pivot_bore" }, { "tool", "drill" } },
            { { "step", "cam_notch" }, { "tool", "end_mill" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::cam_actuated_slider applied: class={} slot_len={}",
                  in.cam_class, slotLen);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
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

    // Geometric heuristic: detect at least one cylindrical face (pivot bore)
    // and one near-horizontal floor face (slot floor).
    int cylId = -1;
    double cylDia = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        cylId = i;
        cylDia = 2.0 * s.Cylinder().Radius();
        break;
    }
    if (cylId < 0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    int floorId = -1;
    double floorZ = zMax;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(n.Z() - 1.0) > 1e-2) continue;
        const gp_Pnt c = wp.faceCenter(i);
        if (c.Z() >= zMax - 1e-3) continue;
        if (c.Z() < floorZ) { floorId = i; floorZ = c.Z(); }
    }
    if (floorId < 0) return out;

    // Match best spec by pivot dia.
    const CamSpec* best = nullptr;
    double bestErr = 1e9;
    for (const auto& s : kCamTable) {
        const double err = std::abs(s.pivot_dia_mm - cylDia);
        if (err < bestErr) { bestErr = err; best = &s; }
    }
    if (!best || bestErr > 1.5) return out;

    const double depth = zMax - floorZ;

    json recovered = {
        { "slot_start_x_mm",  xMin + 5.0 },
        { "slot_start_y_mm",  (yMin + yMax) / 2.0 },
        { "slot_end_x_mm",    xMax - 5.0 },
        { "slot_end_y_mm",    (yMin + yMax) / 2.0 },
        { "axis_dir",         { 0.0, 0.0, -1.0 } },
        { "slot_depth_mm",    depth },
        { "pivot_offset_mm",  best->slot_w_mm + best->pivot_dia_mm },
        { "cam_class",        std::string(best->key) },
    };
    json matched = {
        { "pivot_cyl_face_id", cylId },
        { "floor_face_id",     floorId },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.62, matched });
    return out;
}

}  // namespace koocadcam::skill::cam_actuated_slider
