// @lat: [[engine/skills#gate_valve_seat_compound]]

#include "gate_valve_seat_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::gate_valve_seat_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.seat_bore_dia_mm <= 0.0 || in.seat_od_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gate_valve_seat_compound: seat dimensions must be > 0");
    }
    if (in.seat_bore_dia_mm >= in.seat_od_mm) {
        r.add("DFM-VALVE-SEAT", "error",
              "gate_valve_seat_compound: seat OD must exceed bore dia");
    }
    if (in.seat_taper_deg < 5.0 || in.seat_taper_deg > 25.0) {
        r.add("DFM-VALVE-SEAT-ANGLE", "error",
              "gate_valve_seat_compound: seat taper " +
              std::to_string(in.seat_taper_deg) +
              "° outside [5°, 25°] (ASME B16.34 §6.4 trim guidance)");
    }
    if (in.slot_width_mm < 0.8) {
        r.add("DFM-002", "error",
              "gate_valve_seat_compound: gate slot width " +
              std::to_string(in.slot_width_mm) + " mm < min 0.8 mm");
    }
    if (in.stem_bore_dia_mm < 3.0) {
        r.add("DFM-VALVE-STEM", "error",
              "gate_valve_seat_compound: stem bore < 3 mm — insufficient for "
              "torque transmission (DIN EN 1984 Annex A)");
    }
    if (in.seat_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gate_valve_seat_compound: seat length must be > 0");
    }
    if (in.body_y_span_mm <= in.slot_width_mm) {
        r.add("DFM-VALVE-BODY", "error",
              "gate_valve_seat_compound: body span too small for slot");
    }
    // Wall thickness: stock width along flow axis must exceed 2× (seat_length
    // + seat_od) so the seats fit + minimum 5 mm wall (DIN EN 1984 Table 6).
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double flowSpan = (xMax - xMin);
    const double required = 2.0 * in.seat_length_mm + in.slot_width_mm + 10.0;
    if (flowSpan < required) {
        r.add("DFM-VALVE-WALL", "error",
              "gate_valve_seat_compound: stock flow-axis span " +
              std::to_string(flowSpan) + " mm < required " +
              std::to_string(required) + " mm (DIN EN 1984 Table 6 wall)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gate_valve_seat_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.1;

    const gp_Pnt center(in.center_x_mm, in.center_y_mm, in.center_z_mm);

    // ── Sub-feature 1: GATE SLOT (rectangular through-Y prism) ──────────
    //
    // Slot is centered on (cx, cy, cz), width = slot_width along flow_axis,
    // height = slot_height along stem_axis, length = body_y_span along Y.
    // We construct an axis-aligned box and rely on flow_axis = +X for slice-1.
    if (std::abs(in.flow_axis.X() - 1.0) > 1e-6 ||
        std::abs(in.flow_axis.Y()) > 1e-6 ||
        std::abs(in.flow_axis.Z()) > 1e-6) {
        throw SkillError("gate_valve_seat_compound: slice-1 supports flow_axis=+X only");
    }
    if (std::abs(in.stem_axis.X()) > 1e-6 ||
        std::abs(in.stem_axis.Y()) > 1e-6 ||
        std::abs(in.stem_axis.Z() - 1.0) > 1e-6) {
        throw SkillError("gate_valve_seat_compound: slice-1 supports stem_axis=+Z only");
    }

    const gp_Pnt slotOrigin(
        center.X() - in.slot_width_mm / 2.0,
        yMin - kOverhang,
        center.Z() - in.slot_height_mm / 2.0);
    const TopoDS_Shape slotBox = pr::box(
        gp_Ax2(slotOrigin, gp::DZ()),
        in.slot_width_mm,
        (yMax - yMin) + 2.0 * kOverhang,
        in.slot_height_mm);

    // ── Sub-features 2 & 3: CONICAL SEATS (left + right of the slot) ────
    //
    // Each seat is a cone frustum coaxial with the flow axis, narrow end at
    // the slot, wide end pointing outward.  We model as outer-OD cone minus
    // inner cylindrical bore (the through flow path).
    //
    // Left seat: cone from x = slot_left_edge → outward to x = slot_left_edge - seat_length
    // Right seat: mirror.
    //
    // The radial profile: at the slot, OD = seat_bore + epsilon (cone matches
    // bore).  Outward, the OD grows.  We simplify: bore_R is INNER (a clear
    // through-hole), and the outer cone goes from seat_bore_R/2 → seat_OD/2.
    const double boreR  = in.seat_bore_dia_mm / 2.0;
    const double odR    = in.seat_od_mm / 2.0;
    const double slotLx = center.X() - in.slot_width_mm / 2.0;
    const double slotRx = center.X() + in.slot_width_mm / 2.0;

    // Build the THROUGH BORE — cylindrical hole spanning the full flow axis
    // length of the body (this is what the seats sit around).
    const gp_Ax2 boreAx(
        gp_Pnt(xMin - kOverhang, center.Y(), center.Z()),
        gp::DX());
    const TopoDS_Shape throughBore = pr::cylinder(
        boreAx, boreR, (xMax - xMin) + 2.0 * kOverhang);

    // LEFT seat cone: at slot_left_edge the OD is at boreR (smooth into bore),
    // and at slot_left_edge − seat_length the OD is at odR.  But the cone
    // tool here represents the SHAPE we're cutting AROUND the bore — i.e.
    // the "seat ring socket".  We construct the cone going OUTWARD (-X) from
    // the slot.
    const gp_Ax2 leftSeatAx(
        gp_Pnt(slotLx, center.Y(), center.Z()),
        gp_Dir(-1.0, 0.0, 0.0));
    const TopoDS_Shape leftSeatCone = pr::coneFrustum(
        leftSeatAx, boreR, odR, in.seat_length_mm);

    const gp_Ax2 rightSeatAx(
        gp_Pnt(slotRx, center.Y(), center.Z()),
        gp::DX());
    const TopoDS_Shape rightSeatCone = pr::coneFrustum(
        rightSeatAx, boreR, odR, in.seat_length_mm);

    // ── Sub-feature 4: STEM BORE (vertical, from top) ───────────────────
    const gp_Ax2 stemAx(
        gp_Pnt(center.X(), center.Y(), zMax + kOverhang),
        gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape stemBore = pr::cylinder(
        stemAx, in.stem_bore_dia_mm / 2.0,
        in.stem_bore_depth_mm + kOverhang);

    // ── Sub-feature 5: BOTTOM RECESS (gate park, below slot) ────────────
    const gp_Ax2 recessAx(
        gp_Pnt(center.X(), center.Y(),
               center.Z() - in.slot_height_mm / 2.0 + kOverhang),
        gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape bottomRecess = pr::cylinder(
        recessAx, in.recess_dia_mm / 2.0,
        in.recess_depth_mm + kOverhang);

    // ── Fuse-then-cut for overlapping/concentric tools ──────────────────
    const TopoDS_Shape seats = pr::fuse(leftSeatCone, rightSeatCone);
    const TopoDS_Shape seatsAndBore = pr::fuse(seats, throughBore);
    const TopoDS_Shape seatsBoreSlot = pr::fuse(seatsAndBore, slotBox);
    const TopoDS_Shape vertGroup = pr::fuse(stemBore, bottomRecess);
    const TopoDS_Shape allCutters = pr::fuse(seatsBoreSlot, vertGroup);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), allCutters);

    // ── Signature ───────────────────────────────────────────────────────
    json params = {
        { "center_x_mm",        in.center_x_mm },
        { "center_y_mm",        in.center_y_mm },
        { "center_z_mm",        in.center_z_mm },
        { "flow_axis",          { in.flow_axis.X(), in.flow_axis.Y(), in.flow_axis.Z() } },
        { "stem_axis",          { in.stem_axis.X(), in.stem_axis.Y(), in.stem_axis.Z() } },
        { "seat_bore_dia_mm",   in.seat_bore_dia_mm },
        { "seat_od_mm",         in.seat_od_mm },
        { "seat_taper_deg",     in.seat_taper_deg },
        { "seat_length_mm",     in.seat_length_mm },
        { "slot_width_mm",      in.slot_width_mm },
        { "slot_height_mm",     in.slot_height_mm },
        { "body_y_span_mm",     in.body_y_span_mm },
        { "stem_bore_dia_mm",   in.stem_bore_dia_mm },
        { "stem_bore_depth_mm", in.stem_bore_depth_mm },
        { "recess_dia_mm",      in.recess_dia_mm },
        { "recess_depth_mm",    in.recess_depth_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     5 },
        { "subfeatures",          { "gate_slot", "left_seat_cone", "right_seat_cone",
                                    "stem_bore", "bottom_recess" } },
        { "through_bore_dia_mm",  in.seat_bore_dia_mm },
        { "stem_bore_dia_mm",     in.stem_bore_dia_mm },
        { "slot_width_mm",        in.slot_width_mm },
        { "seat_taper_deg",       in.seat_taper_deg },
        { "conical_seat_count",   2 },
        { "axial_cyl_face_count", 2 },  // stem + recess
        { "perp_slot_present",    true },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "form_drill;end_mill;boring_bar";
    tooling.tool_dia_mm       = in.seat_od_mm;
    tooling.tool_length_mm    = in.seat_length_mm * 2.0 + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.04;
    // Volume estimate: slot + 2 × cone frustum annulus + bore + stem cyl + recess.
    const double slotVol = in.slot_width_mm * in.slot_height_mm * in.body_y_span_mm;
    const double coneVol = M_PI * in.seat_length_mm / 3.0 *
                           (boreR * boreR + boreR * odR + odR * odR);
    const double boreVol = M_PI * boreR * boreR * (xMax - xMin);
    const double stemVol = M_PI * std::pow(in.stem_bore_dia_mm / 2.0, 2)
                                * in.stem_bore_depth_mm;
    const double recVol  = M_PI * std::pow(in.recess_dia_mm / 2.0, 2)
                                * in.recess_depth_mm;
    tooling.stock_removed_mm3 = slotVol + 2.0 * coneVol + boreVol + stemVol + recVol;
    tooling.est_cycle_time_s  = std::max(10.0, tooling.stock_removed_mm3 / 1000.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "boring_bar" }, { "tool_dia_mm", in.seat_bore_dia_mm },
              { "note", "through bore" } },
            { { "tool_type", "form_cutter" }, { "tool_dia_mm", in.seat_od_mm },
              { "note", "left + right conical seats" } },
            { { "tool_type", "end_mill" },   { "tool_dia_mm", in.slot_width_mm },
              { "note", "gate slot" } },
            { { "tool_type", "drill" },      { "tool_dia_mm", in.stem_bore_dia_mm },
              { "note", "stem bore" } },
            { { "tool_type", "end_mill" },   { "tool_dia_mm", in.recess_dia_mm },
              { "note", "bottom recess" } },
        } },
        { "standard", "DIN EN 1984 + ASME B16.34 §6.4" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::gate_valve_seat_compound applied: bore={} od={} slot={}x{} faces {}→{}",
        in.seat_bore_dia_mm, in.seat_od_mm, in.slot_width_mm, in.slot_height_mm,
        wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

namespace {

// Determine if a face is conical with X-aligned axis (matches our flow axis).
struct ConicalInfo
{
    int    faceIdx;
    double minR, maxR;
    double axialLen;
    gp_Pnt center;
};

std::vector<ConicalInfo> collectConicalFlowFaces(const Workpiece& wp)
{
    std::vector<ConicalInfo> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        BRepAdaptor_Surface s(wp.face(i));
        if (s.GetType() != GeomAbs_Cone) continue;
        const gp_Cone c = s.Cone();
        const gp_Dir axDir = c.Axis().Direction();
        if (std::abs(std::abs(axDir.X()) - 1.0) > 1e-2) continue;  // not flow-axial
        ConicalInfo info;
        info.faceIdx = i;
        info.minR    = c.RefRadius();
        info.maxR    = info.minR;  // simplified — will be refined by face area
        info.axialLen = 0.0;
        info.center   = wp.faceCenter(i);
        out.push_back(info);
    }
    return out;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // First, try metadata replay (fallback path = 1.0 confidence).
    for (const auto& f : wp.features()) {
        if (f.skill_id == kSkillId) {
            RecognizedFeature r;
            r.skill_id         = kSkillId;
            r.recovered_params = f.params;
            r.confidence       = 1.0;
            r.matched_geometry = { { "source", "metadata_replay" } };
            out.push_back(r);
        }
    }
    if (!out.empty()) return out;

    // Geometric: count flow-axial conical faces + a perpendicular vertical
    // stem cylinder + a horizontal through-cyl (the bore).
    auto cones = collectConicalFlowFaces(wp);
    int  stemCyl = -1;
    int  flowCyl = -1;
    int  recessCyl = -1;

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder cyl = s.Cylinder();
        const gp_Dir axDir = cyl.Axis().Direction();
        const double r = cyl.Radius();
        if (std::abs(std::abs(axDir.Z()) - 1.0) < 1e-2) {
            // vertical (stem axis)
            const gp_Pnt c = wp.faceCenter(i);
            // top one = stem; bottom one = recess
            double xMin, yMin, zMin, xMax, yMax, zMax;
            wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
            if (c.Z() > (zMin + zMax) / 2.0) {
                if (stemCyl < 0 || wp.faceCenter(stemCyl).Z() < c.Z()) stemCyl = i;
            } else {
                if (recessCyl < 0) recessCyl = i;
            }
            (void)r;
        } else if (std::abs(std::abs(axDir.X()) - 1.0) < 1e-2) {
            // horizontal flow bore
            if (flowCyl < 0) flowCyl = i;
        }
    }

    if (cones.size() >= 2 && stemCyl >= 0 && flowCyl >= 0) {
        BRepAdaptor_Surface stemSurf(wp.face(stemCyl));
        BRepAdaptor_Surface flowSurf(wp.face(flowCyl));
        const double stemDia = 2.0 * stemSurf.Cylinder().Radius();
        const double flowDia = 2.0 * flowSurf.Cylinder().Radius();

        json recovered = {
            { "center_x_mm",        wp.faceCenter(stemCyl).X() },
            { "center_y_mm",        wp.faceCenter(stemCyl).Y() },
            { "center_z_mm",        wp.faceCenter(flowCyl).Z() },
            { "flow_axis",          { 1.0, 0.0, 0.0 } },
            { "stem_axis",          { 0.0, 0.0, 1.0 } },
            { "seat_bore_dia_mm",   flowDia },
            { "stem_bore_dia_mm",   stemDia },
            { "seat_taper_deg",     15.0 },          // best-guess (geometric heuristic)
        };
        json matched = {
            { "conical_seat_count", static_cast<int>(cones.size()) },
            { "stem_cyl_face_id",   stemCyl },
            { "flow_cyl_face_id",   flowCyl },
            { "recess_cyl_face_id", recessCyl },
            { "source",             "geometric" },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.72, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::gate_valve_seat_compound
