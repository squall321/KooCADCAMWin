// @lat: [[engine/skills#check_valve_seat_with_stop]]

#include "check_valve_seat_with_stop.hpp"

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

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::check_valve_seat_with_stop {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.inlet_dia_mm <= 0.0 || in.outlet_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "check_valve_seat_with_stop: inlet/outlet dia must be > 0");
    }
    if (in.inlet_length_mm <= 0.0 || in.outlet_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "check_valve_seat_with_stop: inlet/outlet length must be > 0");
    }
    // API 594 §6.3 — Standard seat angle 30..60°.
    if (in.seat_angle_deg < 30.0 || in.seat_angle_deg > 60.0) {
        r.add("DFM-CHECK-VALVE-SEAT-ANGLE", "error",
              "check_valve_seat_with_stop: seat angle " +
              std::to_string(in.seat_angle_deg) +
              "° outside [30°, 60°] (API 594 §6.3)");
    }
    if (in.inlet_dia_mm < in.outlet_dia_mm) {
        r.add("DFM-CHECK-VALVE-FLOW", "error",
              "check_valve_seat_with_stop: inlet dia < outlet dia — "
              "would throttle at seat (anti-NRV pattern)");
    }
    if (in.seat_drop_mm <= 0.0) {
        r.add("DFM-CHECK-VALVE-DROP", "error",
              "check_valve_seat_with_stop: seat drop must be > 0 "
              "(else seat shoulder has no radial extent)");
    }
    // Disc-stop length must allow seat shoulder room.
    if (in.stop_length_mm > in.inlet_length_mm - 4.0) {
        r.add("DFM-CHECK-VALVE-STOP", "error",
              "check_valve_seat_with_stop: stop_length " +
              std::to_string(in.stop_length_mm) +
              " mm > inlet_length − 4 mm (disc cannot lift fully)");
    }
    if (in.stop_dia_mm >= in.inlet_dia_mm) {
        r.add("DFM-CHECK-VALVE-STOP-DIA", "error",
              "check_valve_seat_with_stop: stop dia ≥ inlet dia "
              "(stop would block flow path entirely)");
    }
    // Body-wall sanity: inlet dia + seat drop must fit inside the stock.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double requiredRadial = in.inlet_dia_mm / 2.0 + 5.0;
    if (std::min(yMax - yMin, zMax - zMin) < 2.0 * requiredRadial) {
        r.add("DFM-CHECK-VALVE-WALL", "error",
              "check_valve_seat_with_stop: stock too small for inlet bore "
              "+ 5 mm wall (API 594 Table 5)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "check_valve_seat_with_stop DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    if (std::abs(in.flow_axis.X() - 1.0) > 1e-6 ||
        std::abs(in.flow_axis.Y()) > 1e-6 ||
        std::abs(in.flow_axis.Z()) > 1e-6) {
        throw SkillError("check_valve_seat_with_stop: slice-1 supports flow_axis=+X only");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.1;
    const gp_Pnt center(in.center_x_mm, in.center_y_mm, in.center_z_mm);

    // Layout along +X:
    //   inlet  : from xMin to xInletEnd  = xMin + inlet_length
    //   seat   : from xInletEnd to xSeatEnd = xInletEnd + seat_length
    //              (cone, R_inlet → R_outlet)
    //   outlet : from xSeatEnd to xMax (outlet_length)
    //
    // We anchor the layout so that the SEAT shoulder lies at center.X().
    // seat_length = seat_drop / tan(angle).
    const double inletR  = in.inlet_dia_mm  / 2.0;
    const double outletR = in.outlet_dia_mm / 2.0;
    const double seatLen = in.seat_drop_mm /
                           std::tan(in.seat_angle_deg * M_PI / 180.0);
    const double xInletEnd = center.X();
    const double xSeatEnd  = xInletEnd + seatLen;

    // ── Sub-feature 1: Inlet bore ───────────────────────────────────────
    const gp_Ax2 inletAx(
        gp_Pnt(xMin - kOverhang, center.Y(), center.Z()),
        gp::DX());
    const TopoDS_Shape inletBore = pr::cylinder(
        inletAx, inletR, (xInletEnd - xMin) + kOverhang);

    // ── Sub-feature 2: Seat shoulder (45° conical taper) ────────────────
    const gp_Ax2 seatConeAx(
        gp_Pnt(xInletEnd, center.Y(), center.Z()),
        gp::DX());
    const TopoDS_Shape seatCone = pr::coneFrustum(
        seatConeAx, inletR, outletR, seatLen);

    // ── Sub-feature 3: Outlet bore ──────────────────────────────────────
    const gp_Ax2 outletAx(
        gp_Pnt(xSeatEnd, center.Y(), center.Z()),
        gp::DX());
    const TopoDS_Shape outletBore = pr::cylinder(
        outletAx, outletR, (xMax - xSeatEnd) + kOverhang);

    // ── Sub-feature 4: Disc-stop protrusion ─────────────────────────────
    //
    // The disc-stop is a SOLID boss that remains in the inlet bore — it
    // limits how far the disc can lift before hitting the body ceiling.
    // We model the stop by cutting EVERYTHING WITH the inlet bore EXCEPT
    // the stop region.  Concretely: the cutter is (inlet_bore − stop_cyl).
    // The stop is axially located at `stop_offset_mm` from the seat
    // (i.e. xStop = xInletEnd − stop_offset − stop_length), at the TOP of
    // the inlet bore (Z = center.Z() + inletR − stop_dia/2 − safety).
    //
    // We orient the stop along the flow axis.  Its location is on the
    // inside ceiling of the bore (Y = center.Y(), Z = center.Z() + offset).
    const double stopAxialStart = xInletEnd - in.stop_offset_mm - in.stop_length_mm;
    const double stopZ          = center.Z() + inletR - in.stop_dia_mm / 2.0 - 0.5;
    const gp_Ax2 stopAx(
        gp_Pnt(stopAxialStart, center.Y(), stopZ),
        gp::DX());
    const TopoDS_Shape stopSolid = pr::cylinder(
        stopAx, in.stop_dia_mm / 2.0, in.stop_length_mm);

    // Build the inlet cutter as bore MINUS stop (leaves the stop alive).
    const TopoDS_Shape inletWithStop = pr::cut(inletBore, stopSolid);

    // ── Fuse the bore chain (inlet + seat + outlet); cut from stock ─────
    const TopoDS_Shape bores      = pr::fuse(seatCone, outletBore);
    const TopoDS_Shape allCutters = pr::fuse(inletWithStop, bores);
    const TopoDS_Shape newShape   = pr::cut(wp.shape(), allCutters);

    // ── Signature ───────────────────────────────────────────────────────
    json params = {
        { "center_x_mm",     in.center_x_mm },
        { "center_y_mm",     in.center_y_mm },
        { "center_z_mm",     in.center_z_mm },
        { "flow_axis",       { in.flow_axis.X(), in.flow_axis.Y(), in.flow_axis.Z() } },
        { "inlet_dia_mm",    in.inlet_dia_mm },
        { "inlet_length_mm", in.inlet_length_mm },
        { "seat_angle_deg",  in.seat_angle_deg },
        { "seat_drop_mm",    in.seat_drop_mm },
        { "outlet_dia_mm",   in.outlet_dia_mm },
        { "outlet_length_mm",in.outlet_length_mm },
        { "stop_dia_mm",     in.stop_dia_mm },
        { "stop_length_mm",  in.stop_length_mm },
        { "stop_offset_mm",  in.stop_offset_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     4 },
        { "subfeatures",          { "inlet_bore", "seat_cone",
                                    "outlet_bore", "disc_stop_boss" } },
        { "conical_seat_count",   1 },
        { "coaxial_cyl_count",    2 },
        { "seat_angle_deg",       in.seat_angle_deg },
        { "inlet_dia_mm",         in.inlet_dia_mm },
        { "outlet_dia_mm",        in.outlet_dia_mm },
        { "has_disc_stop",        true },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;conical_seat_cutter";
    tooling.tool_dia_mm       = in.inlet_dia_mm;
    tooling.tool_length_mm    = in.inlet_length_mm + in.outlet_length_mm + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 280.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double inletV  = M_PI * inletR * inletR * (xInletEnd - xMin);
    const double coneV   = M_PI * seatLen / 3.0 *
                           (inletR * inletR + inletR * outletR + outletR * outletR);
    const double outletV = M_PI * outletR * outletR * (xMax - xSeatEnd);
    const double stopV   = M_PI * std::pow(in.stop_dia_mm / 2.0, 2) * in.stop_length_mm;
    tooling.stock_removed_mm3 = inletV + coneV + outletV - stopV;
    tooling.est_cycle_time_s  = std::max(8.0, tooling.stock_removed_mm3 / 1000.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "boring_bar" },          { "tool_dia_mm", in.inlet_dia_mm },
              { "note", "inlet + outlet bores" } },
            { { "tool_type", "conical_seat_cutter" }, { "tool_dia_mm", in.inlet_dia_mm },
              { "angle_deg", in.seat_angle_deg },     { "note", "seat shoulder" } },
            { { "tool_type", "edm_or_emill_relief" }, { "note", "disc-stop boss leave" } },
        } },
        { "standard", "API 594 §6.3" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::check_valve_seat_with_stop applied: inlet={} outlet={} angle={} faces {}→{}",
        in.inlet_dia_mm, in.outlet_dia_mm, in.seat_angle_deg,
        wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay
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

    // Geometric: 2 coaxial flow-axis cylinders of different dia + 1 conical
    // face between them.
    std::vector<int> flowCyls;
    std::vector<double> flowRs;
    int coneFace = -1;
    double coneAngleDeg = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        BRepAdaptor_Surface s(wp.face(i));
        if (s.GetType() == GeomAbs_Cylinder) {
            const gp_Cylinder cyl = s.Cylinder();
            const gp_Dir ad = cyl.Axis().Direction();
            if (std::abs(std::abs(ad.X()) - 1.0) < 1e-2) {
                flowCyls.push_back(i);
                flowRs.push_back(cyl.Radius());
            }
        } else if (s.GetType() == GeomAbs_Cone) {
            const gp_Cone c = s.Cone();
            const gp_Dir ad = c.Axis().Direction();
            if (std::abs(std::abs(ad.X()) - 1.0) < 1e-2) {
                coneFace     = i;
                coneAngleDeg = std::abs(c.SemiAngle()) * 180.0 / M_PI;
            }
        }
    }
    if (flowCyls.size() < 2 || coneFace < 0) return out;

    const double rMax = *std::max_element(flowRs.begin(), flowRs.end());
    const double rMin = *std::min_element(flowRs.begin(), flowRs.end());
    if (std::abs(rMax - rMin) < 1e-3) return out;  // need different radii

    json recovered = {
        { "inlet_dia_mm",   2.0 * rMax },
        { "outlet_dia_mm",  2.0 * rMin },
        { "seat_angle_deg", coneAngleDeg > 0.0 ? coneAngleDeg : 45.0 },
        { "flow_axis",      { 1.0, 0.0, 0.0 } },
    };
    json matched = {
        { "flow_cyl_count", static_cast<int>(flowCyls.size()) },
        { "cone_face_id",   coneFace },
        { "source",         "geometric" },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.7, matched });
    return out;
}

}  // namespace koocadcam::skill::check_valve_seat_with_stop
