// @lat: [[engine/skills#internal_water_jacket]]

#include "internal_water_jacket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/HelicalSweep.hpp"
#include "engine/primitives/Tools.hpp"

#include <Standard_Failure.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::internal_water_jacket {

namespace pr = koocadcam::engine::prim;
namespace eng = koocadcam::engine;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bore_inner_r_mm <= 0.0 || in.outer_r_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "internal_water_jacket: bore_inner_r and outer_r must be > 0");
    }
    if (in.outer_r_mm <= in.bore_inner_r_mm) {
        r.add("DFM-JACKET-GEOM", "error",
              "internal_water_jacket: outer_r " + std::to_string(in.outer_r_mm) +
              " <= bore_inner_r " + std::to_string(in.bore_inner_r_mm));
    }
    if (in.groove_depth_mm <= 0.0 || in.groove_w_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "internal_water_jacket: groove_w and groove_depth must be > 0");
    }
    const double wallThickness = in.outer_r_mm - in.bore_inner_r_mm;
    // Reserve at least 0.8 mm wall outside the groove for structural integrity.
    if (in.groove_depth_mm + 0.8 >= wallThickness) {
        r.add("DFM-JACKET-GEOM", "error",
              "internal_water_jacket: groove_depth " + std::to_string(in.groove_depth_mm) +
              " leaves < 0.8 mm wall against outer surface");
    }
    if (in.jacket_pitch_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "internal_water_jacket: jacket_pitch_mm must be > 0");
    }
    if (in.jacket_pitch_mm < 2.0 * in.groove_w_mm) {
        r.add("DFM-JACKET-PITCH", "error",
              "internal_water_jacket: pitch " + std::to_string(in.jacket_pitch_mm) +
              " < 2× groove_w " + std::to_string(in.groove_w_mm) +
              " — helical turns would overlap");
    }
    if (in.jacket_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "internal_water_jacket: jacket_length_mm must be > 0");
    }

    if (in.port_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "internal_water_jacket: port_dia " + std::to_string(in.port_dia_mm) +
              " mm < min 0.8 mm");
    }

    // Inlet/outlet axial separation must exceed groove_w so they don't share a turn.
    if (std::abs(in.outlet_axial_pos_mm - in.inlet_axial_pos_mm) < in.groove_w_mm) {
        r.add("DFM-JACKET-PORT-CLEAR", "error",
              "internal_water_jacket: inlet/outlet axial separation < groove_w "
              "(ports would clash within one helical turn)");
    }
    if (in.inlet_axial_pos_mm < 0.0 ||
        in.inlet_axial_pos_mm > in.jacket_length_mm) {
        r.add("DFM-JACKET-PORT-CLEAR", "error",
              "internal_water_jacket: inlet_axial_pos outside jacket length");
    }
    if (in.outlet_axial_pos_mm < 0.0 ||
        in.outlet_axial_pos_mm > in.jacket_length_mm) {
        r.add("DFM-JACKET-PORT-CLEAR", "error",
              "internal_water_jacket: outlet_axial_pos outside jacket length");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "internal_water_jacket DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Bore axis frame.
    const gp_Pnt boreOrigin(
        in.bore_origin_x_mm + in.bore_axis.X() * in.bore_axial_start_mm,
        in.bore_origin_y_mm + in.bore_axis.Y() * in.bore_axial_start_mm,
        0.0                  + in.bore_axis.Z() * in.bore_axial_start_mm);
    const gp_Ax1 boreAx(boreOrigin, in.bore_axis);

    // Local in-plane basis (used for port radial directions).
    const gp_Dir zLoc = in.bore_axis;
    gp_Dir xLoc;
    {
        const gp_Dir seed = (std::abs(zLoc.Z()) < 0.9) ? gp_Dir(0, 0, 1) : gp_Dir(1, 0, 0);
        gp_Vec vx = gp_Vec(seed).Crossed(gp_Vec(zLoc));
        if (vx.Magnitude() < 1e-9) vx = gp_Vec(1, 0, 0);
        xLoc = gp_Dir(vx);
    }
    const gp_Dir yLoc(gp_Vec(zLoc).Crossed(gp_Vec(xLoc)));

    // ── Sub-cut #1: helical jacket groove ────────────────────────────────
    //
    // The groove is a ring of radial thickness `groove_depth_mm` swept
    // helically with pitch `jacket_pitch_mm` for `jacket_length_mm`.  We
    // use the existing helicalSweep primitive (designed for threads — it
    // also models cooling grooves correctly when the depth ≈ groove_w).
    eng::HelicalSweepProfile profileTag = eng::HelicalSweepProfile::AnnularFallback;
    TopoDS_Shape jacketCutter;
    try {
        jacketCutter = pr::helicalSweep(
            boreAx,
            in.jacket_pitch_mm,
            in.jacket_length_mm,
            in.bore_inner_r_mm,
            in.groove_depth_mm,
            &profileTag);
    }
    catch (const Standard_Failure& e) {
        throw SkillError(std::string("internal_water_jacket: helical sweep failed: ") + e.what());
    }

    // ── Sub-cuts #2 & #3: inlet + outlet radial ports ────────────────────
    auto portAtClock = [&](double axialPos, double clockDeg) {
        const double rad = clockDeg * M_PI / 180.0;
        const gp_Dir radialDir(
            std::cos(rad) * xLoc.X() + std::sin(rad) * yLoc.X(),
            std::cos(rad) * xLoc.Y() + std::sin(rad) * yLoc.Y(),
            std::cos(rad) * xLoc.Z() + std::sin(rad) * yLoc.Z());
        // The port goes from OUTSIDE (radius > outer_r) inward to bore_inner_r.
        const double portLen = (in.outer_r_mm - in.bore_inner_r_mm) + 2.0;
        // Start outside the outer wall along radialDir (i.e. start point at
        // radius outer_r + 1.0, then cut INWARD = -radialDir).
        const gp_Pnt portStart(
            boreOrigin.X() + zLoc.X() * axialPos + radialDir.X() * (in.outer_r_mm + 1.0),
            boreOrigin.Y() + zLoc.Y() * axialPos + radialDir.Y() * (in.outer_r_mm + 1.0),
            boreOrigin.Z() + zLoc.Z() * axialPos + radialDir.Z() * (in.outer_r_mm + 1.0));
        // Cut direction is -radialDir (inward).
        const gp_Dir cutDir(-radialDir.X(), -radialDir.Y(), -radialDir.Z());
        const gp_Ax2 ax(portStart, cutDir);
        return pr::cylinder(ax, in.port_dia_mm / 2.0, portLen);
    };

    const TopoDS_Shape inletTool  = portAtClock(in.inlet_axial_pos_mm,  in.inlet_clock_deg);
    const TopoDS_Shape outletTool = portAtClock(in.outlet_axial_pos_mm, in.outlet_clock_deg);

    // Fuse everything into one cutter then subtract.
    const TopoDS_Shape combined = pr::fuseMany(jacketCutter, { inletTool, outletTool });
    const TopoDS_Shape newShape = pr::cut(wp.shape(), combined);

    // ── Signature ───────────────────────────────────────────────────────
    json params = {
        { "bore_axis", { in.bore_axis.X(), in.bore_axis.Y(), in.bore_axis.Z() } },
        { "bore_origin_x_mm",    in.bore_origin_x_mm },
        { "bore_origin_y_mm",    in.bore_origin_y_mm },
        { "bore_axial_start_mm", in.bore_axial_start_mm },
        { "bore_inner_r_mm",     in.bore_inner_r_mm },
        { "outer_r_mm",          in.outer_r_mm },
        { "jacket_length_mm",    in.jacket_length_mm },
        { "jacket_pitch_mm",     in.jacket_pitch_mm },
        { "groove_w_mm",         in.groove_w_mm },
        { "groove_depth_mm",     in.groove_depth_mm },
        { "port_dia_mm",         in.port_dia_mm },
        { "inlet_axial_pos_mm",  in.inlet_axial_pos_mm },
        { "outlet_axial_pos_mm", in.outlet_axial_pos_mm },
        { "inlet_clock_deg",     in.inlet_clock_deg },
        { "outlet_clock_deg",    in.outlet_clock_deg },
    };
    const int turnCount = static_cast<int>(std::ceil(in.jacket_length_mm / in.jacket_pitch_mm));
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    3 },                         // groove + 2 ports
        { "jacket_pitch_mm",     in.jacket_pitch_mm },
        { "groove_w_mm",         in.groove_w_mm },
        { "groove_depth_mm",     in.groove_depth_mm },
        { "jacket_length_mm",    in.jacket_length_mm },
        { "port_dia_mm",         in.port_dia_mm },
        { "turn_count",          turnCount },                 // derived
        { "wall_remaining_mm",   in.outer_r_mm - in.bore_inner_r_mm - in.groove_depth_mm },
        { "sweep_profile",       eng::helicalSweepProfileTag(profileTag) },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "thread_mill;drill;drill";
    tooling.tool_dia_mm       = in.groove_w_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.est_cycle_time_s  = std::max(5.0, in.jacket_length_mm * 0.3 + 4.0);
    const double grooveVol = 2.0 * M_PI * in.bore_inner_r_mm * in.groove_depth_mm
                           * in.groove_w_mm * turnCount;
    const double portsVol  = 2.0 * M_PI * (in.port_dia_mm/2.0) * (in.port_dia_mm/2.0)
                           * (in.outer_r_mm - in.bore_inner_r_mm);
    tooling.stock_removed_mm3 = grooveVol + portsVol;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "thread_mill" }, { "tool_dia_mm", in.groove_w_mm },
              { "pitch_mm", in.jacket_pitch_mm }, { "depth_mm", in.groove_depth_mm } },
            { { "tool_type", "drill" }, { "tool_dia_mm", in.port_dia_mm },
              { "role", "inlet_port" } },
            { { "tool_type", "drill" }, { "tool_dia_mm", in.port_dia_mm },
              { "role", "outlet_port" } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::internal_water_jacket applied: pitch={} groove_w={} ports@{},{}",
                  in.jacket_pitch_mm, in.groove_w_mm,
                  in.inlet_axial_pos_mm, in.outlet_axial_pos_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

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
    return out;
}

}  // namespace koocadcam::skill::internal_water_jacket
