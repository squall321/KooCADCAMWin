// @lat: [[engine/skills#p_trap_threaded_port]]

#include "p_trap_threaded_port.hpp"

#include "Workpiece.hpp"
#include "_hydraulic_ports.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::p_trap_threaded_port {

namespace pr = koocadcam::engine::prim;
namespace ht = koocadcam::skill::hyd_ports;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;

// Sequential NPT port cut: pilot bore + taper cone, both cut from `base`.
// Returns the new shape AND fills out the volume removed (approx).
TopoDS_Shape cutNptPort(const TopoDS_Shape& base,
                        const ht::NptSpec& s,
                        const gp_Pnt& entry,
                        const gp_Dir& axis,
                        double& outVol)
{
    const double drillR = s.drill_dia_mm / 2.0;
    const double depth  = s.depth_mm;
    const gp_Ax2 ax(entry, axis);

    // Pilot
    const TopoDS_Shape pilot = pr::cylinder(ax, drillR, depth + kOver);

    // Taper cone (1:16 on dia)
    const double dDelta = depth * ht::kNptTaperRatio;
    const double topR   = drillR + 0.5 * dDelta + 0.10;
    const TopoDS_Shape cone = pr::coneFrustum(ax, topR, drillR, depth + kOver);

    // FUSE concentric overlap, then a single cut (matches manifold_cross pattern).
    const TopoDS_Shape unified = pr::fuse(pilot, cone);

    outVol += M_PI / 3.0 * depth * (topR * topR + topR * drillR + drillR * drillR);
    return pr::cut(base, unified);
}

}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.inlet_thread_size_key.empty() || in.outlet_thread_size_key.empty()) {
        r.add("DFM-NPT-CODE", "error",
              "p_trap_threaded_port: thread size keys must be non-empty");
    } else {
        if (!ht::findNpt(in.inlet_thread_size_key)) {
            r.add("DFM-NPT-CODE", "error",
                  "p_trap_threaded_port: unknown inlet NPT '" +
                  in.inlet_thread_size_key + "'");
        }
        if (!ht::findNpt(in.outlet_thread_size_key)) {
            r.add("DFM-NPT-CODE", "error",
                  "p_trap_threaded_port: unknown outlet NPT '" +
                  in.outlet_thread_size_key + "'");
        }
    }
    if (in.trap_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "p_trap_threaded_port: trap_radius must be > 0");
    }
    const ht::NptSpec* sIn = ht::findNpt(in.inlet_thread_size_key);
    if (sIn && in.trap_radius_mm < sIn->drill_dia_mm * 1.5) {
        r.add("DFM-TRAP-R", "error",
              "p_trap_threaded_port: trap_radius " +
              std::to_string(in.trap_radius_mm) +
              " mm < 1.5 × inlet drill_dia (need ≥ " +
              std::to_string(sIn->drill_dia_mm * 1.5) + " mm)");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "p_trap_threaded_port DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ht::NptSpec* sIn  = ht::findNpt(in.inlet_thread_size_key);
    const ht::NptSpec* sOut = ht::findNpt(in.outlet_thread_size_key);
    if (!sIn || !sOut)
        throw SkillError("p_trap_threaded_port: thread lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;
    const double zTop = zMax;

    double volRemoved = 0.0;
    TopoDS_Shape current = wp.shape();

    // Inlet entry: on top face, axis = -Z (cutting down into stock).
    const gp_Pnt inletEntry(cx - in.trap_radius_mm, cy, zTop + kOver);
    current = cutNptPort(current, *sIn, inletEntry, gp_Dir(0, 0, -1), volRemoved);

    // ── U-shape trap channel: 2 vertical bores + 1 horizontal connector ──
    //
    // The "U" connects the inlet leg to the outlet leg through a horizontal
    // bore at the bottom.  Approximates a pipe sweep along a U-spine but is
    // robust (no MakePipe shell collapse).
    //
    // Channel diameter matches inlet drill_dia.
    const double chR = sIn->drill_dia_mm / 2.0;
    const double trapDepth = std::max(in.trap_radius_mm * 1.5,
                                      sIn->depth_mm + 4.0);
    const double zBottom = zTop - trapDepth;

    // Inlet leg vertical bore (full depth below NPT port)
    {
        const gp_Pnt p(cx - in.trap_radius_mm, cy, zBottom);
        const gp_Ax2 ax(p, gp::DZ());
        const TopoDS_Shape tool = pr::cylinder(ax, chR, trapDepth + kOver);
        current = pr::cut(current, tool);
        volRemoved += M_PI * chR * chR * trapDepth;
    }
    // Outlet leg vertical bore
    {
        const gp_Pnt p(cx + in.trap_radius_mm, cy, zBottom);
        const gp_Ax2 ax(p, gp::DZ());
        const TopoDS_Shape tool = pr::cylinder(ax, chR, trapDepth + kOver);
        current = pr::cut(current, tool);
        volRemoved += M_PI * chR * chR * trapDepth;
    }
    // Horizontal connector bore at the bottom (along +X)
    {
        const gp_Pnt p(cx - in.trap_radius_mm - kOver, cy, zBottom);
        const gp_Ax2 ax(p, gp::DX());
        const double len = 2.0 * in.trap_radius_mm + 2.0 * kOver;
        const TopoDS_Shape tool = pr::cylinder(ax, chR, len);
        current = pr::cut(current, tool);
        volRemoved += M_PI * chR * chR * (2.0 * in.trap_radius_mm);
    }

    // Outlet entry: same +X side.
    const gp_Pnt outletEntry(cx + in.trap_radius_mm, cy, zTop + kOver);
    current = cutNptPort(current, *sOut, outletEntry, gp_Dir(0, 0, -1), volRemoved);

    json params = {
        { "center_x_mm",             cx },
        { "center_y_mm",             cy },
        { "trap_radius_mm",          in.trap_radius_mm },
        { "inlet_thread_size_key",   in.inlet_thread_size_key },
        { "outlet_thread_size_key",  in.outlet_thread_size_key },
    };
    json pattern = {
        { "kind",                     kSkillId },
        { "is_compound",              true },
        { "hvac_feature_type",        "p_trap" },
        { "subfeature_count",         5 },           // inlet + 3 trap segs + outlet
        { "trap_radius_mm",           in.trap_radius_mm },
        { "inlet_thread_size_key",    in.inlet_thread_size_key },
        { "outlet_thread_size_key",   in.outlet_thread_size_key },
        { "inlet_drill_dia_mm",       sIn->drill_dia_mm },
        { "outlet_drill_dia_mm",      sOut->drill_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;npt_tap;drill;npt_tap";
    tooling.tool_dia_mm       = std::max(sIn->drill_dia_mm, sOut->drill_dia_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 100.0;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 25.0;
    tooling.extra = {
        { "standard",         "NPT taper per ANSI/ASME B1.20.1" },
        { "trap_pattern",     "U-shape 3-segment approximation of MakePipe sweep" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::p_trap_threaded_port: inlet={} outlet={} r={}",
                  in.inlet_thread_size_key, in.outlet_thread_size_key,
                  in.trap_radius_mm);

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

    // Geometric: count vertical (+Z/-Z) cylinder faces — expect ≥ 2 (legs).
    int vert = 0;
    int horiz = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.Z()) > 0.9) ++vert;
            else if (std::abs(d.X()) > 0.9) ++horiz;
        } catch (...) {}
    }
    if (vert >= 2 && horiz >= 1) {
        json recovered = { { "trap_radius_mm", 0.0 } };
        json matched   = { { "source", "geometric_u_pattern" },
                           { "vertical_cyl_count",   vert },
                           { "horizontal_cyl_count", horiz } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::p_trap_threaded_port
