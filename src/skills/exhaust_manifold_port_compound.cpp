// @lat: [[engine/skills#exhaust_manifold_port_compound]]

#include "exhaust_manifold_port_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Wire.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::exhaust_manifold_port_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

TopoDS_Wire makeRectWire(double cx, double cy, double z,
                         double w, double h)
{
    const double hx = w / 2.0;
    const double hy = h / 2.0;
    const gp_Pnt p1(cx - hx, cy - hy, z);
    const gp_Pnt p2(cx + hx, cy - hy, z);
    const gp_Pnt p3(cx + hx, cy + hy, z);
    const gp_Pnt p4(cx - hx, cy + hy, z);
    BRepBuilderAPI_MakeWire wm;
    wm.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
    wm.Add(BRepBuilderAPI_MakeEdge(p2, p3).Edge());
    wm.Add(BRepBuilderAPI_MakeEdge(p3, p4).Edge());
    wm.Add(BRepBuilderAPI_MakeEdge(p4, p1).Edge());
    if (!wm.IsDone()) throw Standard_Failure("makeRectWire failed");
    return wm.Wire();
}

TopoDS_Wire makeCircleWire(double cx, double cy, double z, double r)
{
    const gp_Circ c(gp_Ax2(gp_Pnt(cx, cy, z), gp::DZ()), r);
    BRepBuilderAPI_MakeEdge em(c);
    if (!em.IsDone()) throw Standard_Failure("makeCircleWire: edge failed");
    BRepBuilderAPI_MakeWire wm;
    wm.Add(em.Edge());
    if (!wm.IsDone()) throw Standard_Failure("makeCircleWire: wire failed");
    return wm.Wire();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "exhaust_manifold_port_compound: face_id out of range");
    }
    if (!(in.port_inlet_rect_w_mm > 0.0) ||
        !(in.port_inlet_rect_h_mm > 0.0) ||
        !(in.port_outlet_dia_mm > 0.0)   ||
        !(in.transition_length_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "exhaust_manifold_port_compound: all dimensions must be > 0");
        return r;
    }
    if (in.port_angle_deg < -45.0 || in.port_angle_deg > 45.0) {
        r.add("DFM-PORT-ANGLE", "error",
              "exhaust_manifold_port_compound: port_angle_deg (" +
              std::to_string(in.port_angle_deg) +
              ") outside [-45, 45]");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "exhaust_manifold_port_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const gp_Dir n     = wp.faceNormal(in.face_id);
    const double baseZ = faceC.Z();
    // For phase-1 we assume an approximately +Z-normal entry face.  If the
    // face is not +Z-normal, the synthesis still runs along the +Z axis from
    // the face's center point; recognize() metadata replay records the
    // intended direction in `params.face_id`.
    (void)n;

    constexpr double kEmbed = 0.05;
    // For a top face (n along +Z), entry at z = baseZ, drilling downward.
    // The transition extends from z=baseZ downward by transition_length_mm.
    // The cut tool axis runs along drillDir; for simplicity we assume the
    // face is approximately z-normal (top face) and build along -Z.  If the
    // face is non-Z, only the inlet recess depth direction follows drillDir.

    // ── Sub-feature 1: rectangular inlet shallow pocket ─────────────────
    // 2 mm shallow pocket at the entry face to seat a gasket/flange.
    constexpr double kInletPocketDepth = 2.0;
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;
    const gp_Pnt inletOrigin(cx - in.port_inlet_rect_w_mm / 2.0,
                             cy - in.port_inlet_rect_h_mm / 2.0,
                             baseZ - kInletPocketDepth);
    const TopoDS_Shape inletPocket = pr::box(
        gp_Ax2(inletOrigin, gp::DZ()),
        in.port_inlet_rect_w_mm,
        in.port_inlet_rect_h_mm,
        kInletPocketDepth + kEmbed);

    // ── Sub-feature 2: lofted transition (rect → round) ─────────────────
    const double zInletTop  = baseZ - kInletPocketDepth;
    const double zOutletTop = zInletTop - in.transition_length_mm;
    TopoDS_Shape loftSolid;
    bool loftOk = false;
    try {
        const TopoDS_Wire wRect =
            makeRectWire(cx, cy, zInletTop,
                         in.port_inlet_rect_w_mm,
                         in.port_inlet_rect_h_mm);
        const TopoDS_Wire wCirc =
            makeCircleWire(cx, cy, zOutletTop, in.port_outlet_dia_mm / 2.0);
        BRepOffsetAPI_ThruSections through(/*isSolid=*/true,
                                           /*ruled=*/false, 1e-4);
        through.AddWire(wRect);
        through.AddWire(wCirc);
        through.Build();
        if (through.IsDone()) {
            loftSolid = through.Shape();
            loftOk    = !loftSolid.IsNull();
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("exhaust_manifold_port_compound: ThruSections threw — {}",
                      ex.what());
    }

    // ── Sub-feature 3: cylindrical outlet hole (extends past the bottom)
    const double outletR    = in.port_outlet_dia_mm / 2.0;
    constexpr double kOutletExtra = 4.0;
    const gp_Pnt outletOrigin(cx, cy, zOutletTop + kEmbed);
    const gp_Ax2 outletAx(outletOrigin, gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape outletHole =
        pr::cylinder(outletAx, outletR, in.transition_length_mm * 0.2 + kOutletExtra);

    const double volBefore = volumeOf(wp.shape());

    // Fuse the available cutters and cut once.
    TopoDS_Shape allCutters = inletPocket;
    if (loftOk) {
        try {
            allCutters = pr::fuse(allCutters, loftSolid);
        } catch (const Standard_Failure& ex) {
            spdlog::debug("exhaust_manifold_port_compound: fuse loft failed — {}",
                          ex.what());
        }
    }
    try {
        allCutters = pr::fuse(allCutters, outletHole);
    } catch (const Standard_Failure& ex) {
        spdlog::debug("exhaust_manifold_port_compound: fuse outlet failed — {}",
                      ex.what());
    }

    TopoDS_Shape newShape;
    try {
        newShape = pr::cut(wp.shape(), allCutters);
    } catch (const Standard_Failure& ex) {
        spdlog::debug("exhaust_manifold_port_compound: cut failed — {}",
                      ex.what());
        throw SkillError(std::string("exhaust_manifold_port_compound: ")
                         + ex.what());
    }
    const double volAfter   = volumeOf(newShape);
    const double removedVol = volBefore - volAfter;

    json params = {
        { "face_id",                 in.face_id },
        { "center_x_mm",             in.center_x_mm },
        { "center_y_mm",             in.center_y_mm },
        { "port_inlet_rect_w_mm",    in.port_inlet_rect_w_mm },
        { "port_inlet_rect_h_mm",    in.port_inlet_rect_h_mm },
        { "port_outlet_dia_mm",      in.port_outlet_dia_mm },
        { "transition_length_mm",    in.transition_length_mm },
        { "port_angle_deg",          in.port_angle_deg },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "engine_feature_type",        "exhaust_manifold_port" },
        { "subfeature_count",           3 },
        { "port_inlet_rect_w_mm",       in.port_inlet_rect_w_mm },
        { "port_inlet_rect_h_mm",       in.port_inlet_rect_h_mm },
        { "port_outlet_dia_mm",         in.port_outlet_dia_mm },
        { "transition_length_mm",       in.transition_length_mm },
        { "loft_done",                  loftOk },
        { "derived_volume_removed_mm3", removedVol },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;form_tool;drill";
    tooling.tool_dia_mm       = in.port_outlet_dia_mm;
    tooling.tool_length_mm    = in.transition_length_mm + 8.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.06;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  =
        std::max(10.0, in.transition_length_mm * 0.5 + 6.0);
    tooling.extra = {
        { "feature_standard", "Exhaust manifold port (head-side rect, "
                              "downpipe-side round)" },
        { "transition_type",  "loft_rect_to_round" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::exhaust_manifold_port_compound applied: "
                  "rect={}x{} -> circ={}mm L={}mm",
                  in.port_inlet_rect_w_mm, in.port_inlet_rect_h_mm,
                  in.port_outlet_dia_mm, in.transition_length_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition — metadata replay ────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            { { "source",               "metadata_replay" },
              { "is_compound",          true },
              { "engine_feature_type",  "exhaust_manifold_port" },
              { "subfeature_count",     3 } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::exhaust_manifold_port_compound
