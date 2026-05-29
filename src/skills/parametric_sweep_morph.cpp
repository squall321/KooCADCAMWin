// @lat: [[engine/skills#parametric_sweep_morph]]

#include "parametric_sweep_morph.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::parametric_sweep_morph {

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

TopoDS_Wire buildCircleWire(const gp_Pnt& centre,
                            const gp_Dir& axis,
                            double radius)
{
    const gp_Ax2 ax(centre, axis);
    const gp_Circ circle(ax, radius);
    BRepBuilderAPI_MakeEdge em(circle);
    if (!em.IsDone())
        throw Standard_Failure(
            "parametric_sweep_morph: circle edge build failed");
    BRepBuilderAPI_MakeWire wm(em.Edge());
    if (!wm.IsDone())
        throw Standard_Failure(
            "parametric_sweep_morph: wire build failed");
    return wm.Wire();
}

}  // namespace

// ── validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.start_radius <= 0.0 || in.end_radius <= 0.0) {
        r.add("DFM-INPUT", "error",
              "parametric_sweep_morph: start_radius / end_radius must be > 0");
        return r;
    }
    const double pathLen = in.path_start.Distance(in.path_end);
    if (pathLen < 1e-6) {
        r.add("DFM-INPUT", "error",
              "parametric_sweep_morph: path_start coincides with path_end");
    }
    if (in.intermediate_count < 2) {
        r.add("DFM-INPUT", "error",
              "parametric_sweep_morph: intermediate_count must be ≥ 2");
    }
    if (in.start_radius < 0.4 || in.end_radius < 0.4) {
        r.add("DFM-SWEEP-MIN-D", "error",
              "parametric_sweep_morph: radius < 0.4 mm violates "
              "min-cutter gate (start=" + std::to_string(in.start_radius) +
              ", end=" + std::to_string(in.end_radius) + ")");
    }
    const double rMin = std::min(in.start_radius, in.end_radius);
    const double rMax = std::max(in.start_radius, in.end_radius);
    if (rMin > 0.0 && rMax / rMin > 4.0) {
        r.add("DFM-SWEEP-TAPER", "warning",
              "parametric_sweep_morph: radius ratio " +
              std::to_string(rMax / rMin) +
              ":1 > 4:1 — multi-tool sequence advised (ISO 13399)");
    }
    return r;
}

// ── synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "parametric_sweep_morph DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Path direction & length.
    const gp_Vec pathVec(in.path_start, in.path_end);
    const double pathLen = pathVec.Magnitude();
    const gp_Dir pathDir(pathVec);
    const int N = std::max(2, in.intermediate_count);

    // 1+2+3) Build N sections at sample stations 0, 1/(N-1), …, 1.
    std::vector<TopoDS_Wire> wires;
    wires.reserve(static_cast<size_t>(N));
    try {
        for (int k = 0; k < N; ++k) {
            const double s = static_cast<double>(k) /
                             static_cast<double>(N - 1);
            const gp_Pnt centre(
                in.path_start.X() + (in.path_end.X() - in.path_start.X()) * s,
                in.path_start.Y() + (in.path_end.Y() - in.path_start.Y()) * s,
                in.path_start.Z() + (in.path_end.Z() - in.path_start.Z()) * s);
            const double radius =
                in.start_radius + (in.end_radius - in.start_radius) * s;
            wires.push_back(buildCircleWire(centre, pathDir, radius));
        }
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("parametric_sweep_morph: ") + ex.what());
    }

    // 4) ThruSections through all waypoints → sweep solid.
    TopoDS_Shape sweepSolid;
    try {
        BRepOffsetAPI_ThruSections through(true, false, 1.0e-4);
        for (const auto& w : wires) through.AddWire(w);
        through.Build();
        if (!through.IsDone())
            throw Standard_Failure(
                "parametric_sweep_morph: ThruSections.Build failed");
        sweepSolid = through.Shape();
        if (sweepSolid.IsNull())
            throw Standard_Failure(
                "parametric_sweep_morph: ThruSections produced null shape");
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("parametric_sweep_morph: ") + ex.what());
    }

    // 5) Boolean cut against the workpiece.
    TopoDS_Shape newShape;
    try {
        newShape = pr::cut(wp.shape(), sweepSolid);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("parametric_sweep_morph: cut failed: ") +
                         ex.what());
    }

    const double removed = std::max(0.0, volumeOf(sweepSolid));

    json params = {
        { "path_start",         { in.path_start.X(), in.path_start.Y(), in.path_start.Z() } },
        { "path_end",           { in.path_end.X(),   in.path_end.Y(),   in.path_end.Z()   } },
        { "start_radius",       in.start_radius },
        { "end_radius",         in.end_radius },
        { "intermediate_count", N },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     N + 1 },     // N waypoints + 1 cut
        { "path_length_mm",       pathLen },
        { "sweep_volume_mm3",     removed },
        { "start_profile",        { { "radius", in.start_radius } } },
        { "end_profile",          { { "radius", in.end_radius   } } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "tapered_endmill";
    tooling.tool_dia_mm       = 2.0 * std::max(in.start_radius, in.end_radius);
    tooling.tool_length_mm    = pathLen + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = removed;
    tooling.est_cycle_time_s  = std::max(2.0, removed / 800.0);
    tooling.extra = {
        { "operation_chain", {
            "start profile: circle r=" + std::to_string(in.start_radius),
            std::to_string(N - 2) + " intermediate profiles",
            "end profile: circle r=" + std::to_string(in.end_radius),
            "ThruSections sweep",
            "Boolean cut",
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::parametric_sweep_morph applied: r=({}→{}) len={} removed={:.2f}",
                  in.start_radius, in.end_radius, pathLen, removed);

    return SkillOutput{ wpNew, sig };
}

// ── recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rp = {
            { "path_start",         f.params.value("path_start",
                                       json::array({0,0,0})) },
            { "path_end",           f.params.value("path_end",
                                       json::array({0,0,0})) },
            { "start_radius",       f.params.value("start_radius", 0.0) },
            { "end_radius",         f.params.value("end_radius",   0.0) },
            { "intermediate_count", f.params.value("intermediate_count", 3) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rp, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback: at least one non-planar non-cylindrical face hints
    // at a taper-swept residue.
    int freeform = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i) && !wp.isFaceCylinder(i)) ++freeform;
    }
    if (freeform >= 1) {
        RecognizedFeature r;
        r.skill_id = kSkillId;
        r.recovered_params = json{
            { "start_radius",       0.0 },
            { "end_radius",         0.0 },
            { "intermediate_count", 3 },
        };
        r.confidence = 0.55;
        r.matched_geometry = json{
            { "freeform_face_count", freeform },
            { "note", "geometric sweep guess; no history" },
        };
        out.push_back(std::move(r));
    }
    return out;
}

}  // namespace koocadcam::skill::parametric_sweep_morph
