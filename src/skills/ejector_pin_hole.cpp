// @lat: [[engine/skills#ejector_pin_hole]]

#include "ejector_pin_hole.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::ejector_pin_hole {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "ejector_pin_hole dia_mm must be > 0");
    } else if (in.dia_mm < 1.0 || in.dia_mm > 25.0) {
        r.add("DFM-EJ-DIA", "error",
              "ejector_pin_hole dia " + std::to_string(in.dia_mm) +
              " mm outside [1.0, 25.0] mm standard pin range");
    }

    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "ejector_pin_hole depth_mm must be > 0");
    } else if (in.dia_mm > 0.0 && in.depth_mm < in.dia_mm * 2.0) {
        r.add("DFM-EJ-DEPTH", "warning",
              "ejector_pin_hole depth " + std::to_string(in.depth_mm) +
              " < 2 × dia — pin may not clear");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ejector_pin_hole DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("ejector_pin_hole: entry_face datum unresolved");

    BRepAdaptor_Surface entrySurf(wp.face(*entryId));
    if (entrySurf.GetType() != GeomAbs_Plane) {
        throw SkillError("ejector_pin_hole: entry_face must be planar");
    }
    const gp_Pln entryPlane = entrySurf.Plane();
    gp_Dir outward = entryPlane.Axis().Direction();
    if (wp.face(*entryId).Orientation() == TopAbs_REVERSED) outward.Reverse();
    const gp_Dir inward(-outward.X(), -outward.Y(), -outward.Z());

    // Project (position_x, position_y, 0) onto entry plane (same pattern as
    // mold_boss / mold_gate).
    auto projectOntoPlane = [&](double x, double y) {
        const gp_Pnt P0 = entryPlane.Location();
        const gp_Dir n  = entryPlane.Axis().Direction();
        if (std::abs(n.Z()) < 1e-9) return gp_Pnt(x, y, P0.Z());
        const double z = P0.Z() - (n.X() * (x - P0.X()) + n.Y() * (y - P0.Y())) / n.Z();
        return gp_Pnt(x, y, z);
    };
    const gp_Pnt onFace = projectOntoPlane(in.position_x_mm, in.position_y_mm);

    // Start the cutter slightly OUTSIDE the workpiece to guarantee a clean
    // Boolean (avoid coplanar-face issues at the entry plane), and extend
    // by the same overhang on the far side.
    constexpr double kOverhang = 0.05;
    const gp_Pnt start(
        onFace.X() - inward.X() * kOverhang,
        onFace.Y() - inward.Y() * kOverhang,
        onFace.Z() - inward.Z() * kOverhang);

    // gp_Ax2 V = local Z (pitfall): cylinder grows along `inward`.
    const gp_Ax2 ax(start, inward);
    const TopoDS_Shape cutter = pr::cylinder(ax, in.dia_mm / 2.0,
                                              in.depth_mm + 2.0 * kOverhang);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_kind", "resolved_id" },
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "dia_mm",          in.dia_mm },
        { "depth_mm",        in.depth_mm },
    };
    json pattern = {
        { "kind",       kSkillId },
        { "dia_mm",     in.dia_mm },
        { "depth_mm",   in.depth_mm },
        { "through",    true },
        { "axis_dir",   { inward.X(), inward.Y(), inward.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "drill";
    tooling.tool_dia_mm      = in.dia_mm;
    tooling.tool_length_mm   = in.depth_mm;
    const double r = in.dia_mm / 2.0;
    tooling.stock_removed_mm3 = M_PI * r * r * in.depth_mm;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::ejector_pin_hole applied: dia={} depth={} faces {}→{}",
                  in.dia_mm, in.depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay (primary path).
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "position_x_mm", f.params.value("position_x_mm", 0.0) },
            { "position_y_mm", f.params.value("position_y_mm", 0.0) },
            { "dia_mm",        f.params.value("dia_mm",        0.0) },
            { "depth_mm",      f.params.value("depth_mm",      0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.95, json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Topology fallback: look for cylindrical faces with axis ±Z and dia in
    // the ejector-pin range that span the workpiece in Z (= "through").
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thicknessZ = zMax - zMin;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface surf(wp.face(fIdx));
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir adir = cyl.Axis().Direction();
        if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-2) continue;
        const double dia = 2.0 * cyl.Radius();
        if (dia < 1.0 || dia > 25.0) continue;
        const gp_Pnt loc = cyl.Axis().Location();

        json rec = {
            { "position_x_mm", loc.X() },
            { "position_y_mm", loc.Y() },
            { "dia_mm",        dia },
            { "depth_mm",      thicknessZ },
        };
        json matched = { { "cyl_face_id", fIdx } };
        out.push_back(RecognizedFeature{ kSkillId, rec, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::ejector_pin_hole
