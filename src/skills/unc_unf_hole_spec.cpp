// @lat: [[engine/skills#unc_unf_hole_spec]]

#include "unc_unf_hole_spec.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::unc_unf_hole_spec {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── ASME B18.2.8 Table 1 clearance hole data ────────────────────────────
//
// Sourced from the central thread table (kUncUnfThreads in
// _iso_thread_table.hpp).  ASME B18.2.8 fit classes:
//   normal → clearance_normal_mm
//   close  → clearance_close_mm
//   loose  → clearance_loose_mm

double clearanceDiameterFor(const std::string& fastener_size,
                            const std::string& fit_class)
{
    const auto* e = thread_table::findUncUnf(fastener_size);
    if (!e) return 0.0;
    if (fit_class == "normal") return e->clearance_normal_mm;
    if (fit_class == "close")  return e->clearance_close_mm;
    if (fit_class == "loose")  return e->clearance_loose_mm;
    return 0.0;
}

namespace {

struct ReverseMatch { std::string size; std::string fit; double err; };

ReverseMatch closestSpecMatch(double dia_mm, double tol_mm)
{
    ReverseMatch best { {}, {}, tol_mm };
    for (const auto& e : thread_table::kUncUnfThreads) {
        const struct { const char* fit; double d; } cands[3] = {
            { "normal", e.clearance_normal_mm },
            { "close",  e.clearance_close_mm  },
            { "loose",  e.clearance_loose_mm  },
        };
        for (const auto& c : cands) {
            const double err = std::abs(c.d - dia_mm);
            if (err < best.err) { best.err = err; best.size = e.size_key; best.fit = c.fit; }
        }
    }
    return best;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.fastener_size.empty()) {
        r.add("DFM-IMP-UNKNOWN", "error",
              "unc_unf_hole_spec: fastener_size is empty");
        return r;
    }
    if (in.fit_class != "normal" && in.fit_class != "close" && in.fit_class != "loose") {
        r.add("DFM-IMP-FIT", "error",
              "unc_unf_hole_spec: fit_class '" + in.fit_class +
              "' not in {normal, close, loose} per ASME B18.2.8");
        return r;
    }
    const double clrDia = clearanceDiameterFor(in.fastener_size, in.fit_class);
    if (clrDia <= 0.0) {
        r.add("DFM-IMP-UNKNOWN", "error",
              "unc_unf_hole_spec: unknown fastener_size '" + in.fastener_size +
              "' (ASME B18.2.8 supports 8 sizes #2-56..1/2-13)");
        return r;
    }
    if (in.chamfer_angle_deg < 15.0 || in.chamfer_angle_deg > 75.0) {
        r.add("DFM-011", "error",
              "unc_unf_hole_spec: chamfer angle " +
              std::to_string(in.chamfer_angle_deg) +
              " outside [15°, 75°]");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness =
        std::abs(in.axis_dir.X()) * (xMax - xMin) +
        std::abs(in.axis_dir.Y()) * (yMax - yMin) +
        std::abs(in.axis_dir.Z()) * (zMax - zMin);
    if (thickness > 0.0 && thickness < clrDia) {
        r.add("DFM-IMP-DEPTH", "warning",
              "unc_unf_hole_spec: stock_thickness " + std::to_string(thickness) +
              " < clearance_dia " + std::to_string(clrDia));
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "unc_unf_hole_spec DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double clrDia = clearanceDiameterFor(in.fastener_size, in.fit_class);
    const double clrR   = clrDia / 2.0;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const gp_Dir adir = in.axis_dir;
    const double kOverhang = 0.05;

    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0)
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
        else
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kOverhang);
    } else {
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * (bboxDiag + kOverhang),
            in.position_y_mm - adir.Y() * (bboxDiag + kOverhang),
            (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kOverhang));
    }

    const gp_Ax2 toolAx(toolStart, adir);

    const double throughLen = bboxDiag + 2.0 * kOverhang;
    const TopoDS_Shape throughTool = pr::cylinder(toolAx, clrR, throughLen);

    const double leg = std::max(in.chamfer_size_mm, 0.0);
    const double chamferDepth = (leg > 0.0)
        ? leg / std::tan(in.chamfer_angle_deg * M_PI / 180.0)
        : 0.0;
    const double chamferBigR  = clrR + leg;
    TopoDS_Shape chamferTool;
    if (leg > 1e-6 && chamferDepth > 1e-6) {
        chamferTool = pr::coneFrustum(toolAx, chamferBigR, clrR, chamferDepth);
    }

    TopoDS_Shape fusedCutter = throughTool;
    if (!chamferTool.IsNull()) {
        fusedCutter = pr::fuse(throughTool, chamferTool);
    }
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedCutter);

    json params = {
        { "position_x_mm",     in.position_x_mm },
        { "position_y_mm",     in.position_y_mm },
        { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
        { "fastener_size",     in.fastener_size },
        { "fit_class",         in.fit_class },
        { "chamfer_size_mm",   in.chamfer_size_mm },
        { "chamfer_angle_deg", in.chamfer_angle_deg },
        { "clearance_dia_mm",  clrDia },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           2 },
        { "fastener_size",              in.fastener_size },
        { "fit_class",                  in.fit_class },
        { "clearance_dia_mm",           clrDia },
        { "chamfer_dia_mm",             2.0 * chamferBigR },
        { "chamfer_depth_mm",           chamferDepth },
        { "cylindrical_face_count",     1 },
        { "conical_face_count",         (leg > 0.0) ? 1 : 0 },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "drill;chamfer_mill";
    tooling.tool_dia_mm      = clrDia;
    tooling.tool_length_mm   = throughLen + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = M_PI * clrR * clrR * throughLen;
    tooling.est_cycle_time_s  = std::max(1.0, throughLen / 50.0);
    tooling.extra = {
        { "asme_spec", "ASME B18.2.8" },
        { "two_cut_sequence", {
            { { "tool_type", "drill" },        { "tool_dia_mm", clrDia } },
            { { "tool_type", "chamfer_mill" }, { "leg_mm",      leg    } },
        } },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::unc_unf_hole_spec applied: {} {} → clr {:.2f} mm",
                  in.fastener_size, in.fit_class, clrDia);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 0.99;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double dia = 2.0 * cyl.Radius();
        const ReverseMatch m = closestSpecMatch(dia, 0.20);
        if (m.size.empty()) continue;

        gp_Pnt entryCenter = cyl.Axis().Location();
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            entryCenter = crv.Circle().Location();
            break;
        }

        const gp_Dir adir = cyl.Axis().Direction();
        json params = {
            { "position_x_mm",     entryCenter.X() },
            { "position_y_mm",     entryCenter.Y() },
            { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
            { "fastener_size",     m.size },
            { "fit_class",         m.fit },
            { "clearance_dia_mm",  dia },
            { "chamfer_size_mm",   0.5 },
            { "chamfer_angle_deg", 45.0 },
        };
        out.push_back(RecognizedFeature{
            kSkillId, params, 0.75 - std::min(0.2, m.err),
            { { "cyl_face_id", fIdx }, { "asme", "ASME B18.2.8" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::unc_unf_hole_spec
