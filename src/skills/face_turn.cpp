// @lat: [[engine/skills#face_turn]]

#include "face_turn.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::face_turn {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (wp.shape().IsNull()) {
        r.add("DFM-INPUT", "error", "face_turn: workpiece shape is null");
        return r;
    }

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    if (in.face_z_mm >= bb.zMax - 1e-6) {
        r.add("DFM-INPUT", "error",
              "face_turn face_z_mm (" + std::to_string(in.face_z_mm) +
              ") must be < current zMax (" + std::to_string(bb.zMax) +
              ") — nothing to remove");
    }
    if (in.face_z_mm <= bb.zMin + 1e-6) {
        r.add("DFM-INPUT", "error",
              "face_turn face_z_mm (" + std::to_string(in.face_z_mm) +
              ") must be > zMin (" + std::to_string(bb.zMin) +
              ") — would destroy the part");
    }

    // Surface finish is informational; recognised codes (info only).
    const bool finishOk =
        (in.surface_finish == "ra_0.4" ||
         in.surface_finish == "ra_0.8" ||
         in.surface_finish == "ra_1.6" ||
         in.surface_finish == "ra_3.2");
    if (!finishOk) {
        r.add("DFM-FINISH", "warning",
              "face_turn surface_finish '" + in.surface_finish +
              "' not in {ra_0.4, ra_0.8, ra_1.6, ra_3.2}");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "face_turn DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double outerR = bb.outerRadiusXY();

    // Build a slab cutter: full-OD cylinder from face_z up to zMax + overhang.
    // We extend the cutter radius slightly to safely cover any bbox imprecision
    // on the outer perimeter.
    constexpr double kOverhangZ = 0.05;
    constexpr double kOverhangR = 0.5;
    const double startZ = in.face_z_mm;        // ← bottom of the slab cutter
    const double height = (bb.zMax + kOverhangZ) - startZ;

    const gp_Ax2 ax(gp_Pnt(0.0, 0.0, startZ), gp::DZ());
    const TopoDS_Shape cutter = pr::cylinder(ax, outerR + kOverhangR, height);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double removedH = bb.zMax - in.face_z_mm;

    json params = {
        { "face_z_mm",      in.face_z_mm },
        { "surface_finish", in.surface_finish },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "top_planar_face_at_z", in.face_z_mm },
        { "top_normal",          { 0.0, 0.0, 1.0 } },
        { "axis",                { 0.0, 0.0, 1.0 } },
        { "face_z_mm",           in.face_z_mm },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "lathe_face_insert";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = outerR + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 700.0;
    tooling.feed_per_tooth_mm = (in.surface_finish == "ra_0.4") ? 0.05
                              : (in.surface_finish == "ra_0.8") ? 0.08
                              : (in.surface_finish == "ra_1.6") ? 0.12
                              : 0.15;
    tooling.stock_removed_mm3 = M_PI * outerR * outerR * std::max(0.0, removedH);
    tooling.est_cycle_time_s  = std::max(1.0, outerR / 30.0);
    tooling.extra = json{ { "surface_finish", in.surface_finish } };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::face_turn applied: face_z={} (was zMax={}) faces {}→{}",
                  in.face_z_mm, bb.zMax, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: the top of the workpiece is a planar disc face whose normal is
// +Z, located at z ≈ zMax, with a circular boundary edge whose radius is
// ≈ bbox outer radius.  This is the squared face.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    if (wp.shape().IsNull()) return out;

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double outerR = bb.outerRadiusXY();
    if (outerR <= 1e-6) return out;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFacePlanar(fIdx)) continue;
        const TopoDS_Face& f = wp.face(fIdx);
        BRepAdaptor_Surface s(f);
        const gp_Pln pln = s.Plane();
        const gp_Dir n = pln.Axis().Direction();
        // Normal must be +Z (top face).
        if (n.Z() < 0.99) continue;
        // Plane Z must coincide with bbox zMax (within tolerance).
        const double planeZ = pln.Location().Z();
        if (std::abs(planeZ - bb.zMax) > 0.1) continue;

        // Verify the face has a circular boundary edge with radius ≈ outerR.
        bool hasOuterCircle = false;
        double recoveredR = outerR;
        for (TopExp_Explorer exp(f, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Z()) - 1.0) > 1e-3) continue;
            if (std::abs(c.Radius() - outerR) < 0.3) {
                hasOuterCircle = true;
                recoveredR     = c.Radius();
                break;
            }
        }
        if (!hasOuterCircle) continue;

        json recovered = {
            { "face_z_mm",      planeZ },
            { "surface_finish", "ra_1.6" },  // default — finish not directly observable
        };
        json matched = {
            { "face_id",        fIdx },
            { "recovered_R_mm", recoveredR },
            { "plane_z_mm",     planeZ },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.80, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::face_turn
