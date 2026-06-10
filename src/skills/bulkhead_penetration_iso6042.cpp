// @lat: [[engine/skills#bulkhead_penetration_iso6042]]

#include "bulkhead_penetration_iso6042.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::bulkhead_penetration_iso6042 {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.penetration_dia_mm <= 0.0 ||
        in.bulkhead_thickness_mm <= 0.0 ||
        in.fire_seal_ring_dia_mm <= 0.0 ||
        in.fire_seal_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "bulkhead_penetration_iso6042: all dimensions must be > 0");
    }

    if (in.bulkhead_thickness_mm < 3.0) {
        r.add("DFM-MARINE-PLATE", "error",
              "bulkhead_thickness_mm < 3.0 mm (marine bulkhead minimum)");
    }

    if (in.fire_seal_ring_dia_mm <= in.penetration_dia_mm + 4.0) {
        r.add("DFM-FIRE-SEAL", "error",
              "fire_seal_ring_dia must exceed penetration_dia by ≥ 4 mm "
              "(annular seal width ≥ 2 mm)");
    }

    if (in.fire_seal_depth_mm >= in.bulkhead_thickness_mm * 0.5) {
        r.add("DFM-FIRE-SEAL", "error",
              "fire_seal_depth must be < bulkhead_thickness / 2 "
              "(opposing pockets must not meet)");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bulkhead_penetration_iso6042 DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Dir adir = in.axis_dir;
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;

    if (!(std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6 &&
          adir.Z() > 0)) {
        throw SkillError(
            "bulkhead_penetration_iso6042: only axis_dir = +Z is supported");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ    = zMax;
    const double bottomZ = topZ - in.bulkhead_thickness_mm;
    const double kOver   = 0.1;

    const double boreR     = in.penetration_dia_mm / 2.0;
    const double sealR     = in.fire_seal_ring_dia_mm / 2.0;
    const double sealInnerR = boreR;          // annular pocket touches bore wall
    const double depth     = in.fire_seal_depth_mm;

    // ── A) central through-bore ────────────────────────────────────────
    const gp_Ax2 axBore(gp_Pnt(cx, cy, bottomZ - kOver), adir);
    const TopoDS_Shape boreTool = pr::cylinder(
        axBore, boreR, in.bulkhead_thickness_mm + 2.0 * kOver);
    const TopoDS_Shape afterBore = pr::cut(wp.shape(), boreTool);

    // ── B) entry-face (top) annular fire-seal pocket ───────────────────
    const gp_Ax2 axTopSeal(gp_Pnt(cx, cy, topZ - depth), adir);
    const TopoDS_Shape topSealTool = pr::annularRing(
        axTopSeal, sealR, sealInnerR, depth + kOver);
    const TopoDS_Shape afterTopSeal = pr::cut(afterBore, topSealTool);

    // ── C) exit-face (bottom) annular fire-seal pocket ─────────────────
    const gp_Ax2 axBotSeal(gp_Pnt(cx, cy, bottomZ - kOver), adir);
    const TopoDS_Shape botSealTool = pr::annularRing(
        axBotSeal, sealR, sealInnerR, depth + kOver);
    const TopoDS_Shape newShape = pr::cut(afterTopSeal, botSealTool);

    // ── signature ──────────────────────────────────────────────────────
    json params = {
        { "center_x_mm",           cx },
        { "center_y_mm",           cy },
        { "axis_dir",              { adir.X(), adir.Y(), adir.Z() } },
        { "penetration_dia_mm",    in.penetration_dia_mm },
        { "bulkhead_thickness_mm", in.bulkhead_thickness_mm },
        { "fire_seal_ring_dia_mm", in.fire_seal_ring_dia_mm },
        { "fire_seal_depth_mm",    in.fire_seal_depth_mm },
    };

    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "marine_feature_type",   "bulkhead_fire_penetration" },
        { "subfeature_count",      3 },
        { "penetration_dia_mm",    in.penetration_dia_mm },
        { "fire_seal_ring_dia_mm", in.fire_seal_ring_dia_mm },
        { "fire_seal_depth_mm",    in.fire_seal_depth_mm },
        { "fire_seal_annular_id_mm", 2.0 * sealInnerR },
        { "fire_seal_annular_od_mm", 2.0 * sealR },
        { "axis_dir",              { adir.X(), adir.Y(), adir.Z() } },
        { "standard",              "ISO 6042 marine fire-rated penetration" },
    };

    ToolingMeta tooling;
    tooling.tool_type = "drill;groove_cutter";
    tooling.tool_dia_mm = std::max(in.penetration_dia_mm, in.fire_seal_ring_dia_mm);
    tooling.tool_material = "carbide";
    tooling.flute_count = 4;
    tooling.cutting_speed_sfm = 200.0;
    const double vBore = M_PI * boreR * boreR * in.bulkhead_thickness_mm;
    const double vSeal = 2.0 * M_PI *
        (sealR * sealR - sealInnerR * sealInnerR) * depth;
    tooling.stock_removed_mm3 = vBore + vSeal;
    tooling.est_cycle_time_s = 60.0;
    tooling.extra = {
        { "removed_volume_mm3", vBore + vSeal },
        { "standard",           "ISO 6042 marine fire transit" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::bulkhead_penetration_iso6042: bore={} seal={} faces {}→{}",
                  in.penetration_dia_mm, in.fire_seal_ring_dia_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

namespace {

struct CylRec {
    int    faceIdx = -1;
    double radius = 0.0;
    gp_Ax1 axis;
};

std::vector<CylRec> collectCylinders(const Workpiece& wp)
{
    std::vector<CylRec> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder c = s.Cylinder();
        out.push_back({ i, c.Radius(), c.Axis() });
    }
    return out;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay first.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence = 1.0;
        rf.matched_geometry = { { "via", "metadata_replay" } };
        out.push_back(rf);
    }

    const auto cyls = collectCylinders(wp);
    if (cyls.size() < 2) return out;

    // For each candidate central cylinder, look for a coaxial wider cylinder
    // (fire-seal annular wall).
    for (const auto& central : cyls) {
        if (std::abs(central.axis.Direction().Z()) < 0.9) continue;

        for (const auto& outer : cyls) {
            if (outer.faceIdx == central.faceIdx) continue;
            if (std::abs(outer.axis.Direction().Z()) < 0.9) continue;
            const double dx = outer.axis.Location().X() - central.axis.Location().X();
            const double dy = outer.axis.Location().Y() - central.axis.Location().Y();
            if (std::hypot(dx, dy) > 2.0) continue;
            if (outer.radius <= central.radius * 1.2) continue;

            json recovered = {
                { "center_x_mm",           central.axis.Location().X() },
                { "center_y_mm",           central.axis.Location().Y() },
                { "axis_dir",              { 0.0, 0.0, 1.0 } },
                { "penetration_dia_mm",    2.0 * central.radius },
                { "fire_seal_ring_dia_mm", 2.0 * outer.radius },
            };
            json matched = {
                { "central_face_id", central.faceIdx },
                { "seal_face_id",    outer.faceIdx },
                { "via",             "geometric_annular_fire_seal" },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.8, matched });
            break;
        }
    }

    return out;
}

}  // namespace koocadcam::skill::bulkhead_penetration_iso6042
