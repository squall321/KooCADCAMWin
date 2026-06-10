// @lat: [[engine/skills#mirror_pattern]]

#include "mirror_pattern.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::mirror_pattern {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.hole_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "mirror_pattern: hole_dia_mm must be > 0");
    }
    if (in.hole_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "mirror_pattern: hole_depth_mm must be > 0");
    }
    if (in.hole_dia_mm > 0.0 && in.hole_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "mirror_pattern: hole_dia_mm " + std::to_string(in.hole_dia_mm) +
              " < 0.8 mm (ISO 235:2016)");
    }
    if (in.mirror_axis != "x" && in.mirror_axis != "y") {
        r.add("DFM-INPUT", "error",
              "mirror_pattern: mirror_axis must be 'x' or 'y' (got '" +
              in.mirror_axis + "')");
    }
    // DFM-PATT-3 — mirror distance must exceed hole dia.
    double mirrorDist = 0.0;
    if (in.mirror_axis == "x") mirrorDist = 2.0 * std::abs(in.hole_x_mm);
    else                        mirrorDist = 2.0 * std::abs(in.hole_y_mm);
    if (mirrorDist <= in.hole_dia_mm) {
        r.add("DFM-PATT-3", "error",
              "mirror_pattern: mirror distance " + std::to_string(mirrorDist) +
              " <= hole_dia_mm " + std::to_string(in.hole_dia_mm) +
              " (original and mirror would overlap)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "mirror_pattern DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceIdOpt = wp.resolve(in.face);
    if (!faceIdOpt) throw SkillError("mirror_pattern: face datum unresolved");
    const int faceId = *faceIdOpt;
    const gp_Pnt center = wp.faceCenter(faceId);
    const gp_Dir normal = wp.faceNormal(faceId);
    const gp_Dir cutDir(-normal.X(), -normal.Y(), -normal.Z());

    constexpr double kOverhang = 0.1;
    const double toolLen = in.hole_depth_mm + 2.0 * kOverhang;
    const double r = in.hole_dia_mm * 0.5;

    // Original + mirrored XY.
    const double mx = (in.mirror_axis == "x") ? -in.hole_x_mm : in.hole_x_mm;
    const double my = (in.mirror_axis == "y") ? -in.hole_y_mm : in.hole_y_mm;

    auto cylAt = [&](double dx, double dy) {
        const gp_Pnt p(
            center.X() + dx + normal.X() * kOverhang,
            center.Y() + dy + normal.Y() * kOverhang,
            center.Z() +       normal.Z() * kOverhang);
        const gp_Ax2 ax(p, cutDir);
        return pr::cylinder(ax, r, toolLen);
    };

    std::vector<TopoDS_Shape> cutters{
        cylAt(in.hole_x_mm, in.hole_y_mm),
        cylAt(mx, my),
    };
    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    const double perVol = M_PI * r * r * in.hole_depth_mm;
    const double totalVol = 2.0 * perVol;

    json params = {
        { "hole_x_mm",     in.hole_x_mm },
        { "hole_y_mm",     in.hole_y_mm },
        { "hole_dia_mm",   in.hole_dia_mm },
        { "hole_depth_mm", in.hole_depth_mm },
        { "face_id",       faceId },
        { "mirror_axis",   in.mirror_axis },
    };
    json pattern = {
        { "kind",           kSkillId },
        { "is_pattern",     true },
        { "instance_count", 2 },
        { "derived_params", {
            { "hole_dia_mm",       in.hole_dia_mm },
            { "hole_depth_mm",     in.hole_depth_mm },
            { "mirror_axis",       in.mirror_axis },
            { "original_xy",       { in.hole_x_mm, in.hole_y_mm } },
            { "mirror_xy",         { mx, my } },
            { "per_inst_vol_mm3",  perVol },
            { "total_vol_mm3",     totalVol },
        } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill";
    tooling.tool_dia_mm       = in.hole_dia_mm;
    tooling.tool_length_mm    = in.hole_depth_mm * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = totalVol;
    tooling.est_cycle_time_s  = std::max(1.0, in.hole_depth_mm / 50.0) * 2.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::mirror_pattern applied: axis={} dia={} vol={}",
                  in.mirror_axis, in.hole_dia_mm, totalVol);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Heuristic: find any pair of cylindrical faces of equal radius whose XY
// centers are mirror-symmetric about the X=0 or Y=0 plane (within 1e-2).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 0.95;
        r.matched_geometry = f.pattern;
        out.push_back(r);
    }
    if (!out.empty()) return out;

    struct C { double x, y, r; };
    std::vector<C> cs;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder gc = s.Cylinder();
            cs.push_back(C{ gc.Location().X(), gc.Location().Y(), gc.Radius() });
        } catch (...) {}
    }
    if (cs.size() < 2) return out;

    for (size_t a = 0; a < cs.size(); ++a) {
        for (size_t b = a + 1; b < cs.size(); ++b) {
            if (std::abs(cs[a].r - cs[b].r) > 1e-2) continue;
            // Mirror about X=0?
            if (std::abs(cs[a].x + cs[b].x) < 1e-2 &&
                std::abs(cs[a].y - cs[b].y) < 1e-2 &&
                std::abs(cs[a].x) > 1e-2) {
                json recovered = {
                    { "hole_x_mm",     std::abs(cs[a].x) },
                    { "hole_y_mm",     cs[a].y },
                    { "hole_dia_mm",   2.0 * cs[a].r },
                    { "hole_depth_mm", 0.0 },
                    { "face_id",       -1 },
                    { "mirror_axis",   "x" },
                };
                out.push_back(RecognizedFeature{
                    kSkillId, recovered, 0.7,
                    json{ { "match", "mirror_x" } } });
                return out;
            }
            // Mirror about Y=0?
            if (std::abs(cs[a].y + cs[b].y) < 1e-2 &&
                std::abs(cs[a].x - cs[b].x) < 1e-2 &&
                std::abs(cs[a].y) > 1e-2) {
                json recovered = {
                    { "hole_x_mm",     cs[a].x },
                    { "hole_y_mm",     std::abs(cs[a].y) },
                    { "hole_dia_mm",   2.0 * cs[a].r },
                    { "hole_depth_mm", 0.0 },
                    { "face_id",       -1 },
                    { "mirror_axis",   "y" },
                };
                out.push_back(RecognizedFeature{
                    kSkillId, recovered, 0.7,
                    json{ { "match", "mirror_y" } } });
                return out;
            }
        }
    }
    return out;
}

}  // namespace koocadcam::skill::mirror_pattern
