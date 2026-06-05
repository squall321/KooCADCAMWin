// @lat: [[engine/skills#stringer_run_out_cutout]]

#include "stringer_run_out_cutout.hpp"

#include "Workpiece.hpp"
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

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::stringer_run_out_cutout {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.stringer_width_mm <= 0.0 || in.stringer_height_mm <= 0.0 ||
        in.mouse_hole_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "stringer_run_out_cutout: all dims must be > 0");
        return r;
    }

    const double minHalf =
        std::min(in.stringer_width_mm, in.stringer_height_mm) / 2.0;
    if (in.mouse_hole_radius_mm >= minHalf) {
        r.add("DFM-RADIUS", "error",
              "stringer_run_out_cutout: mouse_hole_radius_mm (" +
              std::to_string(in.mouse_hole_radius_mm) +
              ") must be < min(width,height)/2 (" +
              std::to_string(minHalf) + ")");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "stringer_run_out_cutout DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double thickness = zMax - zMin;
    const double cutDepth  = thickness + 2.0 * kOver;     // full-depth notch

    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();

    // ── 1) Rectangular stringer cutout (box, full web depth) ─────────────
    const gp_Pnt boxOrigin(cx - in.stringer_width_mm / 2.0,
                           cy - in.stringer_height_mm / 2.0,
                           topZ + kOver - cutDepth);
    const TopoDS_Shape boxTool = pr::box(
        gp_Ax2(boxOrigin, gp::DZ()),
        in.stringer_width_mm, in.stringer_height_mm, cutDepth);
    TopoDS_Shape current = pr::cut(wp.shape(), boxTool);

    // ── 2) Radiused mouse hole at the lower-left corner of the cutout ────
    const double mx = cx - in.stringer_width_mm / 2.0;
    const double my = cy - in.stringer_height_mm / 2.0;
    const gp_Pnt mouseStart(mx, my, topZ + kOver - cutDepth);
    const gp_Ax2 mouseAx(mouseStart, gp::DZ());
    current = pr::cut(current,
                      pr::cylinder(mouseAx, in.mouse_hole_radius_mm, cutDepth));

    // Analytic removed volume (box + mouse-hole quarter-cylinder beyond box).
    const double vBox = in.stringer_width_mm * in.stringer_height_mm * thickness;
    const double vMouse = 0.75 * M_PI * in.mouse_hole_radius_mm *
                          in.mouse_hole_radius_mm * thickness;
    const double volRemoved = vBox + vMouse;

    json params = {
        { "center_xy",            { in.center_xy.X(),
                                    in.center_xy.Y(),
                                    in.center_xy.Z() } },
        { "stringer_width_mm",    in.stringer_width_mm },
        { "stringer_height_mm",   in.stringer_height_mm },
        { "mouse_hole_radius_mm", in.mouse_hole_radius_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "aerostruct_feature_type",    "stringer_run_out_cutout" },
        { "subfeature_count",           2 },
        { "derived_cutout_width_mm",    in.stringer_width_mm },
        { "derived_cutout_height_mm",   in.stringer_height_mm },
        { "derived_mouse_hole_dia_mm",  2.0 * in.mouse_hole_radius_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "stringer run-out / mouse-hole relief" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill";
    tooling.tool_dia_mm       = 2.0 * in.mouse_hole_radius_mm;
    tooling.tool_length_mm    = cutDepth + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 45.0;
    tooling.extra = {
        { "aerostruct_feature_type", "stringer_run_out_cutout" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::stringer_run_out_cutout: {}x{} mouse_r={}",
                  in.stringer_width_mm, in.stringer_height_mm,
                  in.mouse_hole_radius_mm);

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
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",                  "metadata_replay" },
            { "is_compound",             true },
            { "aerostruct_feature_type", "stringer_run_out_cutout" },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: a small +Z cylinder (mouse hole) near a cutout.
    int smallCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.Z()) > 0.9 && s.Cylinder().Radius() <= 10.0) ++smallCyls;
        } catch (...) {}
    }
    if (smallCyls >= 1) {
        json recovered = { { "mouse_hole_radius_mm", 5.0 } };
        json matched   = { { "source",     "geometric_mouse_hole" },
                           { "small_cyls", smallCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::stringer_run_out_cutout
