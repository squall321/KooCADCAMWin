// @lat: [[engine/skills#nacelle_bedplate_mount_pad]]

#include "nacelle_bedplate_mount_pad.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
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

#include <array>
#include <cmath>
#include <memory>
#include <utility>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::nacelle_bedplate_mount_pad {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pad_length_mm <= 0.0 || in.pad_width_mm <= 0.0 ||
        in.pad_height_mm <= 0.0 ||
        in.bolt_spacing_x_mm <= 0.0 || in.bolt_spacing_y_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "nacelle_bedplate_mount_pad: all dims must be > 0");
        return r;
    }

    const auto* spec = thread_table::findMetric(in.bolt_thread_key);
    if (!spec) {
        r.add("DFM-THREAD", "error",
              "nacelle_bedplate_mount_pad: bolt_thread_key '" +
              in.bolt_thread_key + "' not in _iso_thread_table");
    }

    // Bolt pattern must fit inside the pad with edge clearance >= bolt dia.
    const double boltDia = spec ? spec->clearance_medium_mm : 0.0;
    const double edgeX = (in.pad_length_mm - in.bolt_spacing_x_mm) / 2.0;
    const double edgeY = (in.pad_width_mm  - in.bolt_spacing_y_mm) / 2.0;
    if (edgeX < boltDia || edgeY < boltDia) {
        r.add("DFM-SPACING", "error",
              "nacelle_bedplate_mount_pad: bolt spacing leaves edge < bolt "
              "dia (edgeX=" + std::to_string(edgeX) +
              " edgeY=" + std::to_string(edgeY) + ")");
    }

    // Net positive: pad volume must exceed 4 bolt holes through pad.
    if (spec) {
        const double rBolt = boltDia / 2.0;
        const double vPad  = in.pad_length_mm * in.pad_width_mm * in.pad_height_mm;
        const double vHoles = 4.0 * M_PI * rBolt * rBolt * in.pad_height_mm;
        if (vPad <= vHoles) {
            r.add("DFM-NET", "error",
                  "nacelle_bedplate_mount_pad: pad volume must exceed bolt "
                  "holes (net positive)");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "nacelle_bedplate_mount_pad DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = thread_table::findMetric(in.bolt_thread_key);
    // spec guaranteed non-null by validate.
    const double boltDia = spec->clearance_medium_mm;
    const double boltR   = boltDia / 2.0;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double baseZ = zMax;            // top face of stock
    const double cx    = in.center_xy.X();
    const double cy    = in.center_xy.Y();

    // ── 1) Raised mounting pad (FUSE a boss box onto the top face) ───────
    const gp_Pnt padOrigin(
        cx - in.pad_length_mm / 2.0,
        cy - in.pad_width_mm  / 2.0,
        baseZ);
    const TopoDS_Shape padTool = pr::box(
        gp_Ax2(padOrigin, gp::DZ()),
        in.pad_length_mm, in.pad_width_mm, in.pad_height_mm);
    TopoDS_Shape current = pr::fuse(wp.shape(), padTool);

    // ── 2..5) 4 bolt holes drilled THROUGH the raised pad ────────────────
    const double padTopZ = baseZ + in.pad_height_mm;
    const double holeDepth = in.pad_height_mm + 2.0 * kOver;
    const std::array<std::pair<double, double>, 4> boltPositions {{
        { cx - in.bolt_spacing_x_mm / 2.0, cy - in.bolt_spacing_y_mm / 2.0 },
        { cx + in.bolt_spacing_x_mm / 2.0, cy - in.bolt_spacing_y_mm / 2.0 },
        { cx - in.bolt_spacing_x_mm / 2.0, cy + in.bolt_spacing_y_mm / 2.0 },
        { cx + in.bolt_spacing_x_mm / 2.0, cy + in.bolt_spacing_y_mm / 2.0 },
    }};
    for (const auto& [bx, by] : boltPositions) {
        const gp_Pnt start(bx, by, padTopZ + kOver - holeDepth);
        const gp_Ax2 ax(start, gp::DZ());
        const TopoDS_Shape boltTool = pr::cylinder(ax, boltR, holeDepth);
        current = pr::cut(current, boltTool);   // sequential — no compound
    }

    // ── Derived volume (analytic; net = pad − 4 holes) ───────────────────
    const double vPad   = in.pad_length_mm * in.pad_width_mm * in.pad_height_mm;
    const double vHoles = 4.0 * M_PI * boltR * boltR * in.pad_height_mm;
    const double volAdded = vPad - vHoles;   // net positive

    json bolts_json = json::array();
    for (const auto& [bx, by] : boltPositions)
        bolts_json.push_back({ { "x_mm", bx }, { "y_mm", by } });

    json params = {
        { "center_xy",        { in.center_xy.X(),
                                in.center_xy.Y(),
                                in.center_xy.Z() } },
        { "pad_length_mm",    in.pad_length_mm },
        { "pad_width_mm",     in.pad_width_mm },
        { "pad_height_mm",    in.pad_height_mm },
        { "bolt_spacing_x_mm",in.bolt_spacing_x_mm },
        { "bolt_spacing_y_mm",in.bolt_spacing_y_mm },
        { "bolt_thread_key",  in.bolt_thread_key },
    };
    json pattern = {
        { "kind",                     kSkillId },
        { "is_compound",              true },
        { "wind_feature_type",        "nacelle_bedplate_mount_pad" },
        { "subfeature_count",         1 + 4 },
        { "bolt_count",               4 },
        { "pad_length_mm",            in.pad_length_mm },
        { "pad_width_mm",             in.pad_width_mm },
        { "pad_height_mm",            in.pad_height_mm },
        { "bolt_thread_key",          in.bolt_thread_key },
        { "bolt_positions",           bolts_json },
        { "derived_bolt_hole_dia_mm", boltDia },
        { "derived_pad_volume_mm3",   vPad },
        { "derived_hole_volume_mm3",  vHoles },
        { "derived_volume_added_mm3", volAdded },
        { "standard",                 "IEC 61400-1 drivetrain mount" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "face_mill;drill";
    tooling.tool_dia_mm       = boltDia;
    tooling.tool_length_mm    = holeDepth + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.06;
    tooling.stock_removed_mm3 = vHoles;
    tooling.est_cycle_time_s  = 120.0;
    tooling.extra = {
        { "wind_feature_type", "nacelle_bedplate_mount_pad" },
        { "added_volume_mm3",  volAdded },
        { "removed_volume_mm3",vHoles },
        { "mount_family",      "drivetrain_pad" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::nacelle_bedplate_mount_pad: pad {}×{}×{} thread={}",
                  in.pad_length_mm, in.pad_width_mm,
                  in.pad_height_mm, in.bolt_thread_key);

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
            { "source",            "metadata_replay" },
            { "is_compound",       true },
            { "wind_feature_type", "nacelle_bedplate_mount_pad" },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: 4 equal-radius +Z cylinders (bolt holes in pad).
    int boltCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.Z()) > 0.9) ++boltCyls;
        } catch (...) {}
    }
    if (boltCyls >= 4) {
        json recovered = { { "bolt_thread_key", "M20" } };
        json matched   = { { "source",    "geometric_pad_4bolt" },
                           { "bolt_cyls", boltCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::nacelle_bedplate_mount_pad
