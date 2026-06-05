// @lat: [[engine/skills#linear_rail_carriage_pocket]]

#include "linear_rail_carriage_pocket.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::linear_rail_carriage_pocket {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.rail_width_mm <= 0.0 || in.groove_width_mm <= 0.0 ||
        in.groove_depth_mm <= 0.0 || in.mount_pitch_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "linear_rail_carriage_pocket: all dimensions must be > 0");
    }

    if (in.mount_hole_count < 2 || in.mount_hole_count > 8) {
        r.add("DFM-ROBOTICS-COUNT", "error",
              "linear_rail_carriage_pocket: mount_hole_count (" +
              std::to_string(in.mount_hole_count) +
              ") must be in [2, 8]");
    }

    if (!tt::findMetric(in.mount_thread_key)) {
        r.add("DFM-THREAD", "error",
              "linear_rail_carriage_pocket: mount_thread_key '" +
              in.mount_thread_key + "' not in central metric thread table");
    }

    // Two grooves are placed symmetric about the rail center line; they must
    // sit within the rail footprint with a central land between them.
    if (2.0 * in.groove_width_mm >= in.rail_width_mm) {
        r.add("DFM-ROBOTICS-RAIL", "error",
              "linear_rail_carriage_pocket: 2*groove_width_mm must be less "
              "than rail_width_mm (grooves would merge / overrun the rail)");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "linear_rail_carriage_pocket DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const tt::MetricThreadSpec* thread = tt::findMetric(in.mount_thread_key);
    if (!thread) throw SkillError("linear_rail_carriage_pocket: thread lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.center_xy.X();
    const double cy = in.center_xy.Y();
    const double thru = (zMax - zMin) + 2.0 * kOver;

    // Grooves run the full carriage length along X (the rail travel axis).
    const double grooveLen = (xMax - xMin);  // span the block in X
    const double grooveOffsetY = (in.rail_width_mm - in.groove_width_mm) / 2.0;

    // ── 1) Ball-groove channel A (+Y side) ─────────────────────────────
    TopoDS_Shape current = wp.shape();
    {
        const gp_Pnt originA(xMin - kOver,
                             cy + grooveOffsetY - in.groove_width_mm / 2.0,
                             topZ - in.groove_depth_mm);
        const gp_Ax2 axA(originA, gp::DZ());
        current = pr::cut(current,
            pr::box(axA, grooveLen + 2.0 * kOver,
                    in.groove_width_mm, in.groove_depth_mm + kOver));
    }

    // ── 2) Ball-groove channel B (-Y side) ─────────────────────────────
    {
        const gp_Pnt originB(xMin - kOver,
                             cy - grooveOffsetY - in.groove_width_mm / 2.0,
                             topZ - in.groove_depth_mm);
        const gp_Ax2 axB(originB, gp::DZ());
        current = pr::cut(current,
            pr::box(axB, grooveLen + 2.0 * kOver,
                    in.groove_width_mm, in.groove_depth_mm + kOver));
    }

    // ── 3..N+2) N mounting through-holes along the rail axis (X), on the
    //            carriage centre line, between the two grooves. ──────────
    const double holeR = thread->tap_pilot_dia_mm / 2.0;
    const double rowSpan = in.mount_pitch_mm *
                           static_cast<double>(in.mount_hole_count - 1);
    const double rowStartX = cx - rowSpan / 2.0;
    json holes_json = json::array();
    for (int i = 0; i < in.mount_hole_count; ++i) {
        const double hx = rowStartX + in.mount_pitch_mm * static_cast<double>(i);
        const gp_Ax2 holeAx(gp_Pnt(hx, cy, zMin - kOver), gp::DZ());
        current = pr::cut(current, pr::cylinder(holeAx, holeR, thru));  // sequential
        holes_json.push_back({ { "x_mm", hx }, { "y_mm", cy } });
    }

    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ──────────────────────────────────────
    const double plateThk = (zMax - zMin);
    const double vGrooves = 2.0 * grooveLen * in.groove_width_mm * in.groove_depth_mm;
    const double vHoles   = static_cast<double>(in.mount_hole_count) *
                            M_PI * holeR * holeR * plateThk;
    const double volRemoved = vGrooves + vHoles;

    const int subfeatureCount = 2 + in.mount_hole_count;

    json params = {
        { "center_xy",         { in.center_xy.X(),
                                 in.center_xy.Y(),
                                 in.center_xy.Z() } },
        { "rail_width_mm",     in.rail_width_mm },
        { "groove_width_mm",   in.groove_width_mm },
        { "groove_depth_mm",   in.groove_depth_mm },
        { "mount_hole_count",  in.mount_hole_count },
        { "mount_thread_key",  in.mount_thread_key },
        { "mount_pitch_mm",    in.mount_pitch_mm },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "robotics_feature_type",      "linear_guide_carriage_underside" },
        { "subfeature_count",           subfeatureCount },
        { "rail_width_mm",              in.rail_width_mm },
        { "groove_width_mm",            in.groove_width_mm },
        { "mount_hole_count",           in.mount_hole_count },
        { "mount_thread_key",           in.mount_thread_key },
        { "hole_positions",             holes_json },
        { "derived_tap_pilot_dia_mm",   thread->tap_pilot_dia_mm },
        { "derived_groove_offset_mm",   grooveOffsetY },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "profiled linear guide carriage practice" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill;drill;tap";
    tooling.tool_dia_mm       = std::max(in.groove_width_mm, thread->nominal_dia_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 40.0 + 6.0 * static_cast<double>(in.mount_hole_count);
    tooling.extra = {
        { "robotics_application", "linear_guide_carriage" },
        { "removed_volume_mm3",   volRemoved },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::linear_rail_carriage_pocket: rail={} grooves=2 holes={} faces {}→{}",
                  in.rail_width_mm, in.mount_hole_count,
                  wp.faceCount(), wpNew->faceCount());

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

    // Geometric fallback: several small +Z mounting cylinders (the grooves
    // are planar box cuts, not cylinders).
    int holeCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            if (std::abs(c.Axis().Direction().Z()) < 0.9) continue;
            const double radius = c.Radius();
            if (radius >= 1.0 && radius < 6.0) ++holeCyls;
        } catch (...) {}
    }
    if (holeCyls >= 2) {
        json recovered = { { "mount_hole_count", static_cast<int>(holeCyls) },
                           { "mount_thread_key", "M4" } };
        json matched   = { { "source",    "geometric_carriage_holes" },
                           { "hole_cyls", holeCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::linear_rail_carriage_pocket
