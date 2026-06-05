// @lat: [[engine/skills#axle_box_bearing_housing]]

#include "axle_box_bearing_housing.hpp"

#include "Workpiece.hpp"
#include "_iso286_fits.hpp"
#include "_iso_thread_table.hpp"
#include "_retaining_rings.hpp"
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

#include <algorithm>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::axle_box_bearing_housing {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;
using retaining_rings::Din472Spec;
using retaining_rings::findDin472;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bearing_od_mm <= 0.0 || in.housing_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "axle_box_bearing_housing: all dimensions must be > 0");
    }

    if (in.ring_size_key.empty()) {
        r.add("DFM-RING", "error",
              "axle_box_bearing_housing: ring_size_key is empty");
    } else if (!findDin472(in.ring_size_key)) {
        r.add("DFM-RING", "error",
              "axle_box_bearing_housing: ring_size_key '" + in.ring_size_key +
              "' not present in central DIN 472 table");
    }

    if (in.grease_thread_key.empty()) {
        r.add("DFM-THREAD", "error",
              "axle_box_bearing_housing: grease_thread_key is empty");
    } else if (!tt::findMetric(in.grease_thread_key)) {
        r.add("DFM-THREAD", "error",
              "axle_box_bearing_housing: grease_thread_key '" +
              in.grease_thread_key +
              "' not present in central _iso_thread_table.hpp");
    }

    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "axle_box_bearing_housing DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const Din472Spec* ring = findDin472(in.ring_size_key);
    if (!ring) throw SkillError("axle_box_bearing_housing: DIN 472 lookup failed");
    const tt::MetricThreadSpec* mSpec = tt::findMetric(in.grease_thread_key);
    if (!mSpec) throw SkillError("axle_box_bearing_housing: thread lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx = in.axis_origin.X();
    const double cy = in.axis_origin.Y();

    // ── 1) Press-fit bearing bore (P7 hole basis on the bearing OD) ───
    // p7_max_mm gives the largest allowable bore Ø (still interference).
    const double boreDia = iso286::p7_max_mm(in.bearing_od_mm);
    const double boreR   = boreDia / 2.0;
    const double boreDepth = std::min(in.housing_depth_mm, (zMax - zMin) - 2.0);
    const gp_Pnt boreStart(cx, cy, topZ - boreDepth);
    const gp_Ax2 boreAx(boreStart, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::cylinder(boreAx, boreR, boreDepth + kOver));

    // ── 2) DIN 472 internal retaining-ring groove (annular ring) ──────
    // Groove OD = boreR + groove_depth; inner overlaps the bore wall so
    // the cut only removes the surrounding ring of material.
    const double grooveR = boreR + ring->groove_depth_mm;
    const double grooveZ = topZ - boreDepth + ring->groove_width_mm;  // near bottom
    const gp_Pnt grooveStart(cx, cy, grooveZ);
    const gp_Ax2 grooveAx(grooveStart, gp::DZ());
    const TopoDS_Shape grooveTool = pr::annularRing(
        grooveAx, grooveR, std::max(0.05, boreR - 0.01), ring->groove_width_mm);
    current = pr::cut(current, grooveTool);

    // ── 3) Grease port hole (radial M-thread, non-overlapping) ────────
    // Drilled in -X from the +X face down to (but not through) the bore.
    const double portClr = mSpec->clearance_medium_mm;
    const double portR   = portClr / 2.0;
    const double portZ   = topZ - boreDepth / 2.0;
    const double portDepth = (xMax - cx) - boreR + kOver;   // stop at bore wall
    const gp_Pnt portStart(xMax + kOver, cy, portZ);
    const gp_Ax2 portAx(portStart, gp_Dir(-1.0, 0.0, 0.0));
    current = pr::cut(current, pr::cylinder(portAx, portR, portDepth));

    const TopoDS_Shape newShape = current;

    // ── Derived volume (analytic) ─────────────────────────────────────
    const double vBore   = M_PI * boreR * boreR * boreDepth;
    const double vGroove = M_PI * (grooveR * grooveR - boreR * boreR)
                           * ring->groove_width_mm;
    const double vPort   = M_PI * portR * portR * std::max(0.0, portDepth);
    const double volRemoved = vBore + vGroove + vPort;

    json params = {
        { "axis_origin",       { in.axis_origin.X(),
                                 in.axis_origin.Y(),
                                 in.axis_origin.Z() } },
        { "bearing_od_mm",     in.bearing_od_mm },
        { "housing_depth_mm",  in.housing_depth_mm },
        { "ring_size_key",     in.ring_size_key },
        { "grease_thread_key", in.grease_thread_key },
    };

    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "railway_feature_type",       "axle_box_bearing_housing" },
        { "subfeature_count",           3 },
        { "bearing_od_mm",              in.bearing_od_mm },
        { "ring_size_key",              in.ring_size_key },
        { "grease_thread_key",          in.grease_thread_key },
        { "derived_bore_p7_max_mm",     boreDia },
        { "derived_bore_p7_min_mm",     iso286::p7_min_mm(in.bearing_od_mm) },
        { "derived_groove_dia_mm",      2.0 * grooveR },
        { "derived_groove_width_mm",    ring->groove_width_mm },
        { "derived_groove_depth_mm",    ring->groove_depth_mm },
        { "derived_grease_clearance_mm", portClr },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "ISO 286 P7 press fit + DIN 472" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;groove_insert;drill";
    tooling.tool_dia_mm       = boreDia;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(60.0, boreDepth * 1.5);
    tooling.extra = {
        { "railway_application", "axle_box_bearing_housing" },
        { "ring_size_key",       in.ring_size_key },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::axle_box_bearing_housing: od={} ring={} thread={} faces {}→{}",
                  in.bearing_od_mm, in.ring_size_key, in.grease_thread_key,
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

    // Geometric fallback: a large Z-axis bore plus a larger concentric
    // groove cylinder (= bore + groove walls).
    int boreCyls   = 0;
    int grooveCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder c = s.Cylinder();
            if (std::abs(c.Axis().Direction().Z()) < 0.9) continue;
            const double radius = c.Radius();
            if (radius >= 30.0) ++boreCyls;
            else if (radius > 0.0) ++grooveCyls;
        } catch (...) {}
    }
    if (boreCyls >= 1) {
        json recovered = { { "bearing_od_mm",     130.0 },
                           { "housing_depth_mm",  40.0 },
                           { "ring_size_key",     "100mm" },
                           { "grease_thread_key", "M10" } };
        json matched   = { { "source",      "geometric_axle_box" },
                           { "bore_cyls",   boreCyls },
                           { "groove_cyls", grooveCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::axle_box_bearing_housing
