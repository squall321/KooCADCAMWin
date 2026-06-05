// @lat: [[engine/skills#chronograph_pusher_tube_bore]]

#include "chronograph_pusher_tube_bore.hpp"

#include "Workpiece.hpp"
#include "_as568_table.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
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

namespace koocadcam::skill::chronograph_pusher_tube_bore {

namespace pr = koocadcam::engine::prim;
namespace as = koocadcam::skill::as568;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.pusher_bore_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "chronograph_pusher_tube_bore: pusher_bore_dia_mm must be > 0");
        return r;
    }

    const as::DashSpec* ring = as::findDash(in.o_ring_size_key);
    if (!ring) {
        r.add("DFM-AS568", "error",
              "chronograph_pusher_tube_bore: o_ring_size_key '" +
              in.o_ring_size_key + "' not in central AS568 table");
    }

    const tt::MetricThreadSpec* thr = tt::findMetric(in.thread_size_key);
    if (!thr) {
        r.add("DFM-THREAD", "error",
              "chronograph_pusher_tube_bore: thread_size_key '" +
              in.thread_size_key + "' not in central metric thread table");
    }

    // Retaining-ring relief OD = thread nominal; must clear pusher bore.
    if (thr && thr->nominal_dia_mm <= in.pusher_bore_dia_mm) {
        r.add("DFM-CLEARANCE", "error",
              "chronograph_pusher_tube_bore: thread nominal " +
              std::to_string(thr->nominal_dia_mm) +
              " mm must exceed pusher_bore_dia " +
              std::to_string(in.pusher_bore_dia_mm) + " mm");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "chronograph_pusher_tube_bore DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const as::DashSpec* ring = as::findDash(in.o_ring_size_key);
    if (!ring) throw SkillError("chronograph_pusher_tube_bore: AS568 lookup failed");
    const tt::MetricThreadSpec* thr = tt::findMetric(in.thread_size_key);
    if (!thr) throw SkillError("chronograph_pusher_tube_bore: thread lookup failed");

    // Inward bore direction (normalize) and starting point pulled OUT of the
    // side face by an overhang so the cut breaks the surface cleanly.
    const gp_Dir dir = in.bore_dir;
    const gp_Pnt org = in.bore_origin;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double diag = std::sqrt(std::pow(xMax - xMin, 2.0) +
                                  std::pow(yMax - yMin, 2.0) +
                                  std::pow(zMax - zMin, 2.0));
    const double boreDepth = std::max(diag * 0.6, 6.0);

    const double boreR = in.pusher_bore_dia_mm / 2.0;

    // ── 1) Pusher tube bore (cylinder cut along bore_dir) ────────────────
    const gp_Pnt boreStart(org.X() - dir.X() * kOver,
                           org.Y() - dir.Y() * kOver,
                           org.Z() - dir.Z() * kOver);
    const gp_Ax2 boreAx(boreStart, dir);
    TopoDS_Shape current = pr::cut(
        wp.shape(), pr::cylinder(boreAx, boreR, boreDepth + kOver));

    // ── 2) AS568 O-ring gasket groove (annular ring, coaxial) ────────────
    // Groove sits a little inside the mouth; OD = bore + groove depth on
    // each side, ID = bore radius (biased) so we carve an annular gland.
    const double grooveR_outer = boreR + ring->groove_depth_mm;
    const double grooveR_inner = boreR - 0.02;   // bias to avoid coplanar
    const double grooveWidth   = ring->groove_width_mm;
    const double grooveInset   = ring->cross_section_mm + 0.5;   // from mouth
    const gp_Pnt grooveStart(org.X() + dir.X() * grooveInset,
                             org.Y() + dir.Y() * grooveInset,
                             org.Z() + dir.Z() * grooveInset);
    const gp_Ax2 grooveAx(grooveStart, dir);
    const TopoDS_Shape grooveTool = pr::annularRing(
        grooveAx, grooveR_outer, grooveR_inner, grooveWidth);
    current = pr::cut(current, grooveTool);

    // ── 3) Threaded retaining-ring relief (annular relief at mouth) ──────
    // The threaded tube collar lands in a counterbore-relief of thread
    // nominal dia, a couple of pitches deep at the surface.
    const double reliefR_outer = thr->nominal_dia_mm / 2.0;
    const double reliefR_inner = boreR - 0.02;
    const double reliefDepth   = std::max(thr->pitch_mm * 2.0, 1.0);
    const gp_Pnt reliefStart(org.X() - dir.X() * kOver,
                             org.Y() - dir.Y() * kOver,
                             org.Z() - dir.Z() * kOver);
    const gp_Ax2 reliefAx(reliefStart, dir);
    const TopoDS_Shape reliefTool = pr::annularRing(
        reliefAx, reliefR_outer, reliefR_inner, reliefDepth + kOver);
    current = pr::cut(current, reliefTool);

    const double vBore = M_PI * boreR * boreR * boreDepth;
    const double vGroove = M_PI *
        (grooveR_outer * grooveR_outer - grooveR_inner * grooveR_inner) * grooveWidth;
    const double vRelief = M_PI *
        (reliefR_outer * reliefR_outer - reliefR_inner * reliefR_inner) * reliefDepth;
    const double volRemoved = vBore + vGroove + vRelief;

    json params = {
        { "bore_origin",        { in.bore_origin.X(),
                                  in.bore_origin.Y(),
                                  in.bore_origin.Z() } },
        { "bore_dir",           { in.bore_dir.X(),
                                  in.bore_dir.Y(),
                                  in.bore_dir.Z() } },
        { "pusher_bore_dia_mm", in.pusher_bore_dia_mm },
        { "o_ring_size_key",    in.o_ring_size_key },
        { "thread_size_key",    in.thread_size_key },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "watch_feature_type",         "chronograph_pusher_tube_bore" },
        { "subfeature_count",           3 },
        { "o_ring_size_key",            in.o_ring_size_key },
        { "thread_size_key",            in.thread_size_key },
        { "derived_bore_dia_mm",        in.pusher_bore_dia_mm },
        { "derived_groove_od_mm",       grooveR_outer * 2.0 },
        { "derived_groove_width_mm",    grooveWidth },
        { "derived_relief_dia_mm",      thr->nominal_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "axis_dir",                   { dir.X(), dir.Y(), dir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;groove_tool;counterbore";
    tooling.tool_dia_mm       = in.pusher_bore_dia_mm;
    tooling.tool_length_mm    = boreDepth + 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 180.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(20.0, volRemoved / 50.0);
    tooling.extra = {
        { "watch_application", "chronograph_pusher" },
        { "o_ring_standard",   "AS568" },
        { "thread_standard",   "ISO metric" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::chronograph_pusher_tube_bore: bore={} oring={} thread={}",
                  in.pusher_bore_dia_mm, in.o_ring_size_key, in.thread_size_key);

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

    // Geometric fallback: a small bore cyl plus a wider relief cyl sharing
    // a horizontal (non-Z) axis indicates a pusher-tube bore family.
    int smallCyls = 0;
    int wideCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 0.8 && radius <= 3.0) ++smallCyls;
            else if (radius > 3.0 && radius <= 8.0) ++wideCyls;
        } catch (...) {}
    }
    if (smallCyls >= 1 && wideCyls >= 1) {
        json recovered = { { "pusher_bore_dia_mm", 2.5 },
                           { "o_ring_size_key",    "-006" },
                           { "thread_size_key",    "M4" } };
        json matched   = { { "source",     "geometric_bore_relief_pattern" },
                           { "small_cyls", smallCyls },
                           { "wide_cyls",  wideCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::chronograph_pusher_tube_bore
