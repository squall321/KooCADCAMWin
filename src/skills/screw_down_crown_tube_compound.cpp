// @lat: [[engine/skills#screw_down_crown_tube_compound]]

#include "screw_down_crown_tube_compound.hpp"

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

namespace koocadcam::skill::screw_down_crown_tube_compound {

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

    if (in.tube_bore_dia_mm <= 0.0 || in.shoulder_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "screw_down_crown_tube_compound: tube_bore/shoulder dims must be > 0");
        return r;
    }

    const as::DashSpec* ring = as::findDash(in.o_ring_size_key);
    if (!ring) {
        r.add("DFM-AS568", "error",
              "screw_down_crown_tube_compound: o_ring_size_key '" +
              in.o_ring_size_key + "' not in central AS568 table");
    }

    const tt::MetricThreadSpec* thr = tt::findMetric(in.thread_size_key);
    if (!thr) {
        r.add("DFM-THREAD", "error",
              "screw_down_crown_tube_compound: thread_size_key '" +
              in.thread_size_key + "' not in central metric thread table");
    }

    if (in.shoulder_dia_mm <= in.tube_bore_dia_mm) {
        r.add("DFM-SHOULDER", "error",
              "screw_down_crown_tube_compound: shoulder_dia " +
              std::to_string(in.shoulder_dia_mm) +
              " mm must exceed tube_bore_dia " +
              std::to_string(in.tube_bore_dia_mm) + " mm");
    }
    (void)wp;
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "screw_down_crown_tube_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const as::DashSpec* ring = as::findDash(in.o_ring_size_key);
    if (!ring) throw SkillError("screw_down_crown_tube_compound: AS568 lookup failed");
    const tt::MetricThreadSpec* thr = tt::findMetric(in.thread_size_key);
    if (!thr) throw SkillError("screw_down_crown_tube_compound: thread lookup failed");

    const gp_Dir dir = in.bore_dir;
    const gp_Pnt org = in.bore_origin;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double diag = std::sqrt(std::pow(xMax - xMin, 2.0) +
                                  std::pow(yMax - yMin, 2.0) +
                                  std::pow(zMax - zMin, 2.0));
    const double boreDepth = std::max(diag * 0.6, 6.0);
    const double boreR = in.tube_bore_dia_mm / 2.0;

    auto axAt = [&](double inset) -> gp_Ax2 {
        const gp_Pnt p(org.X() + dir.X() * inset,
                       org.Y() + dir.Y() * inset,
                       org.Z() + dir.Z() * inset);
        return gp_Ax2(p, dir);
    };

    // ── 1) Crown tube bore (cylinder from the side) ──────────────────────
    TopoDS_Shape current = pr::cut(
        wp.shape(), pr::cylinder(axAt(-kOver), boreR, boreDepth + kOver));

    // ── 2) External thread relief (annular relief at the mouth) ──────────
    const double reliefR_outer = thr->nominal_dia_mm / 2.0;
    const double reliefR_inner = boreR - 0.02;
    const double reliefDepth   = std::max(thr->pitch_mm * 3.0, 1.5);
    current = pr::cut(current,
        pr::annularRing(axAt(-kOver), reliefR_outer, reliefR_inner,
                        reliefDepth + kOver));

    // ── 3) O-ring groove #1 (annular, just past the thread relief) ───────
    const double grooveR_outer = boreR + ring->groove_depth_mm;
    const double grooveR_inner = boreR - 0.02;
    const double grooveWidth   = ring->groove_width_mm;
    const double groove1Inset  = reliefDepth + ring->cross_section_mm + 0.5;
    current = pr::cut(current,
        pr::annularRing(axAt(groove1Inset), grooveR_outer, grooveR_inner,
                        grooveWidth));

    // ── 4) O-ring groove #2 (annular, deeper into the bore) ──────────────
    const double groove2Inset = groove1Inset + grooveWidth +
                                ring->cross_section_mm + 0.8;
    current = pr::cut(current,
        pr::annularRing(axAt(groove2Inset), grooveR_outer, grooveR_inner,
                        grooveWidth));

    // ── 5) Tube shoulder counterbore (wider mouth for the tube flange) ───
    const double shoulderR_outer = in.shoulder_dia_mm / 2.0;
    const double shoulderR_inner = boreR - 0.02;
    const double shoulderDepth   = std::max(1.0, ring->cross_section_mm);
    current = pr::cut(current,
        pr::annularRing(axAt(-kOver), shoulderR_outer, shoulderR_inner,
                        shoulderDepth + kOver));

    const double vBore = M_PI * boreR * boreR * boreDepth;
    const double vRelief = M_PI *
        (reliefR_outer * reliefR_outer - reliefR_inner * reliefR_inner) * reliefDepth;
    const double vGroove = M_PI *
        (grooveR_outer * grooveR_outer - grooveR_inner * grooveR_inner) * grooveWidth;
    const double vShoulder = M_PI *
        (shoulderR_outer * shoulderR_outer - shoulderR_inner * shoulderR_inner)
        * shoulderDepth;
    const double volRemoved = vBore + vRelief + 2.0 * vGroove + vShoulder;

    json params = {
        { "bore_origin",      { in.bore_origin.X(),
                                in.bore_origin.Y(),
                                in.bore_origin.Z() } },
        { "bore_dir",         { in.bore_dir.X(),
                                in.bore_dir.Y(),
                                in.bore_dir.Z() } },
        { "tube_bore_dia_mm", in.tube_bore_dia_mm },
        { "thread_size_key",  in.thread_size_key },
        { "o_ring_size_key",  in.o_ring_size_key },
        { "shoulder_dia_mm",  in.shoulder_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "watch_feature_type",         "screw_down_crown_tube" },
        { "subfeature_count",           5 },
        { "thread_size_key",            in.thread_size_key },
        { "o_ring_size_key",            in.o_ring_size_key },
        { "derived_bore_dia_mm",        in.tube_bore_dia_mm },
        { "derived_relief_dia_mm",      thr->nominal_dia_mm },
        { "derived_groove_od_mm",       grooveR_outer * 2.0 },
        { "derived_groove_count",       2 },
        { "derived_shoulder_dia_mm",    in.shoulder_dia_mm },
        { "derived_volume_removed_mm3", volRemoved },
        { "axis_dir",                   { dir.X(), dir.Y(), dir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;groove_tool;counterbore";
    tooling.tool_dia_mm       = in.tube_bore_dia_mm;
    tooling.tool_length_mm    = boreDepth + 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 180.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(25.0, volRemoved / 50.0);
    tooling.extra = {
        { "watch_application", "screw_down_crown_tube" },
        { "o_ring_standard",   "AS568" },
        { "thread_standard",   "ISO metric" },
        { "o_ring_count",      2 },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::screw_down_crown_tube_compound: bore={} thread={} oring={} shoulder={}",
                  in.tube_bore_dia_mm, in.thread_size_key,
                  in.o_ring_size_key, in.shoulder_dia_mm);

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

    // Geometric fallback: tube bore + thread relief + shoulder leaves several
    // concentric cylindrical faces of stepped radii on a horizontal axis.
    int boreCyls = 0;
    int wideCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 1.0 && radius <= 2.5) ++boreCyls;
            else if (radius > 2.5 && radius <= 6.0) ++wideCyls;
        } catch (...) {}
    }
    if (boreCyls >= 1 && wideCyls >= 1) {
        json recovered = { { "tube_bore_dia_mm", 3.0 },
                           { "thread_size_key",  "M5" },
                           { "o_ring_size_key",  "-006" },
                           { "shoulder_dia_mm",  6.0 } };
        json matched   = { { "source",     "geometric_tube_stack_pattern" },
                           { "bore_cyls", boreCyls },
                           { "wide_cyls", wideCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::screw_down_crown_tube_compound
