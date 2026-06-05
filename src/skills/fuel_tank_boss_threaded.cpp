// @lat: [[engine/skills#fuel_tank_boss_threaded]]

#include "fuel_tank_boss_threaded.hpp"

#include "Workpiece.hpp"
#include "_as568_table.hpp"
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

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::fuel_tank_boss_threaded {

namespace pr = koocadcam::engine::prim;
namespace as = koocadcam::skill::as568;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;

// Groove mean diameter for an AS568 ring sealing on the boss top.
double grooveMeanDia(const as::DashSpec& ring)
{
    return ring.inner_dia_mm + ring.cross_section_mm;
}
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.boss_dia_mm <= 0.0 || in.boss_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "fuel_tank_boss_threaded: all dims must be > 0");
        return r;
    }

    const auto* thr = thread_table::findMetric(in.thread_key);
    if (!thr) {
        r.add("DFM-THREAD", "error",
              "fuel_tank_boss_threaded: thread_key '" + in.thread_key +
              "' not in _iso_thread_table");
    }

    const as::DashSpec* ring = as::findDash(in.o_ring_size_key);
    if (!ring) {
        r.add("DFM-ORING", "error",
              "fuel_tank_boss_threaded: o_ring_size_key '" +
              in.o_ring_size_key + "' not in _as568_table");
    }

    // O-ring groove must fit on the boss top: groove OD < boss dia.
    if (ring) {
        const double grooveOd = grooveMeanDia(*ring) + ring->groove_width_mm;
        if (grooveOd >= in.boss_dia_mm) {
            r.add("DFM-FIT", "error",
                  "fuel_tank_boss_threaded: O-ring groove OD (" +
                  std::to_string(grooveOd) +
                  ") does not fit on boss dia (" +
                  std::to_string(in.boss_dia_mm) + ")");
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
        std::string msg = "fuel_tank_boss_threaded DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* thr = thread_table::findMetric(in.thread_key);
    const as::DashSpec* ring = as::findDash(in.o_ring_size_key);
    if (!thr)  throw SkillError("fuel_tank_boss_threaded: thread lookup failed");
    if (!ring) throw SkillError("fuel_tank_boss_threaded: o-ring lookup failed");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double baseZ = zMax;        // top face of stock
    const double cx    = in.center_xy.X();
    const double cy    = in.center_xy.Y();

    // ── 1) Raised boss (FUSE a cylinder onto the top face) ───────────────
    const double bossR = in.boss_dia_mm / 2.0;
    const gp_Pnt bossStart(cx, cy, baseZ);
    const gp_Ax2 bossAx(bossStart, gp::DZ());
    const TopoDS_Shape bossTool = pr::cylinder(bossAx, bossR, in.boss_height_mm);
    TopoDS_Shape current = pr::fuse(wp.shape(), bossTool);

    const double bossTopZ = baseZ + in.boss_height_mm;

    // ── 2) Threaded port bore (tap-pilot drill from the thread table) ────
    const double pilotR = thr->tap_pilot_dia_mm / 2.0;
    // Bore goes down through the boss into the wall (most of the wall).
    const double boreDepth = in.boss_height_mm + (baseZ - zMin) * 0.6;
    const gp_Pnt boreStart(cx, cy, bossTopZ + kOver - boreDepth);
    const gp_Ax2 boreAx(boreStart, gp::DZ());
    current = pr::cut(current, pr::cylinder(boreAx, pilotR, boreDepth));

    // ── 3) AS568 O-ring face groove on the boss top (annular ring) ───────
    const double grooveMean = grooveMeanDia(*ring);
    const double grooveId   = grooveMean - ring->groove_width_mm / 2.0;
    const double grooveOd   = grooveMean + ring->groove_width_mm / 2.0;
    const double grooveDepth = ring->groove_depth_mm;
    const gp_Pnt grStart(cx, cy, bossTopZ - grooveDepth);
    const gp_Ax2 grAx(grStart, gp::DZ());
    const TopoDS_Shape grooveTool = pr::annularRing(
        grAx, grooveOd / 2.0, grooveId / 2.0, grooveDepth + kOver);
    current = pr::cut(current, grooveTool);

    // Analytic NET volume: boss added − pilot bore − groove (net positive).
    const double vBoss = M_PI * bossR * bossR * in.boss_height_mm;
    const double vBore = M_PI * pilotR * pilotR * boreDepth;
    const double vGroove = M_PI *
        ((grooveOd * grooveOd - grooveId * grooveId) / 4.0) * grooveDepth;
    const double volAdded = vBoss - vBore - vGroove;   // net positive by DFM

    json params = {
        { "center_xy",       { in.center_xy.X(),
                               in.center_xy.Y(),
                               in.center_xy.Z() } },
        { "boss_dia_mm",     in.boss_dia_mm },
        { "boss_height_mm",  in.boss_height_mm },
        { "thread_key",      in.thread_key },
        { "o_ring_size_key", in.o_ring_size_key },
    };
    json pattern = {
        { "kind",                     kSkillId },
        { "is_compound",              true },
        { "aerostruct_feature_type",  "fuel_tank_boss_threaded" },
        { "subfeature_count",         3 },
        { "thread_key",               in.thread_key },
        { "o_ring_size_key",          in.o_ring_size_key },
        { "derived_tap_pilot_dia_mm", thr->tap_pilot_dia_mm },
        { "derived_groove_id_mm",     grooveId },
        { "derived_groove_od_mm",     grooveOd },
        { "derived_boss_volume_mm3",  vBoss },
        { "derived_bore_volume_mm3",  vBore },
        { "derived_volume_added_mm3", volAdded },
        { "standard",                 "integral fuel-tank boss (face seal)" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "face_mill;drill;groove_mill";
    tooling.tool_dia_mm       = thr->tap_pilot_dia_mm;
    tooling.tool_length_mm    = boreDepth + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = vBore + vGroove;
    tooling.est_cycle_time_s  = 120.0;
    tooling.extra = {
        { "aerostruct_feature_type", "fuel_tank_boss_threaded" },
        { "added_volume_mm3",        volAdded },
        { "removed_volume_mm3",      vBore + vGroove },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::fuel_tank_boss_threaded: boss_dia={} h={} thread={} oring={}",
                  in.boss_dia_mm, in.boss_height_mm,
                  in.thread_key, in.o_ring_size_key);

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
            { "aerostruct_feature_type", "fuel_tank_boss_threaded" },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: a large boss cylinder face + a central bore +Z cyl.
    int bossCyls = 0;
    int boreCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.Z()) < 0.9) continue;
            const double rad = s.Cylinder().Radius();
            if (rad >= 10.0) ++bossCyls;
            else if (rad <= 8.0) ++boreCyls;
        } catch (...) {}
    }
    if (bossCyls >= 1 && boreCyls >= 1) {
        json recovered = { { "thread_key",      "M12" },
                           { "o_ring_size_key", "-016" } };
        json matched   = { { "source",    "geometric_boss_bore" },
                           { "boss_cyls", bossCyls },
                           { "bore_cyls", boreCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::fuel_tank_boss_threaded
