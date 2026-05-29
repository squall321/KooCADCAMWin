// @lat: [[engine/skills#pcb_standoff_threaded]]

#include "pcb_standoff_threaded.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::pcb_standoff_threaded {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── ISO 68-1 metric coarse pilot table (mirrors drill_and_tap) ──────────
namespace {

struct ThreadPilot {
    const char* size;
    double      pilot_dia_mm;
    double      hex_flat_mm;   // matching captive-nut wrench size (M3 → 5.5, etc.)
};

constexpr std::array<ThreadPilot, 7> kThreadTable {{
    { "M1.6", 1.25, 3.2 },
    { "M2",   1.60, 4.0 },
    { "M2.5", 2.05, 5.0 },
    { "M3",   2.50, 5.5 },
    { "M4",   3.30, 7.0 },
    { "M5",   4.20, 8.0 },
    { "M6",   5.00, 10.0 },
}};

constexpr double kPCBClearanceAdd_mm   = 1.0;  // recess Ø = boss + 1
constexpr double kPCBClearanceDepth_mm = 0.5;
constexpr double kMinBossWall_mm       = 1.0;  // wall around tap
constexpr double kMaxBossAR            = 5.0;  // height / dia limit

}  // namespace

double pilotDiameterFor(const std::string& thread_M)
{
    for (const auto& e : kThreadTable)
        if (thread_M == e.size) return e.pilot_dia_mm;
    return 0.0;
}

static double hexFlatFor(const std::string& thread_M)
{
    for (const auto& e : kThreadTable)
        if (thread_M == e.size) return e.hex_flat_mm;
    return 0.0;
}

static double resolvedBossDia(const Input& in)
{
    if (in.boss_dia_mm > 0.0) return in.boss_dia_mm;
    const double pilot = pilotDiameterFor(in.thread_M);
    return pilot + 2.0 * (kMinBossWall_mm + 1.0);  // pilot + 4 mm default
}

static double resolvedThreadDepth(const Input& in)
{
    if (in.thread_depth_mm > 0.0) return in.thread_depth_mm;
    return std::max(0.5, in.boss_height_mm - 0.5);
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.boss_height_mm < 1.5) {
        r.add("DFM-INPUT", "error",
              "pcb_standoff_threaded: boss_height_mm " +
              std::to_string(in.boss_height_mm) + " < 1.5 mm");
    }
    if (in.boss_height_mm > 25.0) {
        r.add("DFM-INPUT", "error",
              "pcb_standoff_threaded: boss_height_mm " +
              std::to_string(in.boss_height_mm) + " > 25 mm");
    }

    const double pilot = pilotDiameterFor(in.thread_M);
    if (pilot <= 0.0) {
        r.add("DFM-TAP-SIZE", "error",
              "pcb_standoff_threaded: unknown thread_M '" + in.thread_M +
              "' (supported: M1.6/M2/M2.5/M3/M4/M5/M6)");
        return r;
    }
    if (pilot < 0.8) {
        r.add("DFM-002", "error",
              "pcb_standoff_threaded: pilot dia < 0.8 mm");
    }

    const double bossDia = resolvedBossDia(in);
    if (bossDia < pilot + 2.0 * kMinBossWall_mm) {
        r.add("DFM-BOSS-WALL", "error",
              "pcb_standoff_threaded: boss_dia " +
              std::to_string(bossDia) + " mm gives wall " +
              std::to_string((bossDia - pilot) / 2.0) +
              " mm < min " + std::to_string(kMinBossWall_mm) + " mm");
    }

    if (in.boss_height_mm > 0.0 && bossDia > 0.0 &&
        in.boss_height_mm / bossDia > kMaxBossAR) {
        r.add("DFM-BOSS-AR", "warning",
              "pcb_standoff_threaded: boss aspect ratio " +
              std::to_string(in.boss_height_mm / bossDia) +
              " > 5 — tall boss may warp on thermal cycling");
    }

    const double threadDepth = resolvedThreadDepth(in);
    if (threadDepth > in.boss_height_mm + 1e-6) {
        r.add("DFM-INPUT", "error",
              "pcb_standoff_threaded: thread_depth_mm > boss_height_mm");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// This compound skill includes a FUSE (boss protrusion) plus CUTS (recess +
// tapped hole), so it doesn't fit the pure-cut pipeline.  Sequence:
//   step 1: fuse  boss collar onto the face (adds material)
//   step 2: cut   PCB-clearance recess (annular shallow ring at boss top)
//   step 3: cut   tapped blind hole (cylinder of pilot dia, into boss top)

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pcb_standoff_threaded DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.face);
    if (!entryId) throw SkillError("pcb_standoff_threaded: face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double pilotDia = pilotDiameterFor(in.thread_M);
    const double bossDia  = resolvedBossDia(in);
    const double threadDepth = resolvedThreadDepth(in);

    constexpr double kOverhang = 0.05;
    const double topZ = zMax;

    // Step 1: boss collar protrusion — cylinder above top face.
    const gp_Pnt bossOrigin(in.position_x_mm, in.position_y_mm, topZ);
    const gp_Ax2 bossAx(bossOrigin, gp_Dir(0.0, 0.0, 1.0));
    const TopoDS_Shape bossCyl = pr::cylinder(bossAx, bossDia / 2.0, in.boss_height_mm);

    const TopoDS_Shape afterFuse = pr::fuse(wp.shape(), bossCyl);

    // Step 2: PCB clearance recess — shallow annular pocket on top of boss.
    //         Outer Ø = boss + 1, inner Ø = pilot, depth = 0.5 mm.
    const double bossTopZ = topZ + in.boss_height_mm;
    const gp_Pnt recessOrigin(in.position_x_mm, in.position_y_mm,
                              bossTopZ - kPCBClearanceDepth_mm);
    const gp_Ax2 recessAx(recessOrigin, gp_Dir(0.0, 0.0, 1.0));
    const TopoDS_Shape recessOuter = pr::cylinder(recessAx,
        (bossDia + kPCBClearanceAdd_mm) / 2.0,
        kPCBClearanceDepth_mm + kOverhang);

    // Step 3: tapped blind hole — pilot Ø, threadDepth deep into boss.
    const gp_Pnt tapStart(in.position_x_mm, in.position_y_mm,
                          bossTopZ + kOverhang);
    const gp_Ax2 tapAx(tapStart, gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape tapTool = pr::cylinder(tapAx, pilotDia / 2.0,
                                              threadDepth + kOverhang);

    // The recess and tap hole are coaxial overlapping cutters; fuse them.
    const TopoDS_Shape fusedCutter = pr::fuse(recessOuter, tapTool);
    const TopoDS_Shape afterCuts = pr::cut(afterFuse, fusedCutter);

    // ── Signature ───────────────────────────────────────────────────────
    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "thread_M",       in.thread_M },
        { "boss_height_mm", in.boss_height_mm },
        { "boss_dia_mm",    bossDia },
        { "thread_depth_mm", threadDepth },
    };

    json pattern = {
        { "kind",                   kSkillId },
        { "is_compound",            true },
        { "subfeature_count",       3 },
        { "thread_M",               in.thread_M },
        { "pilot_dia_mm",           pilotDia },
        { "boss_height_mm",         in.boss_height_mm },
        { "boss_dia_mm",            bossDia },
        { "thread_depth_mm",        threadDepth },
        { "pcb_clearance_dia_mm",   bossDia + kPCBClearanceAdd_mm },
        { "pcb_clearance_depth_mm", kPCBClearanceDepth_mm },
        { "captive_nut_hex_mm",     hexFlatFor(in.thread_M) },
        { "axis_dir",               { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "subfeatures",            {
            { { "kind", "boss_protrusion" },
              { "dia_mm",    bossDia },
              { "height_mm", in.boss_height_mm } },
            { { "kind", "pcb_clearance_recess" },
              { "dia_mm",   bossDia + kPCBClearanceAdd_mm },
              { "depth_mm", kPCBClearanceDepth_mm } },
            { { "kind", "tapped_blind_hole" },
              { "thread_size",  in.thread_M },
              { "pilot_dia_mm", pilotDia },
              { "depth_mm",     threadDepth } },
        } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "end_mill;drill;tap";
    tooling.tool_dia_mm      = bossDia;
    tooling.tool_length_mm   = std::max(in.boss_height_mm, threadDepth) * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double tapVol = M_PI * (pilotDia / 2.0) * (pilotDia / 2.0) * threadDepth;
    const double recessVol = M_PI *
        (std::pow((bossDia + kPCBClearanceAdd_mm) / 2.0, 2) -
         std::pow(pilotDia / 2.0, 2)) * kPCBClearanceDepth_mm;
    // boss adds material (positive), tap + recess remove material.  Use
    // remove volume only for "stock_removed" (net add is not stock removal).
    tooling.stock_removed_mm3 = tapVol + recessVol;
    tooling.est_cycle_time_s  = std::max(3.0, in.boss_height_mm + threadDepth);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "boss_extrusion" },
              { "tool_dia_mm", bossDia },
              { "height_mm",   in.boss_height_mm },
              { "note", "additive — boss protrusion (cast / weld / extrude)" } },
            { { "tool_type", "end_mill" },
              { "tool_dia_mm", bossDia + kPCBClearanceAdd_mm },
              { "depth_mm",    kPCBClearanceDepth_mm },
              { "note", "PCB clearance recess" } },
            { { "tool_type", "drill" },
              { "tool_dia_mm", pilotDia },
              { "depth_mm",    threadDepth },
              { "note", "pilot for tap" } },
            { { "tool_type", "tap" },
              { "thread_size", in.thread_M },
              { "depth_mm",    threadDepth },
              { "note", "metric coarse pitch — thread cut as metadata" } },
        } },
        { "standard", "ISO 68-1 metric coarse" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(afterCuts, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::pcb_standoff_threaded applied: M={} boss_dia={} boss_h={} faces {}→{}",
                  in.thread_M, bossDia, in.boss_height_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        const auto& p = f.pattern;
        json recovered = {
            { "position_x_mm",   f.params.value("position_x_mm",   0.0) },
            { "position_y_mm",   f.params.value("position_y_mm",   0.0) },
            { "thread_M",        f.params.value("thread_M",        "M3") },
            { "boss_height_mm",  f.params.value("boss_height_mm",  6.0) },
            { "boss_dia_mm",     f.params.value("boss_dia_mm",     0.0) },
            { "thread_depth_mm", f.params.value("thread_depth_mm", 0.0) },
            { "axis_dir",        f.params.value("axis_dir",        json::array({0.0,0.0,1.0})) },
        };
        json matched = {
            { "source",           "feature_history_replay" },
            { "subfeature_count", p.value("subfeature_count", 0) },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 1.0, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::pcb_standoff_threaded
