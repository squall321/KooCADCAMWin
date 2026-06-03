// @lat: [[engine/skills#pcb_standoff_boss_metric]]

#include "pcb_standoff_boss_metric.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::pcb_standoff_boss_metric {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Small-screw pilot table ──────────────────────────────────────────────
//
// Central _iso_thread_table starts at M3. M1.6 / M2 / M2.5 are kept locally
// (mirrors the auto_standoff_floating_point convention).
namespace {

struct SmallSpec {
    const char* key;
    double      nominal_dia_mm;
    double      pitch_mm;
    double      pilot_dia_mm;     // ISO 965 60% engagement pilot
    double      min_wall_mm;
};

const std::array<SmallSpec, 4>& smallScrewTable()
{
    static const std::array<SmallSpec, 4> kTable {{
        { "M1.6", 1.6,  0.35, 1.25, 0.4 },
        { "M2",   2.0,  0.4,  1.6,  0.4 },
        { "M2.5", 2.5,  0.45, 2.05, 0.5 },
        { "M3",   3.0,  0.5,  2.5,  0.5 },
    }};
    return kTable;
}

const SmallSpec* lookupSmall(const std::string& key)
{
    for (const auto& s : smallScrewTable())
        if (key == s.key) return &s;
    return nullptr;
}

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

double resolvePilot(const std::string& key)
{
    // Prefer central table if it has the size; fall back to small-screw table.
    const auto* m = thread_table::findMetric(key);
    if (m) return m->tap_pilot_dia_mm;
    const auto* sm = lookupSmall(key);
    if (sm) return sm->pilot_dia_mm;
    return 0.0;
}

double resolveWallMin(const std::string& key)
{
    const auto* sm = lookupSmall(key);
    if (sm) return sm->min_wall_mm;
    return 0.5;   // default
}

// Synthesised boss outer diameter when caller leaves boss_dia=0.
double computeBossDia(const std::string& key)
{
    const double pilot = resolvePilot(key);
    const double wall  = resolveWallMin(key);
    // boss_dia ≥ pilot + 2 × wall; pad with 0.4 mm for tolerance.
    return std::max(2.4, pilot + 2.0 * wall + 0.4);
}

}  // namespace

// Public helper for tests.
double pilotDiameterFor(const std::string& thread_size_key)
{
    return resolvePilot(thread_size_key);
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "pcb_standoff_boss_metric: face_id out of range");
    }
    if (!(in.boss_height_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "pcb_standoff_boss_metric: boss_height_mm must be > 0");
    }
    if (!(in.thread_depth_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "pcb_standoff_boss_metric: thread_depth_mm must be > 0");
    }

    const double pilot = resolvePilot(in.thread_size_key);
    if (pilot <= 0.0) {
        r.add("DFM-BAD-SPEC", "error",
              "pcb_standoff_boss_metric: unknown thread_size_key '" +
              in.thread_size_key + "' (supported: M1.6 / M2 / M2.5 / M3)");
        return r;
    }

    // DFM-MIN-HOLE: holes ≥ 0.3 mm
    if (pilot < 0.3) {
        r.add("DFM-MIN-HOLE", "error",
              "pcb_standoff_boss_metric: pilot dia " +
              std::to_string(pilot) + " < 0.3 mm minimum");
    }

    const double bossDia = computeBossDia(in.thread_size_key);
    const double wall    = (bossDia - pilot) / 2.0;
    if (wall < 0.4) {
        r.add("DFM-MIN-WALL", "error",
              "pcb_standoff_boss_metric: wall " +
              std::to_string(wall) + " mm < 0.4 mm minimum");
    }

    if (in.boss_height_mm > 0.0 && bossDia > 0.0) {
        const double ar = in.boss_height_mm / bossDia;
        if (ar > 5.0) {
            r.add("DFM-AR", "warning",
                  "pcb_standoff_boss_metric: aspect ratio " +
                  std::to_string(ar) + " > 5 (boss may warp)");
        }
    }

    if (in.thread_depth_mm > 0.0 && in.boss_height_mm > 0.0 &&
        in.thread_depth_mm > in.boss_height_mm - 0.3) {
        r.add("DFM-DEPTH", "warning",
              "pcb_standoff_boss_metric: thread_depth " +
              std::to_string(in.thread_depth_mm) +
              " leaves floor < 0.3 mm under boss_height " +
              std::to_string(in.boss_height_mm));
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pcb_standoff_boss_metric DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double pilot   = resolvePilot(in.thread_size_key);
    const double bossDia = computeBossDia(in.thread_size_key);
    const double rBoss   = bossDia * 0.5;
    const double rPilot  = pilot   * 0.5;

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const double baseZ = faceC.Z();

    constexpr double kEmbed = 0.05;
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;

    const double volBefore = volumeOf(wp.shape());

    // ── Op 1: ADD boss cylinder (FUSE first) ────────────────────────────
    //
    // Boss bottom sits just below baseZ by kEmbed so it merges into the
    // host face without leaving a thin sliver.
    const gp_Pnt bossOrigin(cx, cy, baseZ - kEmbed);
    const gp_Ax2 axBoss(bossOrigin, gp::DZ());
    const TopoDS_Shape bossTool =
        pr::cylinder(axBoss, rBoss, in.boss_height_mm + kEmbed);

    TopoDS_Shape working = pr::fuse(wp.shape(), bossTool);
    if (working.IsNull()) {
        throw SkillError("pcb_standoff_boss_metric: boss fuse returned null");
    }

    // ── Op 2: CUT tapped pilot from the top of the boss ─────────────────
    const double bossTopZ = baseZ + in.boss_height_mm;
    const gp_Pnt pilotTop(cx, cy, bossTopZ + kEmbed);
    const gp_Ax2 axPilot(pilotTop, gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape pilotTool =
        pr::cylinder(axPilot, rPilot, in.thread_depth_mm + kEmbed);

    TopoDS_Shape newShape = pr::cut(working, pilotTool);
    if (newShape.IsNull()) {
        throw SkillError("pcb_standoff_boss_metric: pilot cut returned null");
    }

    const double volAfter   = volumeOf(newShape);
    const double netDelta   = volAfter - volBefore;        // (boss − pilot)
    const double vBoss      = M_PI * rBoss  * rBoss  * in.boss_height_mm;
    const double vPilot     = M_PI * rPilot * rPilot * in.thread_depth_mm;

    json params = {
        { "face_id",         in.face_id },
        { "center_x_mm",     in.center_x_mm },
        { "center_y_mm",     in.center_y_mm },
        { "boss_height_mm",  in.boss_height_mm },
        { "thread_size_key", in.thread_size_key },
        { "thread_depth_mm", in.thread_depth_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "is_phone_feature",    true },
        { "phone_feature_type",  "pcb_standoff_boss" },
        { "subfeature_count",    2 },
        { "thread_size_key",     in.thread_size_key },
        { "boss_dia_mm",         bossDia },
        { "pilot_dia_mm",        pilot },
        { "boss_height_mm",      in.boss_height_mm },
        { "thread_depth_mm",     in.thread_depth_mm },
        { "added_volume_mm3",    vBoss },
        { "removed_volume_mm3",  vPilot },
        { "net_delta_mm3",       netDelta },
        { "standard",            "ISO 68-1" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "mould;drill;tap";
    tooling.tool_dia_mm       = pilot;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 180.0;
    tooling.stock_removed_mm3 = vPilot;
    tooling.est_cycle_time_s  = 25.0;
    tooling.extra = {
        { "boss_volume_mm3",  vBoss },
        { "pilot_volume_mm3", vPilot },
        { "thread_size_key",  in.thread_size_key },
        { "standard",         "ISO 68-1" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::pcb_standoff_boss_metric: thread={} bossDia={} pilot={} h={}",
        in.thread_size_key, bossDia, pilot, in.boss_height_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

namespace {

struct CylRec { int idx; double radius; gp_Ax1 axis; };

std::vector<CylRec> collectCyls(const Workpiece& wp)
{
    std::vector<CylRec> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        out.push_back({ i, s.Cylinder().Radius(), s.Cylinder().Axis() });
    }
    return out;
}

bool axesNear(const gp_Ax1& a, const gp_Ax1& b, double posTol)
{
    const double dot = std::abs(
        a.Direction().X() * b.Direction().X() +
        a.Direction().Y() * b.Direction().Y() +
        a.Direction().Z() * b.Direction().Z());
    if (dot < 0.999) return false;
    const gp_Pnt& pa = a.Location();
    const gp_Pnt& pb = b.Location();
    return std::sqrt(std::pow(pa.X() - pb.X(), 2) +
                     std::pow(pa.Y() - pb.Y(), 2)) < posTol;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    auto emitMetadataReplay = [&]() {
        for (const auto& f : wp.features()) {
            if (f.skill_id != kSkillId) continue;
            RecognizedFeature rf;
            rf.skill_id         = kSkillId;
            rf.recovered_params = f.params;
            rf.confidence       = 1.0;
            rf.matched_geometry = { { "via", "metadata_replay" } };
            out.push_back(rf);
            return;
        }
    };

    // Geometric: find concentric (boss_outer, pilot) cylinder pair whose
    // pilot diameter matches the small-screw table within 0.3 mm.
    const auto cyls = collectCyls(wp);
    if (cyls.size() >= 2) {
        for (size_t i = 0; i < cyls.size(); ++i) {
            for (size_t j = i + 1; j < cyls.size(); ++j) {
                if (!axesNear(cyls[i].axis, cyls[j].axis, 0.5)) continue;
                const CylRec& small =
                    (cyls[i].radius < cyls[j].radius) ? cyls[i] : cyls[j];
                const CylRec& large =
                    (cyls[i].radius < cyls[j].radius) ? cyls[j] : cyls[i];
                if (large.radius - small.radius < 0.3) continue;

                const std::string keyGuess = [&]() -> std::string {
                    for (const auto& s : smallScrewTable()) {
                        if (std::abs(2.0 * small.radius - s.pilot_dia_mm) < 0.3) {
                            return s.key;
                        }
                    }
                    return {};
                }();
                if (keyGuess.empty()) continue;

                json recovered = {
                    { "face_id",         -1 },
                    { "center_x_mm",     small.axis.Location().X() },
                    { "center_y_mm",     small.axis.Location().Y() },
                    { "boss_height_mm",  2.0 },
                    { "thread_size_key", keyGuess },
                    { "thread_depth_mm", 1.4 },
                };
                json matched = {
                    { "boss_face_id",  large.idx },
                    { "pilot_face_id", small.idx },
                    { "via",           "geometric_primary" },
                };
                out.push_back(RecognizedFeature{ kSkillId, recovered, 0.78, matched });
            }
        }
    }

    if (out.empty()) emitMetadataReplay();
    return out;
}

}  // namespace koocadcam::skill::pcb_standoff_boss_metric
