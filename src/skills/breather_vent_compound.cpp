// @lat: [[engine/skills#breather_vent_compound]]

#include "breather_vent_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::breather_vent_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec library ─────────────────────────────────────────────────────────

namespace {

struct SpecEntry {
    const char* key;
    double through_dia_mm;
    double counterbore_dia_mm;
    double counterbore_depth_mm;
    double groove_dia_mm;
    double groove_width_mm;
    double groove_depth_mm;
};

constexpr std::array<SpecEntry, 4> kSpecTable {{
    { "BV-M8",   4.0,  8.0, 2.0, 10.0, 0.8, 0.5 },
    { "BV-M12",  6.0, 12.0, 3.0, 14.0, 1.0, 0.7 },
    { "BV-M16",  8.0, 16.0, 4.0, 18.0, 1.5, 1.0 },
    { "BV-M20", 10.0, 20.0, 5.0, 22.0, 2.0, 1.2 },
}};

}  // namespace

BreatherSpec breatherSpecFor(const std::string& key)
{
    for (const auto& e : kSpecTable) {
        if (key == e.key) {
            return { e.through_dia_mm,    e.counterbore_dia_mm,
                     e.counterbore_depth_mm,
                     e.groove_dia_mm,     e.groove_width_mm,
                     e.groove_depth_mm };
        }
    }
    return {};
}

namespace {

BreatherSpec resolveSpec(const Input& in)
{
    if (!in.spec_key.empty()) return breatherSpecFor(in.spec_key);
    return { in.through_dia_mm,
             in.counterbore_dia_mm, in.counterbore_depth_mm,
             in.groove_dia_mm,      in.groove_width_mm,
             in.groove_depth_mm };
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!in.spec_key.empty()) {
        BreatherSpec s = breatherSpecFor(in.spec_key);
        if (s.through_dia_mm <= 0.0) {
            r.add("DFM-BV-SP", "error",
                  "breather_vent_compound: unknown spec_key '" + in.spec_key +
                  "' (supported: BV-M8/BV-M12/BV-M16/BV-M20)");
            return r;
        }
    } else {
        if (in.through_dia_mm     <= 0.0 ||
            in.counterbore_dia_mm <= 0.0 ||
            in.counterbore_depth_mm <= 0.0 ||
            in.groove_dia_mm      <= 0.0 ||
            in.groove_width_mm    <= 0.0 ||
            in.groove_depth_mm    <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "breather_vent_compound: spec_key empty AND override "
                  "params not all > 0");
            return r;
        }
    }
    const BreatherSpec eff = resolveSpec(in);

    if (eff.through_dia_mm > 0.0 && eff.through_dia_mm < 0.8) {
        r.add("DFM-BV-THROUGH", "error",
              "breather_vent_compound: through_dia " +
              std::to_string(eff.through_dia_mm) +
              " mm < 0.8 mm (DFM-002 derived)");
    }
    if (eff.counterbore_dia_mm <= eff.through_dia_mm) {
        r.add("DFM-BV-GEOM", "error",
              "breather_vent_compound: counterbore_dia must be > through_dia");
    }
    if (eff.groove_dia_mm <= eff.counterbore_dia_mm) {
        r.add("DFM-BV-GEOM", "error",
              "breather_vent_compound: groove_dia must be > counterbore_dia");
    }

    // Auto-detect wall thickness from workpiece bbox along axis_dir.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    double wallThk = 0.0;
    const gp_Dir adir = in.axis_dir;
    if (std::abs(adir.Z()) > 0.95) {
        wallThk = zMax - zMin;
    } else if (std::abs(adir.X()) > 0.95) {
        wallThk = xMax - xMin;
    } else if (std::abs(adir.Y()) > 0.95) {
        wallThk = yMax - yMin;
    } else {
        // Diagonal: use the diagonal length conservatively.
        wallThk = std::sqrt(
            std::pow(xMax - xMin, 2.0) +
            std::pow(yMax - yMin, 2.0) +
            std::pow(zMax - zMin, 2.0));
    }
    const double consumed =
        eff.counterbore_depth_mm + eff.groove_depth_mm;
    if (wallThk > 0.0 && consumed >= wallThk - 0.5) {
        r.add("DFM-BV-WALL", "error",
              "breather_vent_compound: counterbore + groove depth " +
              std::to_string(consumed) +
              " mm >= wall thickness " + std::to_string(wallThk) +
              " mm − 0.5 mm clearance (no material left for through hole)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "breather_vent_compound DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const BreatherSpec eff = resolveSpec(in);

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("breather_vent_compound: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        std::pow(xMax - xMin, 2.0) +
        std::pow(yMax - yMin, 2.0) +
        std::pow(zMax - zMin, 2.0));
    constexpr double kOverhang = 0.05;

    const gp_Dir adir = in.axis_dir;

    // toolStart anchored at the entry face (top of workpiece for -Z drill).
    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm,
                               zMax + kOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm,
                               zMin - kOverhang);
        }
    } else {
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * (bboxDiag + kOverhang),
            in.position_y_mm - adir.Y() * (bboxDiag + kOverhang),
            (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kOverhang));
    }

    const gp_Ax2 axAll(toolStart, adir);

    // 1) Through cylinder — long enough to pass through bbox.
    const double throughLen = bboxDiag + 2.0 * kOverhang;
    const TopoDS_Shape throughCyl =
        pr::cylinder(axAll, eff.through_dia_mm / 2.0, throughLen);

    // 2) Counterbore cylinder — only counterbore_depth deep + overhang.
    const TopoDS_Shape cbCyl =
        pr::cylinder(axAll, eff.counterbore_dia_mm / 2.0,
                     eff.counterbore_depth_mm + kOverhang);

    // Fuse counterbore + through so the concentric Boolean is robust
    // (per counterbore.cpp pattern — concentric overlapping cylinders
    // need to fuse before cut).
    const TopoDS_Shape stepBoreCutter = pr::fuse(throughCyl, cbCyl);

    // 3) O-ring groove — annular ring positioned at groove start
    //    (entry + counterbore_depth, going groove_depth further along axis).
    //    Use prim::annularRing built at the groove top depth.
    const gp_Pnt grooveStart(
        toolStart.X() + adir.X() * eff.counterbore_depth_mm,
        toolStart.Y() + adir.Y() * eff.counterbore_depth_mm,
        toolStart.Z() + adir.Z() * eff.counterbore_depth_mm);
    const gp_Ax2 grooveAx(grooveStart, adir);
    // annular ring outer = groove_dia / 2; inner = counterbore_dia / 2.
    // The groove SITS in the counterbore wall — it extends radially outward
    // from the counterbore.
    const double grooveOuterR = eff.groove_dia_mm / 2.0;
    const double grooveInnerR = eff.counterbore_dia_mm / 2.0;
    const TopoDS_Shape grooveRing = pr::annularRing(
        grooveAx, grooveOuterR, grooveInnerR, eff.groove_width_mm);

    // Apply BOTH cuts sequentially.  We can also fuse step-bore + groove
    // and do one cut — but the groove geometry is non-overlapping with the
    // through hole, so cutMany suffices.
    std::vector<TopoDS_Shape> tools = { stepBoreCutter, grooveRing };
    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), tools);

    // Stock-removed estimate (counterbore annulus + through + groove ring).
    const double rThru = eff.through_dia_mm / 2.0;
    const double rCb   = eff.counterbore_dia_mm / 2.0;
    const double rGr   = eff.groove_dia_mm / 2.0;
    // Through volume — through length through wall.
    // Use a reasonable wall thickness estimate from bbox along axis.
    double wallThk = 0.0;
    if (std::abs(adir.Z()) > 0.95)      wallThk = zMax - zMin;
    else if (std::abs(adir.X()) > 0.95) wallThk = xMax - xMin;
    else if (std::abs(adir.Y()) > 0.95) wallThk = yMax - yMin;
    else wallThk = bboxDiag * 0.3;
    const double removedMm3 =
        M_PI * rThru * rThru * wallThk +
        M_PI * (rCb * rCb - rThru * rThru) * eff.counterbore_depth_mm +
        M_PI * (rGr * rGr - rCb * rCb) * eff.groove_width_mm;

    json params = {
        { "entry_face_id",        *entryId },
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
        { "spec_key",             in.spec_key },
        { "through_dia_mm",       eff.through_dia_mm },
        { "counterbore_dia_mm",   eff.counterbore_dia_mm },
        { "counterbore_depth_mm", eff.counterbore_depth_mm },
        { "groove_dia_mm",        eff.groove_dia_mm },
        { "groove_width_mm",      eff.groove_width_mm },
        { "groove_depth_mm",      eff.groove_depth_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      3 },     // through + counterbore + groove
        { "spec_key",              in.spec_key },
        { "through_dia_mm",        eff.through_dia_mm },
        { "counterbore_dia_mm",    eff.counterbore_dia_mm },
        { "groove_dia_mm",         eff.groove_dia_mm },
        { "axis_dir",              { adir.X(), adir.Y(), adir.Z() } },
        { "cylindrical_face_count", 3 },    // through + cb + groove inner
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill;counterbore;groove_cutter";
    tooling.tool_dia_mm       = eff.counterbore_dia_mm;
    tooling.tool_length_mm    = wallThk + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = removedMm3;
    tooling.est_cycle_time_s  = std::max(8.0, wallThk * 0.6);
    tooling.extra = {
        { "tool_sequence", json::array({
            json{ { "tool_type", "drill" },
                  { "tool_dia_mm", eff.through_dia_mm },
                  { "note", "through hole for air pass" } },
            json{ { "tool_type", "counterbore" },
                  { "tool_dia_mm", eff.counterbore_dia_mm },
                  { "depth_mm",    eff.counterbore_depth_mm },
                  { "note", "host for PTFE membrane filter" } },
            json{ { "tool_type", "groove_cutter" },
                  { "tool_dia_mm", eff.groove_dia_mm },
                  { "width_mm",    eff.groove_width_mm },
                  { "depth_mm",    eff.groove_depth_mm },
                  { "note", "O-ring gasket retention groove" } },
        }) },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::breather_vent_compound applied: spec={} through={} cb={}",
                  in.spec_key, eff.through_dia_mm, eff.counterbore_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Recognition: look for the 3-cylinder concentric pattern in feature
// history first (replay).  Geometric fallback returns nothing — the
// counterbore::recognize would already pick up the 2-cylinder step bore
// and we'd want a higher-confidence flag from metadata.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "position_x_mm",        f.params.value("position_x_mm", 0.0) },
            { "position_y_mm",        f.params.value("position_y_mm", 0.0) },
            { "axis_dir",             f.params.value("axis_dir",
                                        json::array({0.0,0.0,-1.0})) },
            { "spec_key",             f.params.value("spec_key",     std::string()) },
            { "through_dia_mm",       f.params.value("through_dia_mm", 0.0) },
            { "counterbore_dia_mm",   f.params.value("counterbore_dia_mm", 0.0) },
            { "counterbore_depth_mm", f.params.value("counterbore_depth_mm", 0.0) },
            { "groove_dia_mm",        f.params.value("groove_dia_mm",       0.0) },
            { "groove_width_mm",      f.params.value("groove_width_mm",     0.0) },
            { "groove_depth_mm",      f.params.value("groove_depth_mm",     0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.94, json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::breather_vent_compound
