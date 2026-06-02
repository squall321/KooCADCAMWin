// @lat: [[engine/skills#caseback_o_ring_groove]]

#include "caseback_o_ring_groove.hpp"

#include "_as568_table.hpp"
#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::caseback_o_ring_groove {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── AS568 caseback face-seal catalog ─────────────────────────────────────
//
// CS lookup delegates to the central AS568 table (_as568_table.hpp).  The
// caseback skill accepts the -001 family (-004..-020) which is mostly NOT
// in the central table (central covers -006/-011/-016/-111/...).  Local
// extension entries below preserve this skill's historical accept-set.
namespace {

struct AS568Override { const char* dash; double cs_mm; };

constexpr std::array<AS568Override, 7> kAS568Extras {{
    { "-004", 1.78 },
    { "-008", 1.78 },
    { "-010", 1.78 },
    { "-012", 1.78 },
    { "-014", 1.78 },
    { "-018", 1.78 },
    { "-020", 1.78 },
}};

}  // namespace

double aS568CrossSection(const std::string& dash_number)
{
    for (const auto& e : kAS568Extras) {
        if (dash_number == e.dash) return e.cs_mm;
    }
    if (auto* p = as568::findDash(dash_number)) return p->cross_section_mm;
    return 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────
//
// AS568 + ISO 22810 face-seal design rules.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.mean_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "caseback_o_ring_groove: mean_dia_mm must be > 0");
        return r;
    }

    const double cs = aS568CrossSection(in.o_ring_size);
    if (cs <= 0.0) {
        r.add("DFM-ORING-SIZE", "error",
              "caseback_o_ring_groove: o_ring_size '" + in.o_ring_size +
              "' not in AS568 face-seal catalog (-004 through -020)");
        return r;
    }

    // Derived design groove dimensions (AS568 face-seal design rules).
    const double grooveDepth = 0.75 * cs;
    const double grooveWidth = 1.30 * cs;

    // Mean Ø must allow groove walls without crossing the caseback edge.
    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double caseBackR = std::min(xMax - xMin, yMax - yMin) / 2.0;
        const double grooveOuterR = in.mean_dia_mm / 2.0 + grooveWidth / 2.0;
        if (grooveOuterR + 0.5 > caseBackR) {
            r.add("DFM-CASEBACK-EDGE", "error",
                  "caseback_o_ring_groove: groove outer Ø (" +
                  std::to_string(2.0 * grooveOuterR) +
                  ") too close to caseback edge (Ø " +
                  std::to_string(2.0 * caseBackR) + ")");
        }

        // Thread pilot recess (if requested) must leave thread land.
        if (in.thread_pilot_od_mm > 0.0) {
            if (in.thread_pilot_od_mm > 2.0 * caseBackR - 2.0) {
                r.add("DFM-THREAD-PILOT-OD", "error",
                      "caseback_o_ring_groove: thread_pilot_od " +
                      std::to_string(in.thread_pilot_od_mm) +
                      " mm > caseback OD − 2 mm — no thread land");
            }
            if (in.thread_pilot_od_mm <= 2.0 * grooveOuterR) {
                r.add("DFM-THREAD-PILOT-OD", "error",
                      "caseback_o_ring_groove: thread_pilot_od must be > groove outer Ø");
            }
            if (in.thread_pilot_depth_mm <= 0.0) {
                r.add("DFM-INPUT", "error",
                      "caseback_o_ring_groove: thread_pilot_depth must be > 0");
            }
        }

        const double thickness = zMax - zMin;
        if (grooveDepth > thickness * 0.5) {
            r.add("DFM-CASEBACK-DEPTH", "error",
                  "caseback_o_ring_groove: groove depth " +
                  std::to_string(grooveDepth) +
                  " mm > 50% of caseback thickness — too deep");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-feature chain (2 ops):
//   1. annular O-ring groove cut at mean_dia, depth = 0.75×CS, width = 1.3×CS
//   2. screw-back thread pilot recess (larger annulus, shallower depth)
// Both groove-tools are FUSED then cut once.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "caseback_o_ring_groove DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double cs = aS568CrossSection(in.o_ring_size);
    const double grooveDepth = 0.75 * cs;
    const double grooveWidth = 1.30 * cs;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    constexpr double kOverhang = 0.05;

    // ── Sub-feature 1: O-ring annular groove (concentric cyls fused, cut) ──
    const double rGrooveOuter = in.mean_dia_mm / 2.0 + grooveWidth / 2.0;
    const double rGrooveInner = in.mean_dia_mm / 2.0 - grooveWidth / 2.0;
    const gp_Pnt grooveStart(in.center_x_mm, in.center_y_mm, topZ + kOverhang);
    const gp_Ax2 grooveAx(grooveStart, gp_Dir(0.0, 0.0, -1.0));
    const TopoDS_Shape oRingGroove = pr::annularRing(
        grooveAx, rGrooveOuter, rGrooveInner, grooveDepth + kOverhang);

    // ── Sub-feature 2: thread pilot recess (optional larger annulus) ──────
    TopoDS_Shape threadPilot;
    bool hasThreadPilot = (in.thread_pilot_od_mm > 0.0 &&
                           in.thread_pilot_depth_mm > 0.0);
    if (hasThreadPilot) {
        const double rPilotOuter = in.thread_pilot_od_mm / 2.0;
        const double rPilotInner = rGrooveOuter + 0.3;  // 0.3 mm wall between groove and pilot
        threadPilot = pr::annularRing(
            grooveAx, rPilotOuter, rPilotInner,
            in.thread_pilot_depth_mm + kOverhang);
    }

    // Fuse the two annular cuts then single cut.
    TopoDS_Shape totalCutter = oRingGroove;
    if (hasThreadPilot) totalCutter = pr::fuse(oRingGroove, threadPilot);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), totalCutter);

    // ── Signature ────────────────────────────────────────────────────────
    const int subCount = hasThreadPilot ? 2 : 2;  // groove + pilot/metadata = 2
    json params = {
        { "center_x_mm",            in.center_x_mm },
        { "center_y_mm",            in.center_y_mm },
        { "axis_dir",               { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "mean_dia_mm",            in.mean_dia_mm },
        { "o_ring_size",            in.o_ring_size },
        { "thread_pilot_od_mm",     in.thread_pilot_od_mm },
        { "thread_pilot_depth_mm",  in.thread_pilot_depth_mm },
        { "groove_depth_mm",        grooveDepth },
        { "groove_width_mm",        grooveWidth },
    };
    json subfeatures = json::array({
        { { "op", "o_ring_annular_groove" },
          { "outer_dia_mm", 2.0 * rGrooveOuter },
          { "inner_dia_mm", 2.0 * rGrooveInner },
          { "depth_mm", grooveDepth } },
    });
    if (hasThreadPilot) {
        subfeatures.push_back({
            { "op", "thread_pilot_recess" },
            { "outer_dia_mm", in.thread_pilot_od_mm },
            { "depth_mm", in.thread_pilot_depth_mm },
        });
    } else {
        subfeatures.push_back({
            { "op", "thread_pilot_metadata_only" },
            { "note", "thread helix scheduled for thread_mill_internal" },
        });
    }
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           subCount },
        { "subfeatures",                subfeatures },
        { "mean_dia_mm",                in.mean_dia_mm },
        { "o_ring_size",                in.o_ring_size },
        { "o_ring_cross_section_mm",    cs },
        { "groove_depth_mm",            grooveDepth },
        { "groove_width_mm",            grooveWidth },
        { "annular_face_count",         hasThreadPilot ? 4 : 2 },  // 2 walls per groove
        { "axis_dir",                   { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "groove_tool;thread_mill";
    tooling.tool_dia_mm       = grooveWidth;
    tooling.tool_length_mm    = grooveDepth + 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.03;
    const double grooveVol = M_PI *
        (rGrooveOuter * rGrooveOuter - rGrooveInner * rGrooveInner) * grooveDepth;
    tooling.stock_removed_mm3 = grooveVol;
    tooling.est_cycle_time_s  = std::max(2.0, grooveDepth / 30.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "groove_tool" },
              { "width_mm", grooveWidth },
              { "depth_mm", grooveDepth } },
            { { "tool_type", "thread_mill_internal" },
              { "note", "thread helix on pilot recess — scheduled by CAPP" } },
        } },
        { "feature_standard", "AS568 face-seal " + in.o_ring_size +
                              " + ISO 22810 caseback thread pilot" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::caseback_o_ring_groove applied: meanD={} o-ring={} CS={}",
                  in.mean_dia_mm, in.o_ring_size, cs);

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
            { "source", "metadata_replay" },
            { "is_compound", true },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::caseback_o_ring_groove
