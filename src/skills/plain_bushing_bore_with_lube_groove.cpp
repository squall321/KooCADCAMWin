// @lat: [[engine/skills#plain_bushing_bore_with_lube_groove]]

#include "plain_bushing_bore_with_lube_groove.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/HelicalSweep.hpp"
#include "engine/primitives/Tools.hpp"

#include <Standard_Failure.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::plain_bushing_bore_with_lube_groove {

namespace pr = koocadcam::engine::prim;
namespace eng = koocadcam::engine;
using nlohmann::json;

namespace {

constexpr double kGrooveDepth_mm  = 0.3;   // SAE J506 lube-groove depth
constexpr double kChamferAxial_mm = 0.5;
constexpr double kChamferAngleDeg = 30.0;

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.bore_dia_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": bore_dia_mm must be > 0");
    if (in.length_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": length_mm must be > 0");
    if (in.groove_pitch_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": groove_pitch_mm must be > 0");

    if (in.bore_dia_mm > 0.0 && in.bore_dia_mm < 0.8) {
        r.add("DFM-002", "error", std::string(kSkillId) +
              ": bore_dia " + std::to_string(in.bore_dia_mm) +
              " mm < min 0.8 mm");
    }
    if (in.bore_dia_mm > 0.0 && in.length_mm > 0.0) {
        const double ratio = in.length_mm / in.bore_dia_mm;
        if (ratio > 8.0) {
            r.add("DFM-BUSHING-RATIO", "warning", std::string(kSkillId) +
                  ": length/bore_dia " + std::to_string(ratio) +
                  " > 8 (long bushing; chip evacuation / deflection)");
        }
    }
    if (in.bore_dia_mm > 0.0 && in.groove_pitch_mm > 0.0 &&
        in.groove_pitch_mm < 0.5 * in.bore_dia_mm) {
        r.add("DFM-GROOVE-PITCH", "error", std::string(kSkillId) +
              ": groove_pitch " + std::to_string(in.groove_pitch_mm) +
              " mm < 0.5 × bore_dia (turns self-intersect)");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError(std::string(kSkillId) +
                                   ": entry_face datum unresolved");

    const double boreR = in.bore_dia_mm / 2.0;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    constexpr double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;
    gp_Pnt entry(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
    if (!(std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6 && adir.Z() < 0)) {
        const double bboxDiag = std::sqrt(
            (xMax - xMin) * (xMax - xMin) +
            (yMax - yMin) * (yMax - yMin) +
            (zMax - zMin) * (zMax - zMin));
        entry = gp_Pnt(
            in.position_x_mm - adir.X() * (bboxDiag + kEntryOverhang),
            in.position_y_mm - adir.Y() * (bboxDiag + kEntryOverhang),
            (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kEntryOverhang));
    }

    // Sub-feature 1: bore — full length cylinder.
    const gp_Ax2 boreAx(entry, adir);
    const TopoDS_Shape bore =
        pr::cylinder(boreAx, boreR, in.length_mm + kEntryOverhang);

    // Sub-feature 2: helical lube groove — HelicalSweep with thread_depth =
    // kGrooveDepth.  The sweep returns a thin helical ribbon at radius
    // boreR; cutting it from the workpiece carves the groove into the
    // bore wall.
    const gp_Ax1 helixAxis(entry, adir);
    eng::HelicalSweepProfile profileUsed = eng::HelicalSweepProfile::AnnularFallback;
    TopoDS_Shape grooveTool;
    try {
        grooveTool = eng::prim::helicalSweep(
            helixAxis,
            in.groove_pitch_mm,
            in.length_mm,
            boreR,
            kGrooveDepth_mm,
            &profileUsed);
    } catch (const Standard_Failure& e) {
        spdlog::warn("plain_bushing_bore_with_lube_groove: helicalSweep failed ({}); "
                     "falling back to annular ring",
                     e.what());
        // Annular ring of the same axial length as a last-resort groove.
        const gp_Ax2 fbAx(entry, adir);
        grooveTool = pr::annularRing(fbAx,
                                     boreR + kGrooveDepth_mm,
                                     boreR + 1e-4,
                                     in.length_mm);
        profileUsed = eng::HelicalSweepProfile::AnnularFallback;
    }

    // Sub-feature 3: 0.5 × 30° lead-in chamfer.
    const double chamferTopR = boreR + kChamferAxial_mm * std::tan(
        kChamferAngleDeg * M_PI / 180.0);
    const gp_Ax2 chamferAx(entry, adir);
    const TopoDS_Shape chamfer =
        pr::coneFrustum(chamferAx, chamferTopR, boreR, kChamferAxial_mm);

    const TopoDS_Shape fused1 = pr::fuse(bore, grooveTool);
    const TopoDS_Shape fused  = pr::fuse(fused1, chamfer);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "bore_dia_mm",     in.bore_dia_mm },
        { "length_mm",       in.length_mm },
        { "groove_pitch_mm", in.groove_pitch_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      3 },
        { "bore_dia_mm",           in.bore_dia_mm },
        { "length_mm",             in.length_mm },
        { "groove_pitch_mm",       in.groove_pitch_mm },
        { "groove_depth_mm",       kGrooveDepth_mm },
        { "chamfer_axial_mm",      kChamferAxial_mm },
        { "chamfer_angle_deg",     kChamferAngleDeg },
        { "helical_profile_used",  eng::helicalSweepProfileTag(profileUsed) },
        { "axis_dir",              { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type     = "boring_bar;groove_mill;chamfer_tool";
    tooling.tool_dia_mm   = in.bore_dia_mm;
    tooling.tool_material = "carbide";
    tooling.flute_count   = 1;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 =
        M_PI * boreR * boreR * in.length_mm +
        2.0 * M_PI * boreR * (in.length_mm / in.groove_pitch_mm) *
        kGrooveDepth_mm * 0.8;   // helical ribbon approx
    tooling.est_cycle_time_s = std::max(5.0, in.length_mm / 15.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "boring_bar" },
              { "dia_mm", in.bore_dia_mm },
              { "depth_mm", in.length_mm } },
            { { "tool_type", "groove_mill" },
              { "pitch_mm", in.groove_pitch_mm },
              { "depth_mm", kGrooveDepth_mm },
              { "starts", 1 },
              { "profile", eng::helicalSweepProfileTag(profileUsed) } },
            { { "tool_type", "chamfer_tool" },
              { "axial_mm", kChamferAxial_mm },
              { "angle_deg", kChamferAngleDeg } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::plain_bushing_bore_with_lube_groove applied: bd={} l={} p={}",
                  in.bore_dia_mm, in.length_mm, in.groove_pitch_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::plain_bushing_bore_with_lube_groove
