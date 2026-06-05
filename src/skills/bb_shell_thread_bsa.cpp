// @lat: [[engine/skills#bb_shell_thread_bsa]]

#include "bb_shell_thread_bsa.hpp"

#include "Workpiece.hpp"
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

namespace koocadcam::skill::bb_shell_thread_bsa {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;

// BSA English thread major diameter (1.370 in nominal).
constexpr double kBsaMajorMm   = 34.798;
constexpr double kBsaToleranceMm = 0.6;   // accept ~34.2 .. 35.4 mm bore
constexpr double kReliefRadialMm = 0.6;   // relief radius beyond major bore
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.shell_width_mm <= 0.0 || in.shell_bore_dia_mm <= 0.0 ||
        in.thread_relief_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "bb_shell_thread_bsa: all dimensions must be > 0");
        return r;
    }

    if (std::abs(in.shell_width_mm - 68.0) > 1.0 &&
        std::abs(in.shell_width_mm - 73.0) > 1.0) {
        r.add("DFM-SHELL", "error",
              "bb_shell_thread_bsa: shell_width_mm " +
              std::to_string(in.shell_width_mm) +
              " is not a standard BSA shell (68 or 73 mm)");
    }

    if (std::abs(in.shell_bore_dia_mm - kBsaMajorMm) > kBsaToleranceMm) {
        r.add("DFM-BORE", "error",
              "bb_shell_thread_bsa: shell_bore_dia_mm " +
              std::to_string(in.shell_bore_dia_mm) +
              " out of BSA major-bore band (34.8 mm +/- 0.6)");
    }

    if (2.0 * in.thread_relief_width_mm >= in.shell_width_mm) {
        r.add("DFM-INPUT", "error",
              "bb_shell_thread_bsa: relief grooves overrun the shell width");
    }

    // The shell bore + relief must fit inside the stock bounding box (radially).
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double reliefR = in.shell_bore_dia_mm / 2.0 + kReliefRadialMm;
    const double stockR  = 0.5 * std::min(xMax - xMin, yMax - yMin);
    if (reliefR >= stockR) {
        r.add("DFM-STOCK", "error",
              "bb_shell_thread_bsa: relief radius " + std::to_string(reliefR) +
              " mm exceeds stock radius " + std::to_string(stockR) + " mm");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "bb_shell_thread_bsa DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx   = in.axis_origin.X();
    const double cy   = in.axis_origin.Y();

    // ── 1) Central through bore (major dia) along the shell axis ─────────
    const double boreR  = in.shell_bore_dia_mm / 2.0;
    const double boreLen = in.shell_width_mm;
    const gp_Pnt boreStart(cx, cy, topZ - boreLen);
    const gp_Ax2 boreAx(boreStart, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::cylinder(boreAx, boreR, boreLen + 2.0 * kOver));

    // ── 2) Drive-side (top) thread-relief annular groove ─────────────────
    const double reliefOuterR = boreR + kReliefRadialMm;
    const double reliefDepth  = in.thread_relief_width_mm;
    const gp_Pnt topReliefStart(cx, cy, topZ - reliefDepth);
    const gp_Ax2 topReliefAx(topReliefStart, gp::DZ());
    current = pr::cut(
        current,
        pr::annularRing(topReliefAx, reliefOuterR, boreR, reliefDepth + kOver));

    // ── 3) Non-drive-side (bottom) thread-relief annular groove ──────────
    const double botZ = topZ - boreLen;
    const gp_Pnt botReliefStart(cx, cy, botZ - kOver);
    const gp_Ax2 botReliefAx(botReliefStart, gp::DZ());
    current = pr::cut(
        current,
        pr::annularRing(botReliefAx, reliefOuterR, boreR, reliefDepth + kOver));

    // ── Derived volume (analytic) ────────────────────────────────────────
    const double vBore = M_PI * boreR * boreR * boreLen;
    const double vRelief = 2.0 * M_PI *
        (reliefOuterR * reliefOuterR - boreR * boreR) * reliefDepth;
    const double volRemoved = vBore + vRelief;

    json params = {
        { "axis_origin",            { in.axis_origin.X(),
                                      in.axis_origin.Y(),
                                      in.axis_origin.Z() } },
        { "shell_width_mm",         in.shell_width_mm },
        { "shell_bore_dia_mm",      in.shell_bore_dia_mm },
        { "thread_relief_width_mm", in.thread_relief_width_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "bicycle_feature_type",       "bsa_threaded_bb_shell" },
        { "subfeature_count",           3 },
        { "shell_width_mm",             in.shell_width_mm },
        { "shell_bore_dia_mm",          in.shell_bore_dia_mm },
        { "derived_thread_major_mm",    kBsaMajorMm },
        { "derived_relief_outer_dia_mm", 2.0 * reliefOuterR },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "BSA 1.37in x 24 TPI" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;thread_tap;groove_mill";
    tooling.tool_dia_mm       = in.shell_bore_dia_mm;
    tooling.tool_length_mm    = boreLen + 20.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 120.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 75.0;
    tooling.extra = {
        { "bicycle_feature_type", "bsa_threaded_bb_shell" },
        { "thread_hand",          "drive_LH;nondrive_RH" },
        { "standard",             "BSA English" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::bb_shell_thread_bsa: width={} bore Ø{} relief {}",
                  in.shell_width_mm, in.shell_bore_dia_mm,
                  in.thread_relief_width_mm);

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
            { "source",               "metadata_replay" },
            { "is_compound",          true },
            { "bicycle_feature_type", "bsa_threaded_bb_shell" },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: a major bore (~17.4 mm radius) plus two slightly
    // wider relief cylinders coaxial with it.
    int boreCyls   = 0;
    int reliefCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 16.5 && radius <= 18.0) ++boreCyls;
            else if (radius > 18.0 && radius <= 19.5) ++reliefCyls;
        } catch (...) {}
    }
    if (boreCyls >= 1 && reliefCyls >= 1) {
        json recovered = { { "shell_bore_dia_mm", kBsaMajorMm },
                           { "shell_width_mm",    68.0 } };
        json matched   = { { "source",      "geometric_bore_relief_pattern" },
                           { "bore_cyls",   boreCyls },
                           { "relief_cyls", reliefCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::bb_shell_thread_bsa
