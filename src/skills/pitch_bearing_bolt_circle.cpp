// @lat: [[engine/skills#pitch_bearing_bolt_circle]]

#include "pitch_bearing_bolt_circle.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::pitch_bearing_bolt_circle {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bolt_circle_dia_mm <= 0.0 || in.bolt_dia_mm <= 0.0 ||
        in.center_bore_dia_mm <= 0.0 || in.bore_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pitch_bearing_bolt_circle: all dims must be > 0");
        return r;
    }

    if (in.bolt_count < 12 || in.bolt_count > 120) {
        r.add("DFM-BOLT-COUNT", "error",
              "pitch_bearing_bolt_circle: bolt_count " +
              std::to_string(in.bolt_count) +
              " outside utility band [12, 120]");
    }

    // Bolt circle must clear the center bore (PCD > bore_dia + 2*bolt_dia).
    if (in.bolt_circle_dia_mm <=
        in.center_bore_dia_mm + 2.0 * in.bolt_dia_mm) {
        r.add("DFM-PCD", "error",
              "pitch_bearing_bolt_circle: bolt_circle_dia too small for "
              "center bore + bolt clearance");
    }

    // Bolt holes must fit inside the flange OD (from stock bounding box).
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double flangeR = 0.5 * std::min(xMax - xMin, yMax - yMin);
    const double boltOuterR = in.bolt_circle_dia_mm / 2.0 + in.bolt_dia_mm / 2.0;
    if (boltOuterR >= flangeR) {
        r.add("DFM-STOCK", "error",
              "pitch_bearing_bolt_circle: bolt circle " +
              std::to_string(boltOuterR) +
              " mm exceeds flange radius " + std::to_string(flangeR) + " mm");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pitch_bearing_bolt_circle DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double cx   = in.center_xy.X();
    const double cy   = in.center_xy.Y();

    // ── 1) Central spindle bore (down from top face) ─────────────────────
    const double boreR = in.center_bore_dia_mm / 2.0;
    const gp_Pnt boreStart(cx, cy, topZ - in.bore_depth_mm);
    const gp_Ax2 boreAx(boreStart, gp::DZ());
    TopoDS_Shape current = pr::cut(
        wp.shape(),
        pr::cylinder(boreAx, boreR, in.bore_depth_mm + kOver));

    // ── 2..N+1) Bolt holes via SEQUENTIAL gp_Trsf rotation ───────────────
    const double pcdR    = in.bolt_circle_dia_mm / 2.0;
    const double boltR   = in.bolt_dia_mm / 2.0;
    const double boltDepth = (zMax - zMin) + 2.0 * kOver;  // through-flange

    const gp_Pnt boltSeed(cx + pcdR, cy, zMin - kOver);
    const gp_Ax2 boltSeedAx(boltSeed, gp::DZ());
    const TopoDS_Shape boltTemplate = pr::cylinder(boltSeedAx, boltR, boltDepth);

    const gp_Pnt rotCenter(cx, cy, zMin);
    const gp_Ax1 rotAxis(rotCenter, gp::DZ());

    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        gp_Trsf rot;
        rot.SetRotation(rotAxis, theta);
        BRepBuilderAPI_Transform xform(boltTemplate, rot, true);
        if (!xform.IsDone()) {
            throw SkillError(
                "pitch_bearing_bolt_circle: bolt rotation failed");
        }
        current = pr::cut(current, xform.Shape());  // sequential — no compound
    }

    // ── Derived volume (analytic) ────────────────────────────────────────
    const double thru   = (zMax - zMin);
    const double vBore   = M_PI * boreR * boreR * in.bore_depth_mm;
    const double vBolts  = static_cast<double>(in.bolt_count) *
                           M_PI * boltR * boltR * thru;
    const double volRemoved = vBore + vBolts;

    json bolts_json = json::array();
    for (int i = 0; i < in.bolt_count; ++i) {
        const double theta = (2.0 * M_PI * static_cast<double>(i)) /
                             static_cast<double>(in.bolt_count);
        bolts_json.push_back({
            { "x_mm",      cx + pcdR * std::cos(theta) },
            { "y_mm",      cy + pcdR * std::sin(theta) },
            { "theta_rad", theta },
        });
    }

    json params = {
        { "center_xy",          { in.center_xy.X(),
                                  in.center_xy.Y(),
                                  in.center_xy.Z() } },
        { "bolt_circle_dia_mm", in.bolt_circle_dia_mm },
        { "bolt_count",         in.bolt_count },
        { "bolt_dia_mm",        in.bolt_dia_mm },
        { "center_bore_dia_mm", in.center_bore_dia_mm },
        { "bore_depth_mm",      in.bore_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "wind_feature_type",          "pitch_bearing_bolt_circle" },
        { "subfeature_count",           in.bolt_count + 1 },
        { "bolt_count",                 in.bolt_count },
        { "bolt_circle_dia_mm",         in.bolt_circle_dia_mm },
        { "bolt_dia_mm",                in.bolt_dia_mm },
        { "center_bore_dia_mm",         in.center_bore_dia_mm },
        { "bolt_positions",             bolts_json },
        { "derived_pcd_radius_mm",      pcdR },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "IEC 61400-1" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;drill";
    tooling.tool_dia_mm       = in.center_bore_dia_mm;
    tooling.tool_length_mm    = boltDepth + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 150.0;
    tooling.feed_per_tooth_mm = 0.06;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 90.0 + 15.0 * static_cast<double>(in.bolt_count);
    tooling.extra = {
        { "wind_feature_type", "pitch_bearing_bolt_circle" },
        { "flange_family",     "slewing_ring" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::pitch_bearing_bolt_circle: PCD={} bolts={} bore Ø{}",
                  in.bolt_circle_dia_mm, in.bolt_count, in.center_bore_dia_mm);

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
            { "source",            "metadata_replay" },
            { "is_compound",       true },
            { "wind_feature_type", "pitch_bearing_bolt_circle" },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: many equal-radius +Z cylinders (bolt holes) plus a
    // single much larger +Z cylinder (center bore).
    int smallCyls = 0;
    double maxR = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.Z()) < 0.9) continue;
            const double radius = s.Cylinder().Radius();
            if (radius > maxR) maxR = radius;
            if (radius < 60.0) ++smallCyls;
        } catch (...) {}
    }
    if (smallCyls >= 12 && maxR > 100.0) {
        json recovered = { { "bolt_count", smallCyls } };
        json matched   = { { "source",     "geometric_bolt_circle" },
                           { "small_cyls", smallCyls },
                           { "max_radius", maxR } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::pitch_bearing_bolt_circle
