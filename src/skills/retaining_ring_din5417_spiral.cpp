// @lat: [[engine/skills#retaining_ring_din5417_spiral]]

#include "retaining_ring_din5417_spiral.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::retaining_ring_din5417_spiral {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Per ISO 5417 §5.2: spiral groove depth equals 1× ring thickness so the
// stacked turns are flush with the shaft surface; groove width equals
// turn_count × ring_thickness with a small clearance.
constexpr double kAxialClearanceFactor = 1.05;  // 5% axial clearance
constexpr double kDepthEqualsThickness = 1.0;   // depth = thk × 1.0

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.shaft_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "retaining_ring_din5417_spiral: shaft_dia_mm must be > 0");
    }
    if (in.ring_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "retaining_ring_din5417_spiral: ring_thickness_mm must be > 0");
    }
    if (in.turn_count != 2 && in.turn_count != 3) {
        r.add("DFM-INPUT", "error",
              "retaining_ring_din5417_spiral: turn_count must be 2 or 3 (got " +
              std::to_string(in.turn_count) + ")");
    }
    if (in.shaft_dia_mm <= 0.0 || in.ring_thickness_mm <= 0.0) return r;

    const double depth  = in.ring_thickness_mm * kDepthEqualsThickness;
    const double width  = in.turn_count * in.ring_thickness_mm * kAxialClearanceFactor;
    const double shaftR = in.shaft_dia_mm / 2.0;
    if (shaftR - depth < 1.0) {
        r.add("DFM-DIN5417-WALL", "error",
              "retaining_ring_din5417_spiral: groove would leave shaft wall "
              "below 1.0 mm");
    }

    if (!wp.shape().IsNull()) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        const double zLow  = in.shaft_z_position_mm - width / 2.0;
        const double zHigh = in.shaft_z_position_mm + width / 2.0;
        if (zLow < bb.zMin - 1e-3 || zHigh > bb.zMax + 1e-3) {
            r.add("DFM-DIN5417-Z", "error",
                  "retaining_ring_din5417_spiral: groove Z zone outside stock Z");
        }
        const double outerR = bb.outerRadiusXY();
        if (outerR > 1e-6 &&
            std::abs(2.0 * outerR - in.shaft_dia_mm) / in.shaft_dia_mm > 0.30) {
            r.add("DFM-DIN5417-FIT", "warning",
                  "retaining_ring_din5417_spiral: stock outer Ø " +
                  std::to_string(2.0 * outerR) +
                  " differs from declared shaft Ø " +
                  std::to_string(in.shaft_dia_mm) + " by >30%");
        }
    }

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) {
        r.add("DFM-INPUT", "error",
              "retaining_ring_din5417_spiral: face_id datum unresolved");
    } else if (!wp.isFaceCylinder(*faceId)) {
        r.add("DFM-DIN5417-FACE", "error",
              "retaining_ring_din5417_spiral: face_id is not a cylindrical face");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "retaining_ring_din5417_spiral DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double outerR = bb.outerRadiusXY();
    const double depth  = in.ring_thickness_mm * kDepthEqualsThickness;
    const double width  = in.turn_count * in.ring_thickness_mm * kAxialClearanceFactor;
    const double grooveR = outerR - depth;

    constexpr double kOverhangR = 0.5;
    const double startZ = in.shaft_z_position_mm - width / 2.0;
    const gp_Ax2 ax(gp_Pnt(0.0, 0.0, startZ), gp::DZ());
    const TopoDS_Shape cutter = pr::annularRing(ax,
                                                outerR + kOverhangR,
                                                grooveR,
                                                width);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double ringVol = M_PI * (outerR * outerR - grooveR * grooveR) * width;

    json params = {
        { "face_id_kind",       "datum" },
        { "shaft_z_position_mm", in.shaft_z_position_mm },
        { "shaft_dia_mm",       in.shaft_dia_mm },
        { "ring_thickness_mm",  in.ring_thickness_mm },
        { "turn_count",         in.turn_count },
    };
    json derived = {
        { "shaft_dia_mm",      in.shaft_dia_mm },
        { "groove_dia_mm",     2.0 * grooveR },
        { "groove_width_mm",   width },
        { "groove_depth_mm",   depth },
        { "ring_thickness_mm", in.ring_thickness_mm },
        { "turn_count",        in.turn_count },
    };
    json pattern = {
        { "kind",            kSkillId },
        { "is_compound",     true },
        { "ring_standard",   kRingStandard },
        { "ring_size_key",   std::to_string(in.turn_count) + "turn_x_" +
                             std::to_string(in.ring_thickness_mm) + "mm" },
        { "derived_dims",    derived },
        { "ring_volume_mm3", ringVol },
        { "center_z_mm",     in.shaft_z_position_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "lathe_groove_insert_wide";
    tooling.tool_dia_mm       = width;
    tooling.tool_length_mm    = depth + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = ringVol;
    tooling.est_cycle_time_s  = std::max(2.0, depth / 3.0 * in.turn_count);
    tooling.extra = {
        { "standard",   kRingStandard },
        { "turn_count", in.turn_count },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::retaining_ring_din5417_spiral applied: thk={} turns={} W={}",
                  in.ring_thickness_mm, in.turn_count, width);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& feat : wp.features()) {
        if (feat.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = feat.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",  "metadata_replay" },
            { "pattern", feat.pattern },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: any annular groove wider than depth × 1.8 is
    // likely a spiral multi-turn ring groove.
    if (wp.shape().IsNull()) return out;
    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double outerR = bb.outerRadiusXY();
    if (outerR <= 1e-6) return out;

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface surf(wp.face(fIdx));
        const gp_Cylinder cyl = surf.Cylinder();
        const double radius = cyl.Radius();
        if (radius >= outerR - 1e-3) continue;
        const gp_Dir adir = cyl.Axis().Direction();
        if (std::abs(std::abs(adir.Z()) - 1.0) > 1e-3) continue;
        const double depthEst = outerR - radius;

        json recovered = {
            { "shaft_dia_mm",        2.0 * outerR },
            { "ring_thickness_mm",   depthEst },
            { "turn_count",          2 },
            { "shaft_z_position_mm", 0.0 },
        };
        json matched = {
            { "source",       "geometric_fallback" },
            { "cyl_face_id",  fIdx },
            { "depth_est",    depthEst },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.45, matched });
        break;
    }
    return out;
}

}  // namespace koocadcam::skill::retaining_ring_din5417_spiral
