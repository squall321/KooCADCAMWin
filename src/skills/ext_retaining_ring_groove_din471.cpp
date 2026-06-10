// @lat: [[engine/skills#ext_retaining_ring_groove_din471]]

#include "ext_retaining_ring_groove_din471.hpp"

#include "_retaining_rings.hpp"
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

namespace koocadcam::skill::ext_retaining_ring_groove_din471 {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;
using retaining_rings::findDin471;
using retaining_rings::Din471Spec;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.ring_size_key.empty()) {
        r.add("DFM-DIN471", "error",
              "ext_retaining_ring_groove_din471: ring_size_key is empty");
        return r;
    }
    const Din471Spec* spec = findDin471(in.ring_size_key);
    if (!spec) {
        r.add("DFM-DIN471", "error",
              "ext_retaining_ring_groove_din471: unknown DIN 471 size '" +
              in.ring_size_key + "'");
        return r;
    }

    if (wp.shape().IsNull()) {
        r.add("DFM-INPUT", "error",
              "ext_retaining_ring_groove_din471: workpiece is null");
        return r;
    }

    // BBox check: groove must lie within stock Z range.
    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double zLow  = in.shaft_z_position_mm - spec->groove_width_mm / 2.0;
    const double zHigh = in.shaft_z_position_mm + spec->groove_width_mm / 2.0;
    if (zLow < bb.zMin - 1e-3 || zHigh > bb.zMax + 1e-3) {
        r.add("DFM-DIN471-Z", "error",
              "ext_retaining_ring_groove_din471: groove Z zone [" +
              std::to_string(zLow) + "," + std::to_string(zHigh) +
              "] outside stock Z [" + std::to_string(bb.zMin) + "," +
              std::to_string(bb.zMax) + "]");
    }

    // Face datum: must resolve to a cylindrical face (when explicit).
    auto faceId = wp.resolve(in.face_id);
    if (!faceId) {
        r.add("DFM-INPUT", "error",
              "ext_retaining_ring_groove_din471: face_id datum unresolved");
    } else if (!wp.isFaceCylinder(*faceId)) {
        r.add("DFM-DIN471-FACE", "error",
              "ext_retaining_ring_groove_din471: face_id is not a cylindrical face");
    }

    // Shaft Ø must roughly match table spec (within 10%) — sanity rather than
    // strict gate, since the stock may be larger than nominal.
    const double outerR = bb.outerRadiusXY();
    if (outerR > 1e-6) {
        const double err = std::abs(2.0 * outerR - spec->shaft_dia_mm) /
                           spec->shaft_dia_mm;
        if (err > 0.30) {
            r.add("DFM-DIN471-FIT", "warning",
                  "ext_retaining_ring_groove_din471: stock outer Ø " +
                  std::to_string(2.0 * outerR) +
                  " differs from DIN 471 nominal " +
                  std::to_string(spec->shaft_dia_mm) + " by >30%");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ext_retaining_ring_groove_din471 DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const Din471Spec* spec = findDin471(in.ring_size_key);
    // validate() guaranteed spec != nullptr.

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double outerR = bb.outerRadiusXY();
    const double grooveR = outerR - spec->groove_depth_mm;

    constexpr double kOverhangR = 0.5;
    const double startZ = in.shaft_z_position_mm - spec->groove_width_mm / 2.0;
    const double height = spec->groove_width_mm;
    const gp_Ax2 ax(gp_Pnt(0.0, 0.0, startZ), gp::DZ());
    const TopoDS_Shape cutter = pr::annularRing(ax,
                                                outerR + kOverhangR,
                                                grooveR,
                                                height);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    const double ringVol =
        M_PI * (outerR * outerR - grooveR * grooveR) * spec->groove_width_mm;

    json params = {
        { "face_id_kind",          "datum" },
        { "shaft_z_position_mm",   in.shaft_z_position_mm },
        { "ring_size_key",         in.ring_size_key },
    };
    json derived = {
        { "shaft_dia_mm",          spec->shaft_dia_mm },
        { "groove_dia_mm",         spec->groove_dia_mm },
        { "groove_width_mm",       spec->groove_width_mm },
        { "groove_depth_mm",       spec->groove_depth_mm },
        { "ring_thickness_mm",     spec->ring_thickness_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "ring_standard",         kRingStandard },
        { "ring_size_key",         in.ring_size_key },
        { "derived_dims",          derived },
        { "ring_volume_mm3",       ringVol },
        { "axis",                  { 0.0, 0.0, 1.0 } },
        { "center_z_mm",           in.shaft_z_position_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "lathe_groove_insert";
    tooling.tool_dia_mm       = spec->groove_width_mm;
    tooling.tool_length_mm    = spec->groove_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 500.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = ringVol;
    tooling.est_cycle_time_s  = std::max(1.0, spec->groove_depth_mm / 4.0);
    tooling.extra = {
        { "standard",     kRingStandard },
        { "ring_size_key", in.ring_size_key },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::ext_retaining_ring_groove_din471 applied: size={} d={} w={}",
                  in.ring_size_key, spec->groove_depth_mm, spec->groove_width_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata replay (preferred) + geometric fallback: detect a cylindrical
// face whose radius < bbox.outerRadiusXY and whose axial extent matches a
// DIN 471 groove width.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay
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

    // Geometric fallback
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
        // Search DIN 471 for matching depth (best fit)
        std::string bestKey;
        double      bestErr = 1e9;
        for (const auto& s : retaining_rings::kDin471) {
            const double err = std::abs(s.groove_depth_mm - depthEst);
            if (err < bestErr) { bestErr = err; bestKey = s.size_key; }
        }
        if (bestKey.empty()) continue;
        if (bestErr > 0.5) continue;   // must be within 0.5 mm to qualify

        json recovered = {
            { "ring_size_key",       bestKey },
            { "shaft_z_position_mm", 0.0 },
        };
        json matched = {
            { "source",          "geometric_fallback" },
            { "cyl_face_id",     fIdx },
            { "groove_radius_mm", radius },
            { "depth_match_err", bestErr },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::ext_retaining_ring_groove_din471
