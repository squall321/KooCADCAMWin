// @lat: [[engine/skills#e_clip_groove_din6799]]

#include "e_clip_groove_din6799.hpp"

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

namespace koocadcam::skill::e_clip_groove_din6799 {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;
using retaining_rings::findDin6799;
using retaining_rings::Din6799Spec;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.ring_size_key.empty()) {
        r.add("DFM-DIN6799", "error",
              "e_clip_groove_din6799: ring_size_key is empty");
        return r;
    }
    const Din6799Spec* spec = findDin6799(in.ring_size_key);
    if (!spec) {
        r.add("DFM-DIN6799", "error",
              "e_clip_groove_din6799: unknown DIN 6799 size '" +
              in.ring_size_key + "'");
        return r;
    }

    if (wp.shape().IsNull()) {
        r.add("DFM-INPUT", "error",
              "e_clip_groove_din6799: workpiece is null");
        return r;
    }

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double depth =
        (spec->shaft_dia_mm - spec->groove_dia_mm) / 2.0;
    const double zLow  = in.shaft_z_position_mm - spec->groove_width_mm / 2.0;
    const double zHigh = in.shaft_z_position_mm + spec->groove_width_mm / 2.0;
    if (zLow < bb.zMin - 1e-3 || zHigh > bb.zMax + 1e-3) {
        r.add("DFM-DIN6799-Z", "error",
              "e_clip_groove_din6799: groove Z zone outside stock Z");
    }

    const double outerR = bb.outerRadiusXY();
    if (outerR > 1e-6 && depth >= outerR - 0.5) {
        r.add("DFM-DIN6799-DEPTH", "error",
              "e_clip_groove_din6799: groove depth " + std::to_string(depth) +
              " exceeds available shaft radius " + std::to_string(outerR));
    }

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) {
        r.add("DFM-INPUT", "error",
              "e_clip_groove_din6799: face_id datum unresolved");
    } else if (!wp.isFaceCylinder(*faceId)) {
        r.add("DFM-DIN6799-FACE", "error",
              "e_clip_groove_din6799: face_id is not a cylindrical face");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "e_clip_groove_din6799 DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const Din6799Spec* spec = findDin6799(in.ring_size_key);
    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double outerR  = bb.outerRadiusXY();
    const double depth   = (spec->shaft_dia_mm - spec->groove_dia_mm) / 2.0;
    const double grooveR = outerR - depth;

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
        { "shaft_dia_mm",                spec->shaft_dia_mm },
        { "groove_dia_mm",               spec->groove_dia_mm },
        { "groove_width_mm",             spec->groove_width_mm },
        { "groove_depth_mm",             depth },
        { "groove_axial_position_mm",    spec->groove_axial_position_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "ring_standard",         kRingStandard },
        { "ring_size_key",         in.ring_size_key },
        { "derived_dims",          derived },
        { "ring_volume_mm3",       ringVol },
        { "center_z_mm",           in.shaft_z_position_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "lathe_groove_insert";
    tooling.tool_dia_mm       = spec->groove_width_mm;
    tooling.tool_length_mm    = depth + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 500.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = ringVol;
    tooling.est_cycle_time_s  = std::max(0.8, depth / 4.0);
    tooling.extra = {
        { "standard",      kRingStandard },
        { "ring_size_key", in.ring_size_key },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::e_clip_groove_din6799 applied: size={} d={} w={}",
                  in.ring_size_key, depth, spec->groove_width_mm);

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
        std::string bestKey;
        double bestErr = 1e9;
        for (const auto& s : retaining_rings::kDin6799) {
            const double sdepth = (s.shaft_dia_mm - s.groove_dia_mm) / 2.0;
            const double err = std::abs(sdepth - depthEst);
            if (err < bestErr) { bestErr = err; bestKey = s.size_key; }
        }
        if (bestKey.empty() || bestErr > 0.3) continue;
        json recovered = {
            { "ring_size_key",       bestKey },
            { "shaft_z_position_mm", 0.0 },
        };
        json matched = {
            { "source",          "geometric_fallback" },
            { "cyl_face_id",     fIdx },
            { "depth_est",       depthEst },
            { "depth_match_err", bestErr },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::e_clip_groove_din6799
