// @lat: [[engine/skills#wing_rib_lightening_hole]]

#include "wing_rib_lightening_hole.hpp"

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

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::wing_rib_lightening_hole {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.hole_dia_mm <= 0.0 || in.flange_width_mm <= 0.0 ||
        in.pitch_mm <= 0.0 || in.hole_count <= 0) {
        r.add("DFM-INPUT", "error",
              "wing_rib_lightening_hole: all dims/count must be > 0");
        return r;
    }

    // Adjacent flange seats must not overlap: pitch > dia + 2*flange.
    const double minPitch = in.hole_dia_mm + 2.0 * in.flange_width_mm;
    if (in.pitch_mm <= minPitch) {
        r.add("DFM-PITCH", "error",
              "wing_rib_lightening_hole: pitch_mm (" +
              std::to_string(in.pitch_mm) +
              ") must exceed hole_dia + 2*flange_width (" +
              std::to_string(minPitch) + ")");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "wing_rib_lightening_hole DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double thickness = zMax - zMin;        // rib web thickness

    const double boreR     = in.hole_dia_mm / 2.0;
    const double flangeR   = boreR + in.flange_width_mm;
    const double boreDepth = thickness + 2.0 * kOver;          // through bore
    const double flangeDepth = std::min(thickness * 0.4, in.flange_width_mm);

    const double ox = in.row_origin.X();
    const double oy = in.row_origin.Y();

    TopoDS_Shape current = wp.shape();

    // Build sequentially: per hole, bore THEN flange counterbore.
    for (int i = 0; i < in.hole_count; ++i) {
        const double cx = ox + static_cast<double>(i) * in.pitch_mm;
        const double cy = oy;

        // 1) through bore
        const gp_Pnt boreStart(cx, cy, topZ + kOver - boreDepth);
        const gp_Ax2 boreAx(boreStart, gp::DZ());
        current = pr::cut(current, pr::cylinder(boreAx, boreR, boreDepth));

        // 2) shallow flange counterbore (wider, shallow, top seat)
        const gp_Pnt flStart(cx, cy, topZ - flangeDepth);
        const gp_Ax2 flAx(flStart, gp::DZ());
        current = pr::cut(current,
                          pr::cylinder(flAx, flangeR, flangeDepth + kOver));
    }

    // Analytic removed volume: N*(bore-through) + N*(flange annulus seat).
    const double vBore = static_cast<double>(in.hole_count) *
                         M_PI * boreR * boreR * thickness;
    const double vFlange = static_cast<double>(in.hole_count) *
                           M_PI * (flangeR * flangeR - boreR * boreR) *
                           flangeDepth;
    const double volRemoved = vBore + vFlange;

    json holes_json = json::array();
    for (int i = 0; i < in.hole_count; ++i)
        holes_json.push_back({ { "x_mm", ox + static_cast<double>(i) * in.pitch_mm },
                               { "y_mm", oy } });

    json params = {
        { "row_origin",      { in.row_origin.X(),
                               in.row_origin.Y(),
                               in.row_origin.Z() } },
        { "hole_dia_mm",     in.hole_dia_mm },
        { "flange_width_mm", in.flange_width_mm },
        { "hole_count",      in.hole_count },
        { "pitch_mm",        in.pitch_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "aerostruct_feature_type",    "wing_rib_lightening_hole" },
        { "subfeature_count",           in.hole_count * 2 },
        { "hole_count",                 in.hole_count },
        { "hole_positions",             holes_json },
        { "derived_bore_dia_mm",        in.hole_dia_mm },
        { "derived_flange_dia_mm",      2.0 * flangeR },
        { "derived_flange_depth_mm",    flangeDepth },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "wing-rib lightening hole (web)" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;counterbore";
    tooling.tool_dia_mm       = in.hole_dia_mm;
    tooling.tool_length_mm    = boreDepth + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(30.0, 12.0 * in.hole_count);
    tooling.extra = {
        { "aerostruct_feature_type", "wing_rib_lightening_hole" },
        { "hole_count",              in.hole_count },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::wing_rib_lightening_hole: {} holes dia={} pitch={}",
                  in.hole_count, in.hole_dia_mm, in.pitch_mm);

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
            { "source",                  "metadata_replay" },
            { "is_compound",             true },
            { "aerostruct_feature_type", "wing_rib_lightening_hole" },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: a row of equal-radius +Z cylinders (lightening bores).
    int bigCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Dir d = s.Cylinder().Axis().Direction();
            if (std::abs(d.Z()) > 0.9 && s.Cylinder().Radius() >= 5.0) ++bigCyls;
        } catch (...) {}
    }
    if (bigCyls >= 2) {
        json recovered = { { "hole_dia_mm", 20.0 }, { "hole_count", bigCyls } };
        json matched   = { { "source",   "geometric_lightening_row" },
                           { "big_cyls", bigCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::wing_rib_lightening_hole
