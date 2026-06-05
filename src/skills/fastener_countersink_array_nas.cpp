// @lat: [[engine/skills#fastener_countersink_array_nas]]

#include "fastener_countersink_array_nas.hpp"

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

namespace koocadcam::skill::fastener_countersink_array_nas {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;

bool isValidAngle(double a)
{
    return std::abs(a - 82.0)  < 1e-6 ||
           std::abs(a - 100.0) < 1e-6 ||
           std::abs(a - 120.0) < 1e-6;
}
}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.fastener_dia_mm <= 0.0 || in.pitch_mm <= 0.0 || in.count <= 0) {
        r.add("DFM-INPUT", "error",
              "fastener_countersink_array_nas: all dims/count must be > 0");
        return r;
    }

    if (!isValidAngle(in.csk_angle_deg)) {
        r.add("DFM-ANGLE", "error",
              "fastener_countersink_array_nas: csk_angle_deg (" +
              std::to_string(in.csk_angle_deg) +
              ") must be one of {82, 100, 120}");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "fastener_countersink_array_nas DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double thickness = zMax - zMin;

    const double boreR     = in.fastener_dia_mm / 2.0;
    const double boreDepth = thickness + 2.0 * kOver;

    // Countersink: included angle -> half angle; head dia ~ 1.8x shank.
    const double halfAngleRad = (in.csk_angle_deg / 2.0) * M_PI / 180.0;
    const double cskTopR  = in.fastener_dia_mm * 0.9;        // head seat radius
    const double cskDepth = (cskTopR - boreR) / std::tan(halfAngleRad);

    const double ox = in.row_origin.X();
    const double oy = in.row_origin.Y();

    TopoDS_Shape current = wp.shape();

    // Build sequentially: per fastener, bore THEN countersink cone.
    for (int i = 0; i < in.count; ++i) {
        const double cx = ox + static_cast<double>(i) * in.pitch_mm;
        const double cy = oy;

        // 1) drilled bore
        const gp_Pnt boreStart(cx, cy, topZ + kOver - boreDepth);
        const gp_Ax2 boreAx(boreStart, gp::DZ());
        current = pr::cut(current, pr::cylinder(boreAx, boreR, boreDepth));

        // 2) conical countersink: narrow (boreR) at bottom, wide (cskTopR) at top
        const gp_Pnt coneStart(cx, cy, topZ - cskDepth);
        const gp_Ax2 coneAx(coneStart, gp::DZ());
        current = pr::cut(current,
                          pr::coneFrustum(coneAx, boreR, cskTopR, cskDepth + kOver));
    }

    // Analytic removed volume: N*(bore through) + N*(countersink frustum).
    const double vBore = static_cast<double>(in.count) *
                         M_PI * boreR * boreR * thickness;
    const double vCsk = static_cast<double>(in.count) *
                        (M_PI * cskDepth / 3.0) *
                        (boreR * boreR + boreR * cskTopR + cskTopR * cskTopR);
    const double volRemoved = vBore + vCsk;

    json fast_json = json::array();
    for (int i = 0; i < in.count; ++i)
        fast_json.push_back({ { "x_mm", ox + static_cast<double>(i) * in.pitch_mm },
                              { "y_mm", oy } });

    json params = {
        { "row_origin",      { in.row_origin.X(),
                               in.row_origin.Y(),
                               in.row_origin.Z() } },
        { "fastener_dia_mm", in.fastener_dia_mm },
        { "csk_angle_deg",   in.csk_angle_deg },
        { "count",           in.count },
        { "pitch_mm",        in.pitch_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "aerostruct_feature_type",    "fastener_countersink_array_nas" },
        { "subfeature_count",           in.count * 2 },
        { "fastener_count",             in.count },
        { "fastener_positions",         fast_json },
        { "csk_angle_deg",              in.csk_angle_deg },
        { "derived_bore_dia_mm",        in.fastener_dia_mm },
        { "derived_csk_head_dia_mm",    2.0 * cskTopR },
        { "derived_csk_depth_mm",       cskDepth },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "NAS flush rivet countersink" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "drill;countersink";
    tooling.tool_dia_mm       = in.fastener_dia_mm;
    tooling.tool_length_mm    = boreDepth + 10.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 3;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(30.0, 10.0 * in.count);
    tooling.extra = {
        { "aerostruct_feature_type", "fastener_countersink_array_nas" },
        { "csk_angle_deg",           in.csk_angle_deg },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::fastener_countersink_array_nas: {} fasteners dia={} csk={}",
                  in.count, in.fastener_dia_mm, in.csk_angle_deg);

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
            { "aerostruct_feature_type", "fastener_countersink_array_nas" },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: a row of conical faces (countersinks).
    int coneFaces = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        try {
            BRepAdaptor_Surface s(wp.face(i));
            if (s.GetType() == GeomAbs_Cone) ++coneFaces;
        } catch (...) {}
    }
    if (coneFaces >= 2) {
        json recovered = { { "fastener_dia_mm", 4.0 },
                           { "csk_angle_deg",   100.0 },
                           { "count",           coneFaces } };
        json matched   = { { "source",     "geometric_csk_row" },
                           { "cone_faces", coneFaces } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::fastener_countersink_array_nas
