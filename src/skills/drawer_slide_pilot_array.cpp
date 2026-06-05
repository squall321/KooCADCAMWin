// @lat: [[engine/skills#drawer_slide_pilot_array]]

#include "drawer_slide_pilot_array.hpp"

#include "Workpiece.hpp"
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::drawer_slide_pilot_array {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {
constexpr double kOver = 0.1;
}  // namespace

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!(in.hole_dia_mm > 0.0) || !(in.pitch_mm > 0.0) ||
        !(in.hole_depth_mm > 0.0) || in.hole_count < 2) {
        r.add("DFM-INPUT", "error",
              "drawer_slide_pilot_array: dims must be > 0 and hole_count >= 2");
        return r;
    }
    if (in.pitch_mm <= in.hole_dia_mm) {
        r.add("DFM-PITCH", "error",
              "drawer_slide_pilot_array: pitch_mm " +
              std::to_string(in.pitch_mm) +
              " must exceed hole_dia_mm " + std::to_string(in.hole_dia_mm) +
              " (pilot holes would overlap)");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    if (in.hole_depth_mm >= (zMax - zMin)) {
        r.add("DFM-DEPTH", "error",
              "drawer_slide_pilot_array: hole_depth_mm " +
              std::to_string(in.hole_depth_mm) +
              " >= panel thickness — pilot holes must be blind");
    }
    return r;
}

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "drawer_slide_pilot_array DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    const double ox = in.row_origin.X();
    const double oy = in.row_origin.Y();

    // ── 1..N) pilot holes along +X at `pitch` (SEQUENTIAL cuts) ──────────
    const double holeR = in.hole_dia_mm / 2.0;
    TopoDS_Shape current = wp.shape();
    for (int i = 0; i < in.hole_count; ++i) {
        const double hx = ox + static_cast<double>(i) * in.pitch_mm;
        const gp_Pnt hStart(hx, oy, topZ - in.hole_depth_mm);
        const gp_Ax2 hAx(hStart, gp::DZ());
        current = pr::cut(current, pr::cylinder(hAx, holeR, in.hole_depth_mm + kOver));
    }

    const int subfeatures = in.hole_count;
    const double volRemoved =
        M_PI * holeR * holeR * in.hole_depth_mm * static_cast<double>(in.hole_count);

    json params = {
        { "row_origin",    { in.row_origin.X(),
                             in.row_origin.Y(),
                             in.row_origin.Z() } },
        { "hole_dia_mm",   in.hole_dia_mm },
        { "hole_count",    in.hole_count },
        { "pitch_mm",      in.pitch_mm },
        { "hole_depth_mm", in.hole_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "furniture_feature_type",     "drawer_slide_pilot_array" },
        { "subfeature_count",           subfeatures },
        { "hole_count",                 in.hole_count },
        { "derived_hole_dia_mm",        in.hole_dia_mm },
        { "derived_pitch_mm",           in.pitch_mm },
        { "derived_hole_depth_mm",      in.hole_depth_mm },
        { "derived_row_length_mm",      in.pitch_mm * (in.hole_count - 1) },
        { "derived_volume_removed_mm3", volRemoved },
        { "standard",                   "drawer slide mounting" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "drill";
    tooling.tool_dia_mm       = in.hole_dia_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 180.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = std::max(8.0, 2.0 * in.hole_count + 4.0);
    tooling.extra = {
        { "furniture_application", "drawer_slide_mounting" },
        { "standard",              "drawer slide mounting" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };
    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::drawer_slide_pilot_array: count={} pitch={} dia={}",
                  in.hole_count, in.pitch_mm, in.hole_dia_mm);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric: a linear row of equal small pilot cylinders.
    int pilotCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const double radius = s.Cylinder().Radius();
            if (radius >= 0.8 && radius <= 4.0) ++pilotCyls;
        } catch (...) {}
    }
    if (pilotCyls >= 2) {
        json recovered = { { "hole_dia_mm",   3.0 },
                           { "hole_count",    pilotCyls },
                           { "pitch_mm",      32.0 },
                           { "hole_depth_mm", 10.0 } };
        json matched   = { { "source",     "geometric_pilot_array" },
                           { "pilot_cyls", pilotCyls } };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::drawer_slide_pilot_array
