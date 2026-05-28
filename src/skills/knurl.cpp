// @lat: [[engine/skills#knurl]]

#include "knurl.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::knurl {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

bool isKnownPattern(const std::string& p)
{
    return p == "diamond" || p == "straight" ||
           p == "helical_left" || p == "helical_right";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.end_z_mm <= in.start_z_mm) {
        r.add("DFM-INPUT", "error",
              "knurl end_z (" + std::to_string(in.end_z_mm) +
              ") must be > start_z (" + std::to_string(in.start_z_mm) + ")");
    }
    if (in.pitch_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "knurl pitch_mm must be > 0 (got " +
              std::to_string(in.pitch_mm) + ")");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "knurl depth_mm must be > 0 (got " +
              std::to_string(in.depth_mm) + ")");
    }
    if (!isKnownPattern(in.pattern)) {
        r.add("DFM-INPUT", "error",
              "knurl pattern '" + in.pattern +
              "' unknown — must be diamond / straight / helical_left / helical_right");
    }

    if (in.pitch_mm > 0.0 && in.depth_mm > 0.0 &&
        in.depth_mm > in.pitch_mm / 2.0) {
        r.add("DFM-KNURL", "error",
              "knurl depth_mm (" + std::to_string(in.depth_mm) +
              ") > pitch_mm/2 (" + std::to_string(in.pitch_mm / 2.0) +
              ") — adjacent ridges would intersect");
    }

    if (!wp.shape().IsNull() && in.end_z_mm > in.start_z_mm) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        if (in.start_z_mm < bb.zMin - 1e-3 || in.end_z_mm > bb.zMax + 1e-3) {
            r.add("DFM-LATHE", "warning",
                  "knurl Z range [" + std::to_string(in.start_z_mm) +
                  "," + std::to_string(in.end_z_mm) +
                  "] outside stock Z [" + std::to_string(bb.zMin) +
                  "," + std::to_string(bb.zMax) + "]");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Geometry is INTENTIONALLY unchanged.  We only stamp a FeatureSignature
// onto the workpiece.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "knurl DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    double outerR = 0.0;
    if (!wp.shape().IsNull())
        outerR = pr::optimalBbox(wp.shape()).outerRadiusXY();

    json params = {
        { "start_z_mm", in.start_z_mm },
        { "end_z_mm",   in.end_z_mm },
        { "pitch_mm",   in.pitch_mm },
        { "depth_mm",   in.depth_mm },
        { "pattern",    in.pattern },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "axis",              { 0.0, 0.0, 1.0 } },
        { "start_z_mm",        in.start_z_mm },
        { "end_z_mm",          in.end_z_mm },
        { "pitch_mm",          in.pitch_mm },
        { "depth_mm",          in.depth_mm },
        { "pattern",           in.pattern },
        { "approx_radius_mm",  outerR },
        { "geometry_changed",  false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "knurling_wheel";
    tooling.tool_dia_mm       = 20.0;       // typical knurling-wheel OD
    tooling.tool_length_mm    = in.end_z_mm - in.start_z_mm + 5.0;
    tooling.tool_material     = "hss";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 60.0;       // knurling is slow
    tooling.feed_per_tooth_mm = 0.2;
    tooling.stock_removed_mm3 = 0.0;        // metadata-only — displaces, doesn't cut
    tooling.est_cycle_time_s  = std::max(2.0,
        (in.end_z_mm - in.start_z_mm) / 15.0);
    tooling.extra = json{
        { "pattern",          in.pattern },
        { "pitch_mm",         in.pitch_mm },
        { "depth_mm",         in.depth_mm },
        { "geometry_no_op",   true },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::knurl applied (metadata-only): pattern={} pitch={} depth={} Z[{},{}]",
                  in.pattern, in.pitch_mm, in.depth_mm,
                  in.start_z_mm, in.end_z_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay only) ───────────────────────────────────

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

}  // namespace koocadcam::skill::knurl
