// @lat: [[engine/skills#reference_plane_at_3pts]]
//
// Metadata-only construction geometry: workpiece shape is unchanged; the
// plane defined by 3 points (normal = unit((B-A) x (C-A)), origin =
// centroid) is captured in the FeatureSignature.

#include "reference_plane_at_3pts.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

namespace koocadcam::skill::reference_plane_at_3pts {

using nlohmann::json;

namespace {

struct V3 { double x, y, z; };

inline V3 sub(const std::array<double,3>& a, const std::array<double,3>& b)
{ return { a[0] - b[0], a[1] - b[1], a[2] - b[2] }; }

inline V3 cross(const V3& u, const V3& v)
{
    return { u.y * v.z - u.z * v.y,
             u.z * v.x - u.x * v.z,
             u.x * v.y - u.y * v.x };
}

inline double mag(const V3& u)
{ return std::sqrt(u.x * u.x + u.y * u.y + u.z * u.z); }

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.name.empty()) {
        r.add("DFM-INPUT", "error",
              "reference_plane_at_3pts: name must be non-empty");
    }
    const V3 ab = sub(in.point_b, in.point_a);
    const V3 ac = sub(in.point_c, in.point_a);
    const V3 nx = cross(ab, ac);
    const double m = mag(nx);
    if (m < 1e-3) {
        r.add("DFM-REF-COLLINEAR", "error",
              "reference_plane_at_3pts: points are collinear "
              "(|AB x AC| = " + std::to_string(m) + " < 1e-3) — "
              "plane is undefined");
    }
    return r;
}

// ── Synthesis (metadata only) ────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "reference_plane_at_3pts DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const V3 ab = sub(in.point_b, in.point_a);
    const V3 ac = sub(in.point_c, in.point_a);
    const V3 nx = cross(ab, ac);
    const double m = mag(nx);
    // m > 1e-3 guaranteed by validate.
    const double nxh = nx.x / m;
    const double nyh = nx.y / m;
    const double nzh = nx.z / m;

    const double cx = (in.point_a[0] + in.point_b[0] + in.point_c[0]) / 3.0;
    const double cy = (in.point_a[1] + in.point_b[1] + in.point_c[1]) / 3.0;
    const double cz = (in.point_a[2] + in.point_b[2] + in.point_c[2]) / 3.0;

    json params = {
        { "point_a", in.point_a },
        { "point_b", in.point_b },
        { "point_c", in.point_c },
        { "name",    in.name },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_reference",      true },
        { "datum_class",       "plane" },
        { "named_id",          in.name },
        { "point_a_xyz",       in.point_a },
        { "point_b_xyz",       in.point_b },
        { "point_c_xyz",       in.point_c },
        { "plane_origin_xyz",  { cx, cy, cz } },
        { "plane_normal_xyz",  { nxh, nyh, nzh } },
        { "cross_magnitude",   m },
        { "geometry_changed",  false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "reference_datum";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "datum_class",     "plane" },
        { "geometric_no_op", true },
        { "named_id",        in.name },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::reference_plane_at_3pts: name={} |AB x AC|={}",
                  in.name, m);

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
        rf.matched_geometry = { { "source", "feature_history" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::reference_plane_at_3pts
