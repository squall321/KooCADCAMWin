// @lat: [[engine/skills#coordinate_system_from_axes]]
//
// Metadata-only local coordinate system: workpiece shape unchanged; CSYS
// (origin, unit-X, unit-Y, unit-Z = X x Y) captured in FeatureSignature.

#include "coordinate_system_from_axes.hpp"

#include "Workpiece.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::coordinate_system_from_axes {

using nlohmann::json;

namespace {

struct V3 { double x, y, z; };

inline double mag(const V3& u)
{ return std::sqrt(u.x * u.x + u.y * u.y + u.z * u.z); }

inline V3 unit(const V3& u, double m)
{ return { u.x / m, u.y / m, u.z / m }; }

inline V3 cross(const V3& u, const V3& v)
{
    return { u.y * v.z - u.z * v.y,
             u.z * v.x - u.x * v.z,
             u.x * v.y - u.y * v.x };
}

inline double dot(const V3& u, const V3& v)
{ return u.x * v.x + u.y * v.y + u.z * v.z; }

inline V3 fromArr(const std::array<double,3>& a)
{ return { a[0], a[1], a[2] }; }

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;
    if (in.name.empty()) {
        r.add("DFM-INPUT", "error",
              "coordinate_system_from_axes: name must be non-empty");
    }
    const V3 xv = fromArr(in.x_axis_dir);
    const V3 yv = fromArr(in.y_axis_dir);
    const double xm = mag(xv);
    const double ym = mag(yv);
    if (xm < 1e-9) {
        r.add("DFM-INPUT", "error",
              "coordinate_system_from_axes: x_axis_dir is zero-length");
    }
    if (ym < 1e-9) {
        r.add("DFM-INPUT", "error",
              "coordinate_system_from_axes: y_axis_dir is zero-length");
    }
    if (xm >= 1e-9 && ym >= 1e-9) {
        const V3 xu = unit(xv, xm);
        const V3 yu = unit(yv, ym);
        const double cosA = std::max(-1.0, std::min(1.0, dot(xu, yu)));
        const double angleDeg = std::acos(cosA) * 180.0 / 3.14159265358979323846;
        const double deviation = std::abs(angleDeg - 90.0);
        if (deviation > 5.0) {
            r.add("DFM-REF-ORTHOGONALITY", "error",
                  "coordinate_system_from_axes: angle(X,Y) = " +
                  std::to_string(angleDeg) +
                  "° deviates from 90° by " +
                  std::to_string(deviation) +
                  "° (tolerance 5°)");
        }
    }
    return r;
}

// ── Synthesis (metadata only) ────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "coordinate_system_from_axes DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const V3 xv = fromArr(in.x_axis_dir);
    const V3 yv = fromArr(in.y_axis_dir);
    const V3 xu = unit(xv, mag(xv));
    const V3 yu = unit(yv, mag(yv));
    const V3 zv = cross(xu, yu);
    const double zm = mag(zv);
    // zm > 0 because validate ensures angle(X,Y) is within [85°, 95°].
    const V3 zu = unit(zv, zm);

    const double cosA = std::max(-1.0, std::min(1.0, dot(xu, yu)));
    const double angleDeg = std::acos(cosA) * 180.0 / 3.14159265358979323846;

    json params = {
        { "origin",     in.origin },
        { "x_axis_dir", in.x_axis_dir },
        { "y_axis_dir", in.y_axis_dir },
        { "name",       in.name },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_reference",      true },
        { "datum_class",       "coord_sys" },
        { "named_id",          in.name },
        { "origin_xyz",        in.origin },
        { "x_axis_xyz",        { xu.x, xu.y, xu.z } },
        { "y_axis_xyz",        { yu.x, yu.y, yu.z } },
        { "z_axis_xyz",        { zu.x, zu.y, zu.z } },
        { "angle_xy_deg",      angleDeg },
        { "geometry_changed",  false },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "reference_datum";
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = 0.0;
    tooling.extra = {
        { "datum_class",     "coord_sys" },
        { "geometric_no_op", true },
        { "named_id",        in.name },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::coordinate_system_from_axes: name={} ang(X,Y)={}°",
                  in.name, angleDeg);

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

}  // namespace koocadcam::skill::coordinate_system_from_axes
