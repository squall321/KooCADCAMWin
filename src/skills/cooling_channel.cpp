// @lat: [[engine/skills#cooling_channel]]

#include "cooling_channel.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::cooling_channel {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double dist(const Waypoint& a, const Waypoint& b)
{
    return std::hypot(std::hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm),
                      b.z_mm - a.z_mm);
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.waypoints.size() < 2) {
        r.add("DFM-COOL-N", "error",
              "cooling_channel requires ≥ 2 waypoints");
    }

    if (in.dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "cooling_channel dia_mm must be > 0");
    } else if (in.dia_mm < 4.0 || in.dia_mm > 16.0) {
        r.add("DFM-COOL-DIA", "error",
              "cooling_channel dia " + std::to_string(in.dia_mm) +
              " mm outside [4.0, 16.0] mm typical range");
    }

    for (size_t i = 1; i < in.waypoints.size(); ++i) {
        const double L = dist(in.waypoints[i - 1], in.waypoints[i]);
        if (L < 1e-6) {
            r.add("DFM-INPUT", "error",
                  "cooling_channel segment " + std::to_string(i - 1) +
                  " has zero length");
        } else if (in.dia_mm > 0.0 && L < in.dia_mm) {
            r.add("DFM-COOL-LEN", "warning",
                  "cooling_channel segment " + std::to_string(i - 1) +
                  " length " + std::to_string(L) +
                  " < dia — too short to be useful");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "cooling_channel DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double radius = in.dia_mm / 2.0;

    // Build one cylinder per polyline segment.  Per Constraints: "For multi-
    // cylinder cuts: fuse first then cut" — cutMany packs all cutter cylinders
    // into a single compound and performs ONE BRepAlgoAPI_Cut, which is the
    // OCCT-blessed way to fuse-then-cut.
    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(in.waypoints.size() - 1);
    double totalLength = 0.0;
    for (size_t i = 1; i < in.waypoints.size(); ++i) {
        const Waypoint& a = in.waypoints[i - 1];
        const Waypoint& b = in.waypoints[i];
        const gp_Pnt p0(a.x_mm, a.y_mm, a.z_mm);
        const gp_Pnt p1(b.x_mm, b.y_mm, b.z_mm);
        gp_Vec v(p0, p1);
        const double L = v.Magnitude();
        if (L < 1e-9) continue;
        totalLength += L;
        v.Normalize();
        const gp_Dir dir(v.X(), v.Y(), v.Z());
        // gp_Ax2 V = local Z: cylinder extrudes along `dir` for length L.
        const gp_Ax2 ax(p0, dir);
        cutters.push_back(pr::cylinder(ax, radius, L));
    }
    if (cutters.empty()) {
        throw SkillError("cooling_channel: all segments degenerate");
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    json wpArr = json::array();
    for (const auto& w : in.waypoints) {
        wpArr.push_back({ { "x_mm", w.x_mm }, { "y_mm", w.y_mm }, { "z_mm", w.z_mm } });
    }

    json params = {
        { "dia_mm",    in.dia_mm },
        { "waypoints", wpArr },
    };
    json pattern = {
        { "kind",            kSkillId },
        { "dia_mm",          in.dia_mm },
        { "waypoint_count",  static_cast<int>(in.waypoints.size()) },
        { "segment_count",   static_cast<int>(cutters.size()) },
        { "total_length_mm", totalLength },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "gun_drill";
    tooling.tool_dia_mm      = in.dia_mm;
    tooling.stock_removed_mm3 = M_PI * radius * radius * totalLength;
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::cooling_channel applied: waypoints={} dia={} faces {}→{}",
                  in.waypoints.size(), in.dia_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "dia_mm",    f.params.value("dia_mm",    0.0) },
            { "waypoints", f.params.value("waypoints", json::array()) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.95, json{ { "source", "feature_history" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::cooling_channel
