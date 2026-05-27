// @lat: [[engine/skills#engrave_path]]
//
// ─── APPROXIMATION NOTE (slice 2) ────────────────────────────────────────
//
// A real engrave_path uses an offset wire derived from the waypoint
// polyline, extruded down by depth_mm, then cut from the workpiece.  Doing
// this cleanly requires:
//
//   MISSING PRIMITIVE:
//       prim::offsetPolylineCutter(
//           const std::vector<gp_Pnt2d>& waypoints,
//           double                       channel_width_mm,
//           double                       depth_mm,
//           const gp_Pln&                base_plane);   // -> TopoDS_Shape
//
//   Internally: BRepBuilderAPI_MakeWire from the polyline → BRepOffsetAPI_
//   MakeOffset for the channel width → BRepPrimAPI_MakePrism for the
//   depth.  Plus end-cap rounding to match a real end-mill footprint.
//
//   Until that primitive lands, engrave_path::apply() walks the waypoint
//   list and emits a chain of overlapping cylindrical "spot" cuts (one at
//   each waypoint, plus interior samples spaced at width_mm/2 along each
//   segment) — producing a connected groove without complex offsetting.
//
//   This sacrifices exact channel-edge fidelity but builds a valid solid
//   that records the engrave-path intent.  Swap for the offset-wire
//   primitive once it lands.
//
// ─────────────────────────────────────────────────────────────────────────

#include "engrave_path.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::engrave_path {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.width_mm < 0.15) {
        r.add("DFM-019", "error",
              "engrave_path width_mm " + std::to_string(in.width_mm) +
              " < 0.15 mm (min V-cutter)");
    }
    if (in.depth_mm < 0.05 || in.depth_mm > 0.5) {
        r.add("DFM-ENGRAVE-DEPTH", "error",
              "engrave_path depth_mm " + std::to_string(in.depth_mm) +
              " not in [0.05, 0.5]");
    }
    if (in.waypoints.size() < 2) {
        r.add("DFM-INPUT", "error",
              "engrave_path requires at least 2 waypoints (got " +
              std::to_string(in.waypoints.size()) + ")");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "engrave_path DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("engrave_path: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.02;
    const double radius    = in.width_mm / 2.0;
    const double cutterH   = in.depth_mm + kOverhang;
    const double stride    = std::max(radius, 0.05);  // overlap to keep solid connected

    auto addSpotAt = [&](std::vector<TopoDS_Shape>& sink, double x, double y) {
        const gp_Pnt top(x, y, zMax + kOverhang);
        const gp_Ax2 ax(top, gp_Dir(0.0, 0.0, -1.0));
        sink.push_back(pr::cylinder(ax, radius, cutterH));
    };

    std::vector<TopoDS_Shape> spots;
    // Spot at every waypoint, plus interpolated samples on each segment.
    for (size_t i = 0; i < in.waypoints.size(); ++i) {
        const double x0 = in.waypoints[i][0];
        const double y0 = in.waypoints[i][1];
        addSpotAt(spots, x0, y0);

        if (i + 1 < in.waypoints.size()) {
            const double x1 = in.waypoints[i + 1][0];
            const double y1 = in.waypoints[i + 1][1];
            const double segLen = std::hypot(x1 - x0, y1 - y0);
            if (segLen > stride) {
                const int n = static_cast<int>(std::floor(segLen / stride));
                for (int k = 1; k <= n; ++k) {
                    const double t = static_cast<double>(k) / (n + 1);
                    addSpotAt(spots, x0 + t * (x1 - x0), y0 + t * (y1 - y0));
                }
            }
        }
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), spots);

    // Serialize waypoints as a JSON array of [x, y] pairs.
    json wpJson = json::array();
    for (const auto& w : in.waypoints) wpJson.push_back({ w[0], w[1] });

    json params = {
        { "entry_face_id", *entryId },
        { "waypoints",     wpJson },
        { "width_mm",      in.width_mm },
        { "depth_mm",      in.depth_mm },
    };
    json pattern = {
        { "kind",           kSkillId },
        { "waypoint_count", in.waypoints.size() },
        { "width_mm",       in.width_mm },
        { "depth_mm",       in.depth_mm },
        { "geometry",       "spot_chain_approximation" },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "v_cutter";
    tooling.tool_dia_mm       = in.width_mm;
    tooling.tool_length_mm    = std::max(10.0, in.depth_mm * 5.0 + 5.0);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 800.0;
    tooling.feed_per_tooth_mm = 0.01;

    // Estimate path length for cycle time / removed volume.
    double pathLen = 0.0;
    for (size_t i = 1; i < in.waypoints.size(); ++i) {
        pathLen += std::hypot(in.waypoints[i][0] - in.waypoints[i - 1][0],
                              in.waypoints[i][1] - in.waypoints[i - 1][1]);
    }
    tooling.stock_removed_mm3 = pathLen * in.width_mm * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(1.0, pathLen * 0.05);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::engrave_path applied: waypoints={} pathLen={} depth={}",
                  in.waypoints.size(), pathLen, in.depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Connected-channel recognition is hard without explicit metadata.
// Approach:
//   (a) Metadata replay — replay signatures in workpiece.features().
//   (b) Without metadata: return empty (the spot chain is hard to
//       distinguish from drill_hole patterns).

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

}  // namespace koocadcam::skill::engrave_path
