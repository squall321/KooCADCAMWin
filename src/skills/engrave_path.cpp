// @lat: [[engine/skills#engrave_path]]
//
// engrave_path — engrave along an arbitrary 2D polyline on a face.
//
// Slice 4: the body now delegates to `prim::offsetPolylineCutter`, which
// builds a stadium-chain cutter (cylinder + box + cylinder per segment)
// and fuses them into a single solid.  This replaces the inline overlapping
// cylinder chain that previously lived in this file.

#include "engrave_path.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/PolylineOffset.hpp"

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

    // Build the channel cutter via the primitive.  The cutter's top sits
    // slightly above the workpiece top (by kOverhang) so the Boolean cut
    // produces a clean entry; total cutter depth = depth_mm + kOverhang.
    const TopoDS_Shape cutter = pr::offsetPolylineCutter(
        in.waypoints,
        in.width_mm,
        in.depth_mm + kOverhang,
        /*base_z=*/zMax + kOverhang);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

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
        { "geometry",       "polyline_offset_cutter" },
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
// Connected-channel recognition without metadata is hard (would require
// detecting a chain of overlapping cylinder/box faces and reconstructing
// the polyline).  We rely on metadata replay; STEP round-trip workpieces
// without history return an empty result.

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
