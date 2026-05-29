// @lat: [[engine/skills#intermittent_fillet_pattern]]

#include "intermittent_fillet_pattern.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <vector>

namespace koocadcam::skill::intermittent_fillet_pattern {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.segment_count < 2) {
        r.add("DFM-WELD-IF-COUNT", "error",
              "intermittent_fillet_pattern: segment_count " +
              std::to_string(in.segment_count) +
              " < 2 (not an intermittent pattern)");
    }
    if (in.segment_length < 25.0) {
        r.add("DFM-WELD-IF-LEN", "error",
              "intermittent_fillet_pattern: segment_length " +
              std::to_string(in.segment_length) +
              " mm < 25 mm minimum (AWS D1.1)");
    }
    if (in.pitch < in.segment_length) {
        r.add("DFM-WELD-IF-PITCH", "error",
              "intermittent_fillet_pattern: pitch " +
              std::to_string(in.pitch) +
              " < segment_length " + std::to_string(in.segment_length) +
              " (segments would overlap)");
    }
    if (in.witness_depth_mm <= 0.0 || in.witness_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "intermittent_fillet_pattern: witness dimensions must be > 0");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "intermittent_fillet_pattern DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId)
        throw SkillError("intermittent_fillet_pattern: entry_face unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;

    const gp_Vec edgeVec(in.edge_dir.X(), in.edge_dir.Y(), 0.0);
    if (edgeVec.Magnitude() < 1e-6)
        throw SkillError("intermittent_fillet_pattern: edge_dir is vertical");
    const gp_Dir xLoc(edgeVec.X(), edgeVec.Y(), 0.0);
    const gp_Dir yLoc(-xLoc.Y(), xLoc.X(), 0.0);

    // Compute pattern start so the pattern is centered on edge_center.
    const double totalLen =
        (in.segment_count - 1) * in.pitch + in.segment_length;
    const double startX0 = in.edge_center_x_mm - xLoc.X() * totalLen / 2.0;
    const double startY0 = in.edge_center_y_mm - xLoc.Y() * totalLen / 2.0;

    // Build N segment cutters as small shallow boxes on the top face.
    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(in.segment_count);
    json segments = json::array();

    for (int i = 0; i < in.segment_count; ++i) {
        const double sStart = i * in.pitch;
        const double cx     = startX0 + xLoc.X() * sStart;
        const double cy     = startY0 + xLoc.Y() * sStart;

        // gp_Ax2(P, V=+Z, Vx=xLoc): DX = segment_length,
        //                            DY = witness_width,
        //                            DZ = witness_depth.
        const gp_Pnt boxOrigin(
            cx - yLoc.X() * in.witness_width_mm / 2.0,
            cy - yLoc.Y() * in.witness_width_mm / 2.0,
            topZ - in.witness_depth_mm);
        const gp_Ax2 ax(boxOrigin, gp::DZ(), xLoc);
        const TopoDS_Shape seg = pr::box(
            ax, in.segment_length, in.witness_width_mm,
            in.witness_depth_mm + 0.01);
        cutters.push_back(seg);

        segments.push_back({
            { "index",       i },
            { "start_x_mm",  cx },
            { "start_y_mm",  cy },
            { "end_x_mm",    cx + xLoc.X() * in.segment_length },
            { "end_y_mm",    cy + xLoc.Y() * in.segment_length },
        });
    }

    // Cut all in one compound op (Cuts.hpp::cutMany).
    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);

    json params = {
        { "entry_face_id",    *entryId },
        { "edge_dir",         { in.edge_dir.X(), in.edge_dir.Y(), in.edge_dir.Z() } },
        { "segment_count",    in.segment_count },
        { "segment_length",   in.segment_length },
        { "pitch",            in.pitch },
        { "edge_center_x_mm", in.edge_center_x_mm },
        { "edge_center_y_mm", in.edge_center_y_mm },
        { "witness_depth_mm", in.witness_depth_mm },
        { "witness_width_mm", in.witness_width_mm },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "is_compound",      true },
        { "subfeature_count", in.segment_count },
        { "segment_count",    in.segment_count },
        { "segment_length",   in.segment_length },
        { "pitch",            in.pitch },
        { "gap_mm",           in.pitch - in.segment_length },   // derived
        { "total_pattern_length_mm", totalLen },                // derived
        { "duty_cycle",       in.segment_length / in.pitch },   // derived
        { "edge_dir",         { in.edge_dir.X(), in.edge_dir.Y(), in.edge_dir.Z() } },
        { "segments",         segments },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "weld_torch";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_material     = "wire_feed";
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 =
        in.segment_count * in.segment_length *
        in.witness_width_mm * in.witness_depth_mm;
    tooling.est_cycle_time_s =
        in.segment_count * std::max(1.0, in.segment_length / 60.0);
    tooling.extra = {
        { "aws_classification", "intermittent_fillet" },
        { "aws_symbol",
          std::to_string(static_cast<int>(in.segment_length)) + " - " +
          std::to_string(static_cast<int>(in.pitch)) },
        { "note",
          "geometry change is witness-only (0.05 mm); welding itself is "
          "metadata-only" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::intermittent_fillet_pattern applied: N={} L={} P={}",
                  in.segment_count, in.segment_length, in.pitch);

    return SkillOutput{ wpNew, sig };
}

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& s : wp.features()) {
        if (s.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = s.params;
        r.confidence       = 1.0;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::intermittent_fillet_pattern
