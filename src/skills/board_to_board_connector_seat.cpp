// @lat: [[engine/skills#board_to_board_connector_seat]]

#include "board_to_board_connector_seat.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <string>
#include <vector>

namespace koocadcam::skill::board_to_board_connector_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "board_to_board_connector_seat: face_id out of range");
    }
    if (!(in.connector_length_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "board_to_board_connector_seat: connector_length_mm must be > 0");
    }
    if (!(in.connector_width_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "board_to_board_connector_seat: connector_width_mm must be > 0");
    }
    if (!(in.connector_height_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "board_to_board_connector_seat: connector_height_mm must be > 0");
    }
    if (in.keep_out_margin_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "board_to_board_connector_seat: keep_out_margin_mm must be ≥ 0");
    }
    // Slice-13 mounting-post option.
    if (in.mounting_post_count != 0 &&
        in.mounting_post_count != 2 &&
        in.mounting_post_count != 4) {
        r.add("DFM-INPUT", "error",
              "board_to_board_connector_seat: mounting_post_count must be 0, 2, or 4");
    }
    if (in.mounting_post_count > 0 && !(in.post_dia_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "board_to_board_connector_seat: post_dia_mm must be > 0 when "
              "mounting_post_count > 0");
    }
    if (in.mounting_post_count > 0 && in.post_dia_mm > 0.0 &&
        in.post_dia_mm < 0.3) {
        r.add("DFM-MIN-HOLE", "error",
              "board_to_board_connector_seat: post_dia_mm " +
              std::to_string(in.post_dia_mm) + " < 0.3 mm minimum");
    }
    // BTB envelope (typical phone middle-frame range)
    if (in.connector_length_mm > 0.0 &&
        (in.connector_length_mm < 8.0 || in.connector_length_mm > 25.0)) {
        r.add("DFM-BTB-RANGE", "warning",
              "BTB length " + std::to_string(in.connector_length_mm) +
              " outside typical phone range [8, 25] mm");
    }
    if (in.connector_width_mm > 0.0 &&
        (in.connector_width_mm < 1.5 || in.connector_width_mm > 2.5)) {
        r.add("DFM-BTB-RANGE", "warning",
              "BTB width " + std::to_string(in.connector_width_mm) +
              " outside typical phone range [1.5, 2.5] mm");
    }
    if (in.connector_height_mm > 0.0 &&
        (in.connector_height_mm < 0.6 || in.connector_height_mm > 1.2)) {
        r.add("DFM-BTB-RANGE", "warning",
              "BTB height " + std::to_string(in.connector_height_mm) +
              " outside typical phone range [0.6, 1.2] mm");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "board_to_board_connector_seat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const double baseZ = faceC.Z();

    const double L     = in.connector_length_mm;
    const double W     = in.connector_width_mm;
    const double H     = in.connector_height_mm;
    const double M     = in.keep_out_margin_mm;
    constexpr double kEmbed = 0.05;
    const double cx    = in.connector_position_x_mm;
    const double cy    = in.connector_position_y_mm;

    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape working = wp.shape();

    std::vector<TopoDS_Shape> cutters;

    // ── Op 1: central pocket (L × W × H) ────────────────────────────────
    {
        const gp_Pnt origin(cx - L / 2.0, cy - W / 2.0, baseZ - H);
        const gp_Ax2 ax(origin, gp::DZ());
        cutters.push_back(pr::box(ax, L, W, H + kEmbed));
    }

    // ── Op 2: keep-out chamfered slope ring (wider, shallow) ─────────────
    // A shallower & wider pocket that sits around the central pocket. We
    // realise the "chamfered slope" as a cone frustum bracket — depth = H/3.
    const double slopeDepth = std::max(0.05, H / 3.0);
    {
        const double Lo = L + 2.0 * M;
        const double Wo = W + 2.0 * M;
        const gp_Pnt origin(cx - Lo / 2.0, cy - Wo / 2.0, baseZ - slopeDepth);
        const gp_Ax2 ax(origin, gp::DZ());
        cutters.push_back(pr::box(ax, Lo, Wo, slopeDepth + kEmbed));
    }

    // ── Op 3: chamfered top edge — thin tapered ring (cone frustum), but
    //         since face_id may be planar we use a tiny wider/shallower box
    //         to simulate the chamfer top relief.
    const double chamW    = std::max(0.05, M * 0.5);
    const double chamDepth = std::max(0.05, slopeDepth * 0.3);
    {
        const double Lo = L + 2.0 * (M + chamW);
        const double Wo = W + 2.0 * (M + chamW);
        const gp_Pnt origin(cx - Lo / 2.0, cy - Wo / 2.0, baseZ - chamDepth);
        const gp_Ax2 ax(origin, gp::DZ());
        cutters.push_back(pr::box(ax, Lo, Wo, chamDepth + kEmbed));
    }

    TopoDS_Shape newShape = pr::cutMany(working, cutters);
    if (newShape.IsNull()) {
        throw SkillError("board_to_board_connector_seat: cutMany returned null");
    }

    // ── Slice-13: optional mounting posts (FUSE beside pocket) ───────────
    //
    // Posts are added AFTER the pocket cut so they remain solid even where
    // the keep-out ring crosses their footprint. Posts sit just OUTSIDE the
    // outer keep-out ring along the connector length axis (2-post) or both
    // axes (4-post), with height = H so the top sits flush with the face.
    int mountingPostCount = 0;
    double postVol = 0.0;
    if (in.mounting_post_count > 0 && in.post_dia_mm > 0.0) {
        const double rPost   = in.post_dia_mm * 0.5;
        // Post height = pocket height H, extruded UPWARD from the face so the
        // post is a real protrusion adding material (small PCB locating pin).
        const double postH   = H;
        constexpr double kEmbed2 = 0.05;
        const double zBot = baseZ - kEmbed2;        // sink 0.05 mm into face
        // Outer ring x extent (chamfered keep-out): L + 2*(M + chamW)
        const double xOff = L / 2.0 + M + chamW + rPost + 0.1;
        const double yOff = W / 2.0 + M + chamW + rPost + 0.1;
        std::vector<TopoDS_Shape> posts;
        // 2 posts: aligned along length axis (left + right of pocket)
        posts.push_back(pr::cylinder(
            gp_Ax2(gp_Pnt(cx - xOff, cy, zBot), gp::DZ()), rPost, postH + kEmbed2));
        posts.push_back(pr::cylinder(
            gp_Ax2(gp_Pnt(cx + xOff, cy, zBot), gp::DZ()), rPost, postH + kEmbed2));
        mountingPostCount = 2;
        if (in.mounting_post_count == 4) {
            posts.push_back(pr::cylinder(
                gp_Ax2(gp_Pnt(cx, cy - yOff, zBot), gp::DZ()), rPost, postH + kEmbed2));
            posts.push_back(pr::cylinder(
                gp_Ax2(gp_Pnt(cx, cy + yOff, zBot), gp::DZ()), rPost, postH + kEmbed2));
            mountingPostCount = 4;
        }
        for (const auto& p : posts) {
            newShape = pr::fuse(newShape, p);
            if (newShape.IsNull()) {
                throw SkillError("board_to_board_connector_seat: post fuse returned null");
            }
        }
        postVol = static_cast<double>(mountingPostCount) *
                  3.14159265358979323846 * rPost * rPost * postH;
    }

    const double volAfter   = volumeOf(newShape);
    const double volRemoved = volBefore - volAfter;

    json params = {
        { "face_id",                 in.face_id },
        { "connector_position_x_mm", in.connector_position_x_mm },
        { "connector_position_y_mm", in.connector_position_y_mm },
        { "connector_length_mm",     in.connector_length_mm },
        { "connector_width_mm",      in.connector_width_mm },
        { "connector_height_mm",     in.connector_height_mm },
        { "keep_out_margin_mm",      in.keep_out_margin_mm },
        { "mounting_post_count",     mountingPostCount },
        { "post_dia_mm",             in.post_dia_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "is_phone_feature",           true },
        { "phone_feature_type",         "board_to_board_connector_seat" },
        { "subfeature_count",           static_cast<int>(cutters.size()) +
                                        mountingPostCount },
        { "derived_volume_removed_mm3", volRemoved },
        { "connector_length_mm",        in.connector_length_mm },
        { "connector_width_mm",         in.connector_width_mm },
        { "connector_height_mm",        in.connector_height_mm },
        { "keep_out_margin_mm",         in.keep_out_margin_mm },
        { "mounting_post_count",        mountingPostCount },
        { "post_dia_mm",                in.post_dia_mm },
        { "post_added_volume_mm3",      postVol },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = std::max(0.5, in.connector_width_mm * 0.4);
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = volRemoved / 100.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::board_to_board_connector_seat: L={} W={} H={} vol-removed={}",
        L, W, H, volRemoved);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay (geometric recognition of a 3-tier nested rectangular
    // pocket would require pocket-clustering; we rely on metadata for now).
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.92;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::board_to_board_connector_seat
