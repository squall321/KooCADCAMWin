// @lat: [[engine/skills#gmt_chronograph_case_compound]]

#include "gmt_chronograph_case_compound.hpp"

#include "Workpiece.hpp"
#include "case_back_screw_down.hpp"
#include "crown_stem_cavity_compound.hpp"
#include "diving_bezel_unidirectional.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::gmt_chronograph_case_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;

    if (in.case_dia_mm <= 0.0 || in.case_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gmt_chronograph_case_compound: dimensions must be > 0");
        return r;
    }
    if (in.case_dia_mm < 38.0 || in.case_dia_mm > 48.0) {
        r.add("DFM-CASE-DIA", "error",
              "gmt_chronograph_case_compound: case_dia " +
              std::to_string(in.case_dia_mm) +
              " mm outside GMT chrono typical [38, 48] mm");
    }
    if (in.case_height_mm < 10.0 || in.case_height_mm > 17.0) {
        r.add("DFM-CASE-HEIGHT", "error",
              "gmt_chronograph_case_compound: case_height " +
              std::to_string(in.case_height_mm) +
              " mm outside [10, 17] mm");
    }
    if (in.subdial_count != 3) {
        r.add("DFM-SUBDIAL", "error",
              "gmt_chronograph_case_compound: subdial_count must be 3");
    }
    if (in.pusher_count != 2) {
        r.add("DFM-PUSHER", "error",
              "gmt_chronograph_case_compound: pusher_count must be 2");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gmt_chronograph_case_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double caseR = in.case_dia_mm / 2.0;
    const double caseH = in.case_height_mm;
    const gp_Ax1 caseAxis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;
    constexpr double kOverhang = 0.05;

    // Stage 1: stock = wp; we'll cut sub-dial pockets ON the top face.
    std::shared_ptr<Workpiece> current =
        std::make_shared<Workpiece>(wp.shape(), wp.material());

    // ── Stage 2: cut 3 sub-dial pockets at 3/6/9 o'clock on top face ─────
    {
        const double subdialR = caseR * 0.18;          // small dials
        const double subdialDepth = 1.0;
        const double pocketCircleR = caseR * 0.45;     // distance from center
        // 3 angular positions: 30° (≈ 1 o'clock area for chrono), 270° (6), 210°.
        // Classic 3-register: 6, 9, 12 → angles 270°, 180°, 90°.
        // We use 90°, 270°, 0° (top, bottom, right of dial face) for variety.
        const double anglesDeg[3] = { 90.0, 270.0, 0.0 };
        std::vector<TopoDS_Shape> pockets;
        pockets.reserve(static_cast<size_t>(in.subdial_count));
        for (int i = 0; i < in.subdial_count; ++i) {
            const double a = anglesDeg[i] * M_PI / 180.0;
            const gp_Pnt p(pocketCircleR * std::cos(a),
                           pocketCircleR * std::sin(a),
                           topZ + kOverhang);
            const gp_Ax2 ax(p, gp_Dir(0, 0, -1));
            pockets.push_back(pr::cylinder(ax, subdialR, subdialDepth + kOverhang));
        }
        const auto newShape = pr::cutMany(current->shape(), pockets);
        current = std::make_shared<Workpiece>(newShape, wp.material());
    }

    // ── Stage 3: crown stem at 3 o'clock ─────────────────────────────────
    {
        crown_stem_cavity_compound::Input csIn;
        csIn.case_axis            = caseAxis;
        csIn.case_outer_radius_mm = caseR;
        csIn.angle_deg            = 0.0;        // 3 o'clock
        csIn.port_center_z_mm     = caseH / 2.0;
        csIn.cavity_dia_mm        = 3.5;
        csIn.cavity_depth_mm      = 2.5;
        csIn.stem_dia_mm          = 1.5;
        csIn.stem_total_length_mm = 8.0;
        csIn.o_ring_cs_mm         = 1.0;
        csIn.o_ring_depth_mm      = 0.4;
        csIn.o_ring_offset_mm     = 1.0;
        auto out = crown_stem_cavity_compound::apply(*current, csIn);
        current = out.workpiece;
    }

    // ── Stage 4: 2 chronograph pushers at 2 and 4 o'clock ────────────────
    {
        // 2 o'clock = 60° azimuth, 4 o'clock = -60° (= 300°).
        const double pusherAnglesDeg[2] = { 60.0, 300.0 };
        const double pusherR = 0.9;             // 1.8 mm diameter
        const double pusherDepth = 3.0;
        std::vector<TopoDS_Shape> pusherHoles;
        pusherHoles.reserve(static_cast<size_t>(in.pusher_count));
        for (int i = 0; i < in.pusher_count; ++i) {
            const double a = pusherAnglesDeg[i] * M_PI / 180.0;
            const gp_Dir radialOut(std::cos(a), std::sin(a), 0.0);
            const gp_Dir radialIn(-radialOut.X(), -radialOut.Y(), 0.0);
            const gp_Pnt start(
                (caseR + 0.2) * radialOut.X(),
                (caseR + 0.2) * radialOut.Y(),
                caseH / 2.0);
            const gp_Ax2 ax(start, radialIn);
            pusherHoles.push_back(pr::cylinder(ax, pusherR, pusherDepth));
        }
        const auto newShape = pr::cutMany(current->shape(), pusherHoles);
        current = std::make_shared<Workpiece>(newShape, wp.material());
    }

    // ── Stage 5: GMT 24h bezel seat (60-click, GMT-style) ────────────────
    {
        diving_bezel_unidirectional::Input bzIn;
        bzIn.case_top_face      = FaceByNormal{ gp_Dir(0, 0, 1) };
        bzIn.case_axis          = caseAxis;
        bzIn.bezel_outer_dia_mm = in.case_dia_mm - 1.0;
        bzIn.bezel_inner_dia_mm = in.case_dia_mm - 6.0;
        bzIn.bezel_height_mm    = 1.5;
        bzIn.click_count        = 60;   // GMT 24h: 1 click/min
        bzIn.click_depth_mm     = 0.4;
        bzIn.click_notch_dia_mm = 0.5;
        auto out = diving_bezel_unidirectional::apply(*current, bzIn);
        current = out.workpiece;
    }

    // ── Stage 6: caseback ────────────────────────────────────────────────
    {
        case_back_screw_down::Input cbIn;
        cbIn.case_bottom_face       = FaceByNormal{ gp_Dir(0, 0, -1) };
        cbIn.case_axis              = caseAxis;
        cbIn.back_dia_mm            = in.case_dia_mm - 4.0;
        cbIn.thread_pitch_mm        = 0.6;
        cbIn.thread_depth_mm        = 0.5;
        cbIn.thread_band_axial_mm   = 3.5;  // > -116 CS(2.62)+0.3=2.92, margin 0.58
        cbIn.o_ring_groove_position = "external";
        cbIn.o_ring_size_key        = "-116";
        auto out = case_back_screw_down::apply(*current, cbIn);
        current = out.workpiece;
    }

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_dia_mm",     in.case_dia_mm },
        { "case_height_mm",  in.case_height_mm },
        { "subdial_count",   in.subdial_count },
        { "pusher_count",    in.pusher_count },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "is_compound",        true },
        { "is_watch_assembly",  true },
        { "watch_style",        "gmt_chronograph" },
        { "subfeature_count",   7 },
        { "subfeatures", json::array({
            { { "op", "subdial_pockets" },                { "count", in.subdial_count } },
            { { "op", "crown_stem_cavity_compound" },     { "angle_deg", 0.0 } },
            { { "op", "chrono_pushers" },                 { "count", in.pusher_count } },
            { { "op", "diving_bezel_unidirectional" },    { "click_count", 60 }, { "scale", "GMT 24h" } },
            { { "op", "case_back_screw_down" },           { "as568_dash", "-116" } },
        }) },
        { "case_dia_mm",        in.case_dia_mm },
        { "case_height_mm",     in.case_height_mm },
        { "subdial_count",      in.subdial_count },
        { "pusher_count",       in.pusher_count },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "assembly_chain";
    tooling.tool_dia_mm       = in.case_dia_mm;
    tooling.tool_length_mm    = in.case_height_mm;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 220.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = M_PI * caseR * caseR * caseH * 0.25;
    tooling.est_cycle_time_s  = 140.0;
    tooling.extra = {
        { "feature_standard", "GMT chronograph case (3 sub-dials + 2 pushers)" },
        { "sub_skill_chain", json::array({
            "subdial_pockets",
            "crown_stem_cavity_compound",
            "chrono_pushers",
            "diving_bezel_unidirectional",
            "case_back_screw_down",
        }) },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current->shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::gmt_chronograph_case_compound applied: dia={} h={} subdials={} pushers={}",
        in.case_dia_mm, in.case_height_mm, in.subdial_count, in.pusher_count);

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
        r.confidence       = 0.5;
        r.matched_geometry = {
            { "source",            "metadata_replay" },
            { "is_compound",       true },
            { "is_watch_assembly", true },
            { "watch_style",       "gmt_chronograph" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::gmt_chronograph_case_compound
