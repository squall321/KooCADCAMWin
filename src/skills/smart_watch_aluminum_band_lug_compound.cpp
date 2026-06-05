// @lat: [[engine/skills#smart_watch_aluminum_band_lug_compound]]

#include "smart_watch_aluminum_band_lug_compound.hpp"

#include "Workpiece.hpp"
#include "crown_stem_cavity_compound.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp_Ax1.hxx>
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

namespace koocadcam::skill::smart_watch_aluminum_band_lug_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;

    if (in.case_w_mm <= 0.0 || in.case_h_mm <= 0.0 || in.case_d_mm <= 0.0 ||
        in.band_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "smart_watch_aluminum_band_lug_compound: dimensions must be > 0");
        return r;
    }
    if (in.case_w_mm < 30.0 || in.case_w_mm > 60.0) {
        r.add("DFM-CASE-DIA", "error",
              "smart_watch_aluminum_band_lug_compound: case_w " +
              std::to_string(in.case_w_mm) +
              " mm outside [30, 60] mm");
    }
    if (in.case_d_mm < 6.0 || in.case_d_mm > 15.0) {
        r.add("DFM-CASE-DEPTH", "error",
              "smart_watch_aluminum_band_lug_compound: case_d " +
              std::to_string(in.case_d_mm) +
              " mm outside [6, 15] mm");
    }
    if (in.band_lug_type != "spring_bar" &&
        in.band_lug_type != "slide_in" &&
        in.band_lug_type != "magnetic_clip") {
        r.add("DFM-BAND-LUG", "error",
              "smart_watch_aluminum_band_lug_compound: band_lug_type must be spring_bar | slide_in | magnetic_clip");
    }
    if (in.band_width_mm != 20.0 && in.band_width_mm != 22.0) {
        r.add("DFM-BAND-WIDTH", "error",
              "smart_watch_aluminum_band_lug_compound: band_width must be 20 or 22 mm");
    }
    if (in.band_width_mm > in.case_w_mm - 4.0) {
        r.add("DFM-BAND-FIT", "error",
              "smart_watch_aluminum_band_lug_compound: band_width must be <= case_w - 4 mm");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "smart_watch_aluminum_band_lug_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double W = in.case_w_mm;
    const double H = in.case_h_mm;
    const double D = in.case_d_mm;
    // Case axis "case_axis" = vertical (Z), centered on rectangle center.
    const gp_Ax1 caseAxis(gp_Pnt(W / 2.0, H / 2.0, 0.0), gp_Dir(0, 0, 1));

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    constexpr double kOverhang = 0.05;

    std::shared_ptr<Workpiece> current =
        std::make_shared<Workpiece>(wp.shape(), wp.material());

    // ── Stage 2: round 4 corners (approximate via small square cuts) ─────
    {
        const double cornerCut = 2.5;
        std::vector<TopoDS_Shape> cuts;
        cuts.reserve(4);
        const double corners[4][2] = {
            { xMin - kOverhang,         yMin - kOverhang },
            { xMax - cornerCut,         yMin - kOverhang },
            { xMin - kOverhang,         yMax - cornerCut },
            { xMax - cornerCut,         yMax - cornerCut },
        };
        for (int i = 0; i < 4; ++i) {
            const gp_Pnt o(corners[i][0], corners[i][1], zMin - kOverhang);
            const gp_Ax2 ax(o, gp_Dir(0, 0, 1));
            cuts.push_back(pr::box(
                ax,
                cornerCut + kOverhang,
                cornerCut + kOverhang,
                D + 2.0 * kOverhang));
        }
        const auto newShape = pr::cutMany(current->shape(), cuts);
        current = std::make_shared<Workpiece>(newShape, wp.material());
    }

    // ── Stage 3: band lug attachment (top + bottom of case) ──────────────
    {
        const double bandW = in.band_width_mm;
        std::vector<TopoDS_Shape> cutters;

        if (in.band_lug_type == "spring_bar") {
            // 2 transverse spring-bar holes per side (4 total).
            const double barR = 0.8;   // 1.6 mm dia
            // Side Y=yMax (top): hole through case along X axis.
            const double zCenter = D / 2.0;
            const double xCenterL = (W - bandW) / 2.0 + 1.5;
            const double xCenterR = (W + bandW) / 2.0 - 1.5;
            // Top side
            for (double xc : { xCenterL, xCenterR }) {
                const gp_Pnt s(xc, yMax + 0.2, zCenter);
                const gp_Ax2 ax(s, gp_Dir(0, -1, 0));
                cutters.push_back(pr::cylinder(ax, barR, 4.0));
            }
            // Bottom side
            for (double xc : { xCenterL, xCenterR }) {
                const gp_Pnt s(xc, yMin - 4.0 + 0.2, zCenter);
                const gp_Ax2 ax(s, gp_Dir(0, 1, 0));
                cutters.push_back(pr::cylinder(ax, barR, 4.0));
            }
        } else if (in.band_lug_type == "slide_in") {
            // Slot pocket per side (front face of case at +Y / -Y).
            const double slotDepth = 2.0;
            const double slotHeight = 2.5;
            // Top: pocket cut into front-Y wall.
            const gp_Pnt sTop((W - bandW) / 2.0,
                              yMax - slotDepth + kOverhang,
                              (D - slotHeight) / 2.0);
            const gp_Ax2 axTop(sTop, gp_Dir(0, 0, 1));
            cutters.push_back(pr::box(axTop, bandW, slotDepth + kOverhang, slotHeight));
            // Bottom
            const gp_Pnt sBot((W - bandW) / 2.0,
                              yMin - kOverhang,
                              (D - slotHeight) / 2.0);
            const gp_Ax2 axBot(sBot, gp_Dir(0, 0, 1));
            cutters.push_back(pr::box(axBot, bandW, slotDepth + kOverhang, slotHeight));
        } else {  // magnetic_clip
            // Shallow rectangular pocket per side for magnetic clip.
            const double clipDepth = 1.0;
            const double clipHeight = 4.0;
            // Top
            const gp_Pnt sTop((W - bandW) / 2.0,
                              yMax - clipDepth + kOverhang,
                              (D - clipHeight) / 2.0);
            const gp_Ax2 axTop(sTop, gp_Dir(0, 0, 1));
            cutters.push_back(pr::box(axTop, bandW, clipDepth + kOverhang, clipHeight));
            // Bottom
            const gp_Pnt sBot((W - bandW) / 2.0,
                              yMin - kOverhang,
                              (D - clipHeight) / 2.0);
            const gp_Ax2 axBot(sBot, gp_Dir(0, 0, 1));
            cutters.push_back(pr::box(axBot, bandW, clipDepth + kOverhang, clipHeight));
        }

        const auto newShape = pr::cutMany(current->shape(), cutters);
        current = std::make_shared<Workpiece>(newShape, wp.material());
    }

    // ── Stage 4: display window pocket on top face ───────────────────────
    {
        const double dispW = W - 6.0;
        const double dispH = H - 8.0;
        const double dispDepth = 1.0;
        const gp_Pnt origin((W - dispW) / 2.0, (H - dispH) / 2.0,
                             zMax - dispDepth + kOverhang);
        const gp_Ax2 ax(origin, gp_Dir(0, 0, 1));
        const TopoDS_Shape pocket = pr::box(
            ax, dispW, dispH, dispDepth + kOverhang);
        const auto newShape = pr::cut(current->shape(), pocket);
        current = std::make_shared<Workpiece>(newShape, wp.material());
    }

    // ── Stage 5: digital crown cavity at 3 o'clock (+X side) ─────────────
    // For rectangular case we drill directly rather than reuse the radial
    // crown skill (which assumes cylindrical case).
    {
        const double crownR = 1.5;
        const gp_Pnt s(xMax + 0.2, H / 2.0, D / 2.0);
        const gp_Ax2 ax(s, gp_Dir(-1, 0, 0));
        const TopoDS_Shape crown = pr::cylinder(ax, crownR, 3.5);
        const auto newShape = pr::cut(current->shape(), crown);
        current = std::make_shared<Workpiece>(newShape, wp.material());
    }

    // ── Stage 6: 2 microphone/speaker grilles ────────────────────────────
    {
        // Cut 2 small holes on -X side (9 o'clock) at slight offsets.
        const double grillR = 0.5;
        std::vector<TopoDS_Shape> grills;
        grills.reserve(2);
        for (int i = 0; i < 2; ++i) {
            const double dz = (i == 0) ? -2.0 : 2.0;
            const gp_Pnt s(xMin - 0.2, H / 2.0, D / 2.0 + dz);
            const gp_Ax2 ax(s, gp_Dir(1, 0, 0));
            grills.push_back(pr::cylinder(ax, grillR, 2.0));
        }
        const auto newShape = pr::cutMany(current->shape(), grills);
        current = std::make_shared<Workpiece>(newShape, wp.material());
    }

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_w_mm",      in.case_w_mm },
        { "case_h_mm",      in.case_h_mm },
        { "case_d_mm",      in.case_d_mm },
        { "band_lug_type",  in.band_lug_type },
        { "band_width_mm",  in.band_width_mm },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "is_compound",        true },
        { "is_watch_assembly",  true },
        { "watch_style",        "smart_aluminum" },
        { "subfeature_count",   6 },
        { "subfeatures", json::array({
            { { "op", "corner_round_cut" },           { "count", 4 } },
            { { "op", "band_lug_attachment" },        { "type", in.band_lug_type } },
            { { "op", "display_window_pocket" } },
            { { "op", "digital_crown_cavity" },       { "angle_deg", 0.0 } },
            { { "op", "mic_speaker_grilles" },        { "count", 2 } },
        }) },
        { "case_w_mm",         in.case_w_mm },
        { "case_h_mm",         in.case_h_mm },
        { "case_d_mm",         in.case_d_mm },
        { "band_lug_type",     in.band_lug_type },
        { "band_width_mm",     in.band_width_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "assembly_chain";
    tooling.tool_dia_mm       = std::max(W, H);
    tooling.tool_length_mm    = D;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 280.0;     // Al6061 — high SFM
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = W * H * D * 0.15;
    tooling.est_cycle_time_s  = 90.0;
    tooling.extra = {
        { "feature_standard", "Apple Watch / Galaxy Watch style aluminum case" },
        { "sub_skill_chain", json::array({
            "corner_round_cut",
            "band_lug_attachment",
            "display_window_pocket",
            "digital_crown_cavity",
            "mic_speaker_grilles",
        }) },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current->shape(), wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::smart_watch_aluminum_band_lug_compound applied: W={} H={} D={} lug={} band={}",
        in.case_w_mm, in.case_h_mm, in.case_d_mm,
        in.band_lug_type, in.band_width_mm);

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
            { "watch_style",       "smart_aluminum" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::smart_watch_aluminum_band_lug_compound
