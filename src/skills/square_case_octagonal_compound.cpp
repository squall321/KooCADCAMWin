// @lat: [[engine/skills#square_case_octagonal_compound]]

#include "square_case_octagonal_compound.hpp"

#include "Workpiece.hpp"
#include "case_back_screw_down.hpp"
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

namespace koocadcam::skill::square_case_octagonal_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    (void)wp;
    DFMReport r;

    if (in.case_width_mm <= 0.0 || in.case_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "square_case_octagonal_compound: dimensions must be > 0");
        return r;
    }
    if (in.case_width_mm < 30.0 || in.case_width_mm > 60.0) {
        r.add("DFM-CASE-DIA", "error",
              "square_case_octagonal_compound: case_width " +
              std::to_string(in.case_width_mm) +
              " mm outside [30, 60] mm typical");
    }
    if (in.case_height_mm < 7.0 || in.case_height_mm > 18.0) {
        r.add("DFM-CASE-HEIGHT", "error",
              "square_case_octagonal_compound: case_height " +
              std::to_string(in.case_height_mm) +
              " mm outside [7, 18] mm");
    }
    if (in.lug_count != 8) {
        r.add("DFM-LUG-COUNT", "error",
              "square_case_octagonal_compound: lug_count must be 8 (octagonal sport)");
    }
    if (in.screwdown_count != 8) {
        r.add("DFM-SCREW-COUNT", "error",
              "square_case_octagonal_compound: screwdown_count must be 8");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "square_case_octagonal_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double W = in.case_width_mm;
    const double H = in.case_height_mm;
    const gp_Ax1 caseAxis(gp_Pnt(W / 2.0, W / 2.0, 0.0), gp_Dir(0, 0, 1));

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    constexpr double kOverhang = 0.05;

    std::shared_ptr<Workpiece> current =
        std::make_shared<Workpiece>(wp.shape(), wp.material());

    // ── Stage 2: octagonal corner trim — 4 corner prisms cut away ────────
    // Each corner: a right-triangular vertical prism, hypotenuse = chamfer
    // chord.  Use a vertical box ROTATED 45° at each corner.  Approach:
    // cut 4 small cubes from corners (axis-aligned, sliced visually like
    // an octagonal bevel).  For analytical simplicity we cut axis-aligned
    // square prisms at the 4 corners, each ~12% of the width — produces
    // a chamfered (octagonal-ish) silhouette.
    {
        const double cornerSize = W * 0.13;
        std::vector<TopoDS_Shape> cornerCuts;
        cornerCuts.reserve(4);
        // 4 corner positions in the bounding box (assume stock placed at origin):
        const double corners[4][2] = {
            { xMin - kOverhang,                 yMin - kOverhang },
            { xMax - cornerSize,                yMin - kOverhang },
            { xMin - kOverhang,                 yMax - cornerSize },
            { xMax - cornerSize,                yMax - cornerSize },
        };
        for (int i = 0; i < 4; ++i) {
            const gp_Pnt origin(corners[i][0], corners[i][1], zMin - kOverhang);
            const gp_Ax2 ax(origin, gp_Dir(0, 0, 1));
            cornerCuts.push_back(pr::box(
                ax,
                cornerSize + kOverhang,
                cornerSize + kOverhang,
                H + 2.0 * kOverhang));
        }
        const auto newShape = pr::cutMany(current->shape(), cornerCuts);
        current = std::make_shared<Workpiece>(newShape, wp.material());
    }

    // ── Stage 3: fuse 4 lug bars at 12/3/6/9 o'clock ─────────────────────
    {
        const double lugLen = 6.0;
        const double lugW   = 8.0;
        const double lugZBot = (H - 2.0) / 2.0;
        std::vector<TopoDS_Shape> lugs;
        lugs.reserve(4);
        // 12 o'clock: +Y side; 6: -Y side; 3: +X; 9: -X.
        // 12: lug extends up beyond ymax.
        {
            const gp_Pnt o(W / 2.0 - lugW / 2.0, yMax - 0.05, lugZBot);
            const gp_Ax2 ax(o, gp_Dir(0, 0, 1));
            lugs.push_back(pr::box(ax, lugW, lugLen + 0.05, 2.0));
        }
        // 6 o'clock: lug extends down (-Y).
        {
            const gp_Pnt o(W / 2.0 - lugW / 2.0, yMin - lugLen, lugZBot);
            const gp_Ax2 ax(o, gp_Dir(0, 0, 1));
            lugs.push_back(pr::box(ax, lugW, lugLen + 0.05, 2.0));
        }
        // 3 o'clock: +X
        {
            const gp_Pnt o(xMax - 0.05, W / 2.0 - lugW / 2.0, lugZBot);
            const gp_Ax2 ax(o, gp_Dir(0, 0, 1));
            lugs.push_back(pr::box(ax, lugLen + 0.05, lugW, 2.0));
        }
        // 9 o'clock: -X
        {
            const gp_Pnt o(xMin - lugLen, W / 2.0 - lugW / 2.0, lugZBot);
            const gp_Ax2 ax(o, gp_Dir(0, 0, 1));
            lugs.push_back(pr::box(ax, lugLen + 0.05, lugW, 2.0));
        }
        const auto newShape = pr::fuseMany(current->shape(), lugs);
        current = std::make_shared<Workpiece>(newShape, wp.material());
    }

    // ── Stage 4: 8 screwdown bolt holes around the bezel ─────────────────
    {
        const double bezelR = W * 0.40;
        const double cX = W / 2.0;
        const double cY = W / 2.0;
        std::vector<TopoDS_Shape> bolts;
        bolts.reserve(static_cast<size_t>(in.screwdown_count));
        for (int i = 0; i < in.screwdown_count; ++i) {
            const double a = (i * 360.0 / in.screwdown_count) * M_PI / 180.0;
            const gp_Pnt p(cX + bezelR * std::cos(a),
                           cY + bezelR * std::sin(a),
                           zMax + kOverhang);
            const gp_Ax2 ax(p, gp_Dir(0, 0, -1));
            bolts.push_back(pr::cylinder(ax, 0.6, 1.5 + kOverhang));
        }
        const auto newShape = pr::cutMany(current->shape(), bolts);
        current = std::make_shared<Workpiece>(newShape, wp.material());
    }

    // ── Stage 5: caseback ────────────────────────────────────────────────
    {
        case_back_screw_down::Input cbIn;
        cbIn.case_bottom_face       = FaceByNormal{ gp_Dir(0, 0, -1) };
        cbIn.case_axis              = caseAxis;
        // Caseback OD ~62 % of the across-flats width (26.0 mm on a 42 mm case
        // — an authentic Royal-Oak/Nautilus-scale screw-down back, sitting well
        // inside the ~42 mm octagonal body, DFM-BACK-FIT clearance ≫ 0.3 mm).
        cbIn.back_dia_mm            = W * 0.62;
        cbIn.thread_pitch_mm        = 0.6;
        // 1.1 mm cosmetic thread-band relief: pulls the flat face-seal groove
        // inboard so its outer Ø clears the caseback edge.  The flat groove
        // mean Ø = back_dia − 2·(thread_depth + 1.0); a deeper band shrinks the
        // mean Ø and buys radial edge clearance for the O-ring gland.
        cbIn.thread_depth_mm        = 1.1;
        cbIn.thread_band_axial_mm   = 1.8;
        cbIn.o_ring_groove_position = "flat";
        // AS568 -016 (CS 1.78 mm, ID 15.54 mm) — a real small face-seal O-ring
        // that fits concentric in a 26 mm caseback.  Its groove width (2.49 mm)
        // is the narrowest standard gland, maximising edge clearance vs. the
        // wider -116 (3.67 mm) that previously overran the caseback rim.
        cbIn.o_ring_size_key        = "-016";
        auto out = case_back_screw_down::apply(*current, cbIn);
        current = out.workpiece;
    }

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "case_width_mm",    in.case_width_mm },
        { "case_height_mm",   in.case_height_mm },
        { "lug_count",        in.lug_count },
        { "screwdown_count",  in.screwdown_count },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "is_compound",        true },
        { "is_watch_assembly",  true },
        { "watch_style",        "square_octagonal" },
        { "subfeature_count",   5 },
        { "subfeatures", json::array({
            { { "op", "octagonal_corner_trim" }, { "corners", 4 } },
            { { "op", "lug_bar_fuse_4way" },     { "count", 4 } },
            { { "op", "screwdown_bezel_array" }, { "count", in.screwdown_count } },
            { { "op", "case_back_screw_down" },  { "as568_dash", "-016" } },
        }) },
        { "case_width_mm",      in.case_width_mm },
        { "case_height_mm",     in.case_height_mm },
        { "lug_count",          in.lug_count },
        { "screwdown_count",    in.screwdown_count },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "assembly_chain";
    tooling.tool_dia_mm       = W;
    tooling.tool_length_mm    = H;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 230.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = W * W * H * 0.2;
    tooling.est_cycle_time_s  = 110.0;
    tooling.extra = {
        { "feature_standard", "Royal Oak / Nautilus square octagonal sport case" },
        { "sub_skill_chain", json::array({
            "octagonal_corner_trim",
            "lug_bar_fuse_4way",
            "screwdown_bezel_array",
            "case_back_screw_down",
        }) },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current->shape(), wp.material());
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::square_case_octagonal_compound applied: W={} H={} lugs={} screws={}",
        in.case_width_mm, in.case_height_mm, in.lug_count, in.screwdown_count);

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
            { "watch_style",       "square_octagonal" },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::square_case_octagonal_compound
