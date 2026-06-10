// @lat: [[engine/skills#sprocket_blank_with_bore]]

#include "sprocket_blank_with_bore.hpp"

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

namespace koocadcam::skill::sprocket_blank_with_bore {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.outer_dia_mm <= 0.0 || in.thickness_mm <= 0.0 ||
        in.bore_dia_mm  <= 0.0) {
        r.add("DFM-INPUT", "error",
              "sprocket_blank_with_bore: outer/thickness/bore must be > 0");
        return r;
    }

    if (in.bore_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "sprocket_blank_with_bore: bore_dia < 0.8 mm");
    }
    if (in.bore_dia_mm >= in.outer_dia_mm) {
        r.add("DFM-INPUT", "error",
              "sprocket_blank_with_bore: bore_dia >= outer_dia");
    }

    const double wallThk = (in.outer_dia_mm - in.bore_dia_mm) / 2.0;
    if (wallThk < 3.0) {
        r.add("DFM-SPROCKET-WALL", "error",
              "sprocket_blank_with_bore: hub wall thickness " +
              std::to_string(wallThk) + " mm < 3 mm minimum");
    }

    if (in.lightening_count < 0 || in.lightening_count > 12) {
        r.add("DFM-INPUT", "error",
              "sprocket_blank_with_bore: lightening_count out of range [0, 12]");
    }

    // Lightening-hole geometry collision check.
    const double bRad = in.bore_dia_mm / 2.0;
    const double oRad = in.outer_dia_mm / 2.0;
    const double pitchR    = (bRad + oRad) / 2.0;
    const double lightDia  = std::min(wallThk * 0.6, (oRad - bRad) * 0.5);
    if (in.lightening_count > 0 &&
        (pitchR - lightDia / 2.0 <= bRad ||
         pitchR + lightDia / 2.0 >= oRad)) {
        r.add("DFM-SPROCKET-LIGHT", "error",
              "sprocket_blank_with_bore: lightening hole geometry collides "
              "with bore or outer rim");
    }

    if (in.chamfer_size_mm < 0.0 ||
        in.chamfer_size_mm >= in.thickness_mm / 2.0) {
        r.add("DFM-INPUT", "error",
              "sprocket_blank_with_bore: chamfer_size_mm invalid");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "sprocket_blank_with_bore DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double oRad      = in.outer_dia_mm / 2.0;
    const double bRad      = in.bore_dia_mm  / 2.0;
    const double Z         = in.thickness_mm;
    const double kOverhang = 0.05;

    // ── Tool 1: through bore
    const gp_Ax2 axZ(gp_Pnt(0, 0, -kOverhang), gp::DZ());
    const TopoDS_Shape boreTool = pr::cylinder(axZ, bRad, Z + 2.0 * kOverhang);

    // ── Tools 2..N+1: lightening holes equispaced around a pitch circle
    const double pitchR   = (bRad + oRad) / 2.0;
    const double lightDia = std::min((oRad - bRad) * 0.5,
                                     (oRad - bRad) * 0.5);   // 50% of wall
    const double lightR   = lightDia / 2.0;
    const int N = static_cast<int>(in.lightening_count);

    std::vector<TopoDS_Shape> lightCutters;
    lightCutters.reserve(static_cast<size_t>(N));
    for (int i = 0; i < N; ++i) {
        const double th = (2.0 * M_PI * i) / N;
        const gp_Pnt p(pitchR * std::cos(th),
                       pitchR * std::sin(th),
                       -kOverhang);
        lightCutters.push_back(
            pr::cylinder(gp_Ax2(p, gp::DZ()), lightR, Z + 2.0 * kOverhang));
    }

    // ── Tools N+2, N+3: top + bottom face chamfers (cone frusta around OD)
    //
    // We model each chamfer as a cone-frustum-shaped annulus subtracting
    // a wedge from the corner.  The cone frustum sits between (r=oR) at
    // the face and (r=oR - chamfer) at chamfer_size into the body.
    std::vector<TopoDS_Shape> tools;
    tools.push_back(boreTool);
    for (const auto& l : lightCutters) tools.push_back(l);

    const double ch = in.chamfer_size_mm;
    if (ch > 0.0) {
        // Top-face chamfer: cone from (oR+ch) at z=Z+ch down to (oR-ch) at z=Z-ch.
        // We use a hollow annular cone via OCCT primitives by subtracting an
        // inner cylinder from a cone-frustum outer.  For simplicity here we
        // synthesize a thick annular cone ring as: outerCone - innerCyl.
        // Outer cone: r1 (bottom) = oR + ch, r2 (top) = oR - ch, height = 2·ch.
        const gp_Ax2 axTop(gp_Pnt(0.0, 0.0, Z - ch), gp::DZ());
        const TopoDS_Shape outerConeTop =
            pr::coneFrustum(axTop, oRad + ch, oRad - ch, 2.0 * ch);
        // Inner cylinder = (oR - 2·ch) cylinder, slightly taller than the cone.
        const double innerR = std::max(0.1, oRad - 2.0 * ch);
        const TopoDS_Shape innerCylTop =
            pr::cylinder(axTop, innerR, 2.0 * ch + 2.0 * kOverhang);
        // Chamfer tool = outerCone − innerCyl (only cuts the ring shell).
        const TopoDS_Shape topChamferTool = pr::cut(outerConeTop, innerCylTop);
        tools.push_back(topChamferTool);

        // Bottom-face chamfer mirror.
        const gp_Ax2 axBot(gp_Pnt(0.0, 0.0, -ch), gp::DZ());
        const TopoDS_Shape outerConeBot =
            pr::coneFrustum(axBot, oRad - ch, oRad + ch, 2.0 * ch);
        const TopoDS_Shape innerCylBot =
            pr::cylinder(axBot, innerR, 2.0 * ch + 2.0 * kOverhang);
        const TopoDS_Shape botChamferTool = pr::cut(outerConeBot, innerCylBot);
        tools.push_back(botChamferTool);
    }

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), tools);

    // Subfeature count = 1 (bore) + N (lightening) + 2 (top/bot chamfer when ch > 0)
    const int subCount = 1 + N + (ch > 0.0 ? 2 : 0);

    // ── Signature
    json params = {
        { "outer_dia_mm",     in.outer_dia_mm },
        { "bore_dia_mm",      in.bore_dia_mm  },
        { "thickness_mm",     in.thickness_mm },
        { "lightening_count", N },
        { "chamfer_size_mm",  ch },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     subCount },
        { "outer_dia_mm",         in.outer_dia_mm },
        { "bore_dia_mm",          in.bore_dia_mm },
        { "thickness_mm",         in.thickness_mm },
        { "wall_thickness_mm",    (in.outer_dia_mm - in.bore_dia_mm) / 2.0 },
        { "lightening_count",     N },
        { "lightening_pitch_r",   pitchR },
        { "lightening_dia_mm",    lightDia },
        { "chamfer_size_mm",      ch },
    };

    ToolingMeta tooling;
    tooling.tool_type    = "drill;end_mill;chamfer_mill";
    tooling.tool_dia_mm  = in.bore_dia_mm;
    tooling.tool_material = "carbide";
    tooling.stock_removed_mm3 =
        M_PI * bRad * bRad * Z +
        static_cast<double>(N) * M_PI * lightR * lightR * Z;
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" }, { "tool_dia_mm", in.bore_dia_mm } },
            { { "tool_type", "drill" }, { "tool_dia_mm", lightDia },
              { "pass_count", N }, { "note", "lightening holes" } },
            { { "tool_type", "chamfer_mill" }, { "size_mm", ch },
              { "pass_count", 2 }, { "note", "top + bottom face chamfer" } },
        } }
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::sprocket_blank_with_bore applied: OD={} bore={} N_light={}",
                  in.outer_dia_mm, in.bore_dia_mm, N);

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
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",           "feature_history_replay" },
            { "subfeature_count", f.pattern.value("subfeature_count", 0) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::sprocket_blank_with_bore
