// @lat: [[engine/skills#i_beam_compound_section]]

#include "i_beam_compound_section.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <cmath>

namespace koocadcam::skill::i_beam_compound_section {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.length_mm <= 0.0 || in.height_mm <= 0.0 ||
        in.flange_w_mm <= 0.0 || in.flange_t_mm <= 0.0 || in.web_t_mm <= 0.0)
    {
        r.add("DFM-IBEAM-INPUT", "error",
              "i_beam_compound_section: all dimensions must be > 0");
        return r;
    }

    if (in.flange_t_mm * 2.0 >= in.height_mm) {
        r.add("DFM-IBEAM-GEOM", "error",
              "i_beam_compound_section: 2 × flange_t (" +
              std::to_string(in.flange_t_mm * 2.0) +
              " mm) >= height (" + std::to_string(in.height_mm) +
              " mm) — no web remains");
    }
    if (in.web_t_mm >= in.flange_w_mm) {
        r.add("DFM-IBEAM-GEOM", "error",
              "i_beam_compound_section: web_t (" +
              std::to_string(in.web_t_mm) + ") >= flange_w (" +
              std::to_string(in.flange_w_mm) + ")");
    }

    const double webH = in.height_mm - 2.0 * in.flange_t_mm;
    if (webH > 0.0 && in.web_t_mm > 0.0) {
        const double webRatio = webH / in.web_t_mm;
        if (webRatio > 100.0) {
            r.add("DFM-IBEAM-WEB-PROP", "warning",
                  "i_beam_compound_section: web slenderness h/tw " +
                  std::to_string(webRatio) +
                  " > 100 — buckling risk per AISC W-shape limits");
        }
    }

    const double flangeOver = (in.flange_w_mm - in.web_t_mm) / 2.0;
    if (flangeOver > 0.0 && in.flange_t_mm > 0.0) {
        const double fr = flangeOver / in.flange_t_mm;
        if (fr > 8.0) {
            r.add("DFM-IBEAM-FLANGE", "warning",
                  "i_beam_compound_section: flange outstand b/t " +
                  std::to_string(fr) +
                  " > 8 — local flange buckling risk");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "i_beam_compound_section DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double L  = in.length_mm;
    const double H  = in.height_mm;
    const double Bf = in.flange_w_mm;
    const double Tf = in.flange_t_mm;
    const double Tw = in.web_t_mm;
    const double Hw = H - 2.0 * Tf;   // web height

    // Sub-feature 1: top flange
    const gp_Pnt topOrigin(0.0, -Bf / 2.0, H - Tf);
    const TopoDS_Shape topFlange = pr::box(
        gp_Ax2(topOrigin, gp::DZ()), L, Bf, Tf);

    // Sub-feature 2: web (centered in Y)
    const gp_Pnt webOrigin(0.0, -Tw / 2.0, Tf);
    const TopoDS_Shape web = pr::box(
        gp_Ax2(webOrigin, gp::DZ()), L, Tw, Hw);

    // Sub-feature 3: bottom flange
    const gp_Pnt botOrigin(0.0, -Bf / 2.0, 0.0);
    const TopoDS_Shape botFlange = pr::box(
        gp_Ax2(botOrigin, gp::DZ()), L, Bf, Tf);

    // Fuse all three into a single I-section.
    TopoDS_Shape ibeam = pr::fuse(topFlange, web);
    ibeam = pr::fuse(ibeam, botFlange);

    // Derived metrics (used by CAPP / DFM downstream).
    const double areaMm2  = 2.0 * Bf * Tf + Hw * Tw;
    const double volMm3   = areaMm2 * L;
    const double weightKg = volMm3 * 7.85e-6;  // mild-steel default

    json params = {
        { "length_mm",   L },
        { "height_mm",   H },
        { "flange_w_mm", Bf },
        { "flange_t_mm", Tf },
        { "web_t_mm",    Tw },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     3 },
        { "length_mm",            L },
        { "height_mm",            H },
        { "flange_w_mm",          Bf },
        { "flange_t_mm",          Tf },
        { "web_t_mm",             Tw },
        { "web_height_mm",        Hw },
        { "section_area_mm2",     areaMm2 },
        { "section_volume_mm3",   volMm3 },
        { "approx_weight_kg",     weightKg },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "hot_roll;saw_cut;weld_assemble";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = L;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    // 60 s/m roll-form-and-saw line.
    tooling.est_cycle_time_s  = std::max(30.0, L / 1000.0 * 60.0);
    tooling.extra = {
        { "subfeature_sequence", {
            { { "name", "top_flange" },    { "L", L }, { "W", Bf }, { "T", Tf } },
            { { "name", "web" },           { "L", L }, { "W", Tw }, { "T", Hw } },
            { { "name", "bottom_flange" }, { "L", L }, { "W", Bf }, { "T", Tf } },
        } },
        { "standard", "AISC_W_shape_or_ISO_657-15" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(ibeam, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::i_beam_compound_section applied: L={} H={} bf={} tf={} tw={}",
                  L, H, Bf, Tf, Tw);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" },
                                { "is_compound", true },
                                { "subfeature_count", 3 } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::i_beam_compound_section
