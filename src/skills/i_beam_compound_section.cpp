// @lat: [[engine/skills#i_beam_compound_section]]

#include "i_beam_compound_section.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <vector>

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

// ── Recognition (metadata replay + geometric fallback) ──────────────────
//
// Geometric signature for an I-section (after STEP round-trip):
//   The fused 3-box I-beam exposes planar faces grouped into THREE
//   distinct Z-bands along its height:
//     - top flange: planar faces at z ≈ Z_max  AND  z ≈ Z_max − Tf
//     - web:        2 parallel vertical (±Y normal) faces at |y| = Tw/2
//                   with axial extent equal to (Hw + 2·Tf − epsilon)
//                   sandwiched between the two flange planes
//     - bot flange: planar faces at z ≈ Z_min  AND  z ≈ Z_min + Tf
//   Net width of the top/bottom flange faces is Bf along Y, length L
//   along X.  The web is the narrowest portion in Y.
//
// Robust recovery: scan the Y-extent of each Z-slice to identify the
// flange width vs. web width.

namespace {

struct PlanarFaceDesc {
    int    faceIdx;
    gp_Dir normal;
    gp_Pnt center;
    double area;
};

std::vector<PlanarFaceDesc> collectPlanar(const Workpiece& wp)
{
    std::vector<PlanarFaceDesc> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        PlanarFaceDesc d;
        d.faceIdx = i;
        try {
            d.normal = wp.faceNormal(i);
            d.center = wp.faceCenter(i);
            d.area   = wp.faceArea(i);
        } catch (...) { continue; }
        out.push_back(d);
    }
    return out;
}

// Geometric fallback: detect I-beam by checking that the bbox has the
// "I" cross-section signature when sliced perpendicular to the length axis.
// We approximate from the workpiece face set: the cross-section has THREE
// distinct widths along the height (wide-narrow-wide) — that's an I.
std::vector<RecognizedFeature> geometric_fallback(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    if (wp.shape().IsNull()) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double L  = xMax - xMin;
    const double H  = zMax - zMin;
    const double Bf = yMax - yMin;
    if (L <= 0.0 || H <= 0.0 || Bf <= 0.0) return out;

    const auto pfaces = collectPlanar(wp);
    if (pfaces.size() < 8) return out;  // I-beam expects many planar faces

    // Top flange top face must be near z = zMax with +Z normal.
    // Bottom flange bottom face must be near z = zMin with -Z normal.
    // Flange-step faces (where web meets flange) appear with ±Z normal at
    // intermediate Z bands.
    bool hasTopOuter = false, hasBotOuter = false;
    std::vector<double> flangeStepZ;      // Z values of step faces between flange/web
    double maxFlangeWidth = 0.0;

    for (const auto& f : pfaces) {
        if (std::abs(std::abs(f.normal.Z()) - 1.0) > 1e-3) continue;
        if (std::abs(f.center.Z() - zMax) < 1e-3 && f.normal.Z() > 0) {
            hasTopOuter = true;
            maxFlangeWidth = std::max(maxFlangeWidth, std::sqrt(f.area / L));
        }
        if (std::abs(f.center.Z() - zMin) < 1e-3 && f.normal.Z() < 0) {
            hasBotOuter = true;
            maxFlangeWidth = std::max(maxFlangeWidth, std::sqrt(f.area / L));
        }
        // Intermediate Z-band step faces (flange inner surface adjacent to web)
        if (f.center.Z() > zMin + 1e-3 && f.center.Z() < zMax - 1e-3) {
            flangeStepZ.push_back(f.center.Z());
        }
    }
    if (!hasTopOuter || !hasBotOuter) return out;

    // For an I-beam there are 4 step faces (2 per flange × 2 sides of web),
    // clustered into TWO Z-bands: one near zMax - Tf, one near zMin + Tf.
    if (flangeStepZ.size() < 4) return out;
    std::sort(flangeStepZ.begin(), flangeStepZ.end());
    const double zStepLow  = flangeStepZ.front();
    const double zStepHigh = flangeStepZ.back();
    if (zStepHigh - zStepLow < H * 0.1) return out;  // need real separation

    const double Tf_bot = zStepLow  - zMin;
    const double Tf_top = zMax      - zStepHigh;
    if (Tf_bot <= 0.0 || Tf_top <= 0.0) return out;
    const double Tf = (Tf_bot + Tf_top) / 2.0;
    if (std::abs(Tf_bot - Tf_top) > Tf * 0.5) return out;  // top/bot must match

    // Web thickness: scan the Y-extents of vertical (±Y normal) faces in
    // the middle Z-band — these are the two web walls.
    std::vector<double> webWallY;
    for (const auto& f : pfaces) {
        if (std::abs(std::abs(f.normal.Y()) - 1.0) > 1e-3) continue;
        if (f.center.Z() <= zStepLow + 1e-3) continue;
        if (f.center.Z() >= zStepHigh - 1e-3) continue;
        webWallY.push_back(f.center.Y());
    }
    if (webWallY.size() < 2) return out;
    std::sort(webWallY.begin(), webWallY.end());
    const double Tw = webWallY.back() - webWallY.front();
    if (Tw <= 0.0 || Tw >= Bf) return out;

    // Pythia-check: web thickness must be visibly less than flange width.
    if (Tw > Bf * 0.5) return out;

    const double Hw = H - 2.0 * Tf;
    if (Hw <= 0.0) return out;

    json recovered = {
        { "length_mm",   L  },
        { "height_mm",   H  },
        { "flange_w_mm", Bf },
        { "flange_t_mm", Tf },
        { "web_t_mm",    Tw },
    };
    json matched = {
        { "source",             "geometric_fallback" },
        { "z_step_low_mm",      zStepLow  },
        { "z_step_high_mm",     zStepHigh },
        { "flange_step_count",  static_cast<int>(flangeStepZ.size()) },
        { "web_wall_count",     static_cast<int>(webWallY.size()) },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.70, matched });
    return out;
}

}  // namespace

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
    if (out.empty()) {
        out = geometric_fallback(wp);
    }
    return out;
}

}  // namespace koocadcam::skill::i_beam_compound_section
