// @lat: [[engine/skills#laser_engrave_deep]]

#include "laser_engrave_deep.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::laser_engrave_deep {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr const char* kShapeRect = "rectangular";
constexpr const char* kShapeCirc = "circular";

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.shape != kShapeRect && in.shape != kShapeCirc) {
        r.add("DFM-INPUT", "error",
              "laser_engrave_deep shape '" + in.shape +
              "' not recognised — use 'rectangular' or 'circular'");
        return r;
    }
    if (in.depth_mm < kMinDepth_mm) {
        r.add("DFM-LE-DEPTH-LO", "error",
              "laser_engrave_deep depth_mm " + std::to_string(in.depth_mm) +
              " < " + std::to_string(kMinDepth_mm) +
              " mm — use laser_mark for sub-0.1 mm engraving");
    }
    if (in.depth_mm > kMaxDepth_mm) {
        r.add("DFM-LE-DEPTH-HI", "error",
              "laser_engrave_deep depth_mm " + std::to_string(in.depth_mm) +
              " > " + std::to_string(kMaxDepth_mm) +
              " mm — use a milled pocket or wire EDM for deeper cavities");
    }
    if (in.shape == kShapeRect) {
        if (in.length_mm <= 0.0 || in.width_mm <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "laser_engrave_deep rectangular shape needs length_mm > 0 "
                  "and width_mm > 0");
        }
    } else if (in.shape == kShapeCirc) {
        if (in.diameter_mm <= 0.0) {
            r.add("DFM-INPUT", "error",
                  "laser_engrave_deep circular shape needs diameter_mm > 0");
        }
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "laser_engrave_deep DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("laser_engrave_deep: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const double kOverhang = 0.05;
    TopoDS_Shape cutter;
    double stockRemoved = 0.0;

    if (in.shape == kShapeRect) {
        const gp_Pnt bottomCenter(in.position_x_mm, in.position_y_mm,
                                  zMax - in.depth_mm);
        cutter = pr::roundedRectPocketTool(
            bottomCenter, in.length_mm, in.width_mm,
            in.depth_mm + kOverhang, 0.0);
        stockRemoved = in.length_mm * in.width_mm * in.depth_mm;
    } else {
        const gp_Pnt toolStart(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
        const gp_Ax2 toolAx(toolStart, gp_Dir(0.0, 0.0, -1.0));
        cutter = pr::cylinder(toolAx, in.diameter_mm / 2.0, in.depth_mm + kOverhang);
        stockRemoved = M_PI * (in.diameter_mm / 2.0) * (in.diameter_mm / 2.0)
                     * in.depth_mm;
    }

    const TopoDS_Shape newShape = pr::cut(wp.shape(), cutter);

    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "shape",          in.shape },
        { "length_mm",      in.length_mm },
        { "width_mm",       in.width_mm },
        { "diameter_mm",    in.diameter_mm },
        { "depth_mm",       in.depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "shape",                      in.shape },
        { "bottom_planar_face_present", true },
        { "depth_mm",                   in.depth_mm },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "laser_head";
    tooling.tool_dia_mm       = 0.05;       // beam diameter
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "fiber_laser";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = stockRemoved;
    // Pulsed fiber laser ablates ~3 mm³/s in steel; conservative 2.0.
    tooling.est_cycle_time_s  = std::max(5.0, stockRemoved / 2.0);
    tooling.extra["dfm_findings"]        = findings;
    tooling.extra["wavelength_nm"]       = 1064.0;     // fiber-laser standard
    tooling.extra["pulse_freq_khz"]      = 50.0;
    tooling.extra["machining_constraint"] =
        "Deep laser engraving — shallow pocket (0.1–2.0 mm); no tool wear; "
        "HAZ ~ 50 μm depending on pulse parameters";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::laser_engrave_deep applied: shape={} depth={} faces {}→{}",
                  in.shape, in.depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

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

}  // namespace koocadcam::skill::laser_engrave_deep
