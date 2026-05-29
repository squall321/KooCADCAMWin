// @lat: [[engine/skills#ball_screw_nut_pocket]]

#include "ball_screw_nut_pocket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::ball_screw_nut_pocket {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

constexpr double kSlipClearanceMm   = 0.05;   // H7/h6-ish OD clearance
constexpr double kMountingHoleDepth = 12.0;   // through-fastener pilot
constexpr double kThruPocketDepth   = 30.0;   // generous shaft clearance
constexpr double kDefaultPCDOffset  = 8.0;    // mounting PCD = nut_od + 8

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.nut_od_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "ball_screw_nut_pocket: nut_od_mm must be > 0");
    }
    if (in.nut_l_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "ball_screw_nut_pocket: nut_l_mm must be > 0");
    }
    if (in.mounting_hole_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "ball_screw_nut_pocket: mounting_hole_dia " +
              std::to_string(in.mounting_hole_dia_mm) + " mm < 0.8 mm");
    }

    if (in.nut_od_mm > 0.0 && (in.nut_od_mm < 10.0 || in.nut_od_mm > 80.0)) {
        r.add("DFM-BS-NUTOD", "warning",
              "ball_screw_nut_pocket: nut_od_mm " +
              std::to_string(in.nut_od_mm) +
              " outside typical [10, 80] mm range");
    }

    const double pcd = (in.mounting_pcd_mm > 0.0)
        ? in.mounting_pcd_mm
        : (in.nut_od_mm + kDefaultPCDOffset);
    // Mounting holes must not overlap the nut OD slip pocket.
    if (in.nut_od_mm > 0.0 &&
        pcd - in.mounting_hole_dia_mm < in.nut_od_mm + 1.0) {
        r.add("DFM-BS-PCD", "error",
              "ball_screw_nut_pocket: mounting holes overlap nut bore (PCD=" +
              std::to_string(pcd) + " hole=" +
              std::to_string(in.mounting_hole_dia_mm) + " nut_od=" +
              std::to_string(in.nut_od_mm) + ")");
    }

    if (in.mounting_pattern != "square4" && in.mounting_pattern != "diamond4") {
        r.add("DFM-INPUT", "error",
              "ball_screw_nut_pocket: mounting_pattern must be 'square4' or 'diamond4', got '" +
              in.mounting_pattern + "'");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ball_screw_nut_pocket DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("ball_screw_nut_pocket: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax-xMin)*(xMax-xMin) + (yMax-yMin)*(yMax-yMin) + (zMax-zMin)*(zMax-zMin));

    constexpr double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Tool start above the entry face along -axis_dir.
    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0) {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
        } else {
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kOverhang);
        }
    } else {
        const double margin = bboxDiag + 1.0;
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * margin,
            in.position_y_mm - adir.Y() * margin,
            (zMin + zMax)/2.0 - adir.Z() * margin);
    }

    const gp_Ax2 nutAx(toolStart, adir);

    // ── Sub-feature 1: nut OD slip pocket (cylinder) ──────────────────
    const double slipDia = in.nut_od_mm + 2.0 * kSlipClearanceMm;
    const TopoDS_Shape nutTool =
        pr::cylinder(nutAx, slipDia / 2.0, in.nut_l_mm + kOverhang);

    // ── Sub-feature 2: shaft through-bore (smaller dia, deeper) ───────
    const double shaftDia = (in.shaft_thru_dia_mm > 0.0)
        ? in.shaft_thru_dia_mm
        : std::max(2.0, in.nut_od_mm / 2.0 - 1.0);
    // We extend the shaft through-bore well past the nut pocket bottom.
    const double thruDepth = in.nut_l_mm + kThruPocketDepth + kOverhang;
    const TopoDS_Shape shaftTool =
        pr::cylinder(nutAx, shaftDia / 2.0, thruDepth);

    // Concentric overlapping cylinders → fuse first then cut.
    const TopoDS_Shape concentricCutter = pr::fuse(nutTool, shaftTool);

    // ── Sub-features 3..6: 4 mounting holes around the PCD ────────────
    const double pcd = (in.mounting_pcd_mm > 0.0)
        ? in.mounting_pcd_mm
        : (in.nut_od_mm + kDefaultPCDOffset);
    const double angOffset = (in.mounting_pattern == "diamond4")
        ? (M_PI / 4.0) : 0.0;

    std::vector<TopoDS_Shape> holes;
    holes.reserve(4);
    json holePositions = json::array();
    for (int k = 0; k < 4; ++k) {
        const double theta = angOffset + k * (M_PI / 2.0);
        const double hx = in.position_x_mm + (pcd / 2.0) * std::cos(theta);
        const double hy = in.position_y_mm + (pcd / 2.0) * std::sin(theta);
        // Mounting hole goes deep (through-fastener); we use a generous depth.
        gp_Pnt holeStart(hx, hy,
            (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6 && adir.Z() < 0)
                ? zMax + kOverhang
                : toolStart.Z());
        gp_Ax2 holeAx(holeStart, adir);
        holes.push_back(pr::cylinder(holeAx, in.mounting_hole_dia_mm / 2.0,
                                     kMountingHoleDepth + kOverhang));
        holePositions.push_back({ { "x", hx }, { "y", hy } });
    }

    // Combine: first the concentric fused cutter, then the 4 holes.
    std::vector<TopoDS_Shape> allTools;
    allTools.reserve(1 + holes.size());
    allTools.push_back(concentricCutter);
    for (const auto& h : holes) allTools.push_back(h);

    const TopoDS_Shape newShape = pr::cutMany(wp.shape(), allTools);

    // Sub-feature count = nut pocket + shaft thru + 4 mounting holes = 6.
    constexpr int kSubFeatures = 6;

    json params = {
        { "entry_face_id",        *entryId },
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
        { "nut_od_mm",            in.nut_od_mm },
        { "nut_l_mm",             in.nut_l_mm },
        { "mounting_pattern",     in.mounting_pattern },
        { "mounting_pcd_mm",      in.mounting_pcd_mm },
        { "mounting_hole_dia_mm", in.mounting_hole_dia_mm },
        { "shaft_thru_dia_mm",    in.shaft_thru_dia_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      kSubFeatures },
        { "nut_pocket_count",      1 },
        { "shaft_thru_count",      1 },
        { "mounting_hole_count",   4 },
        { "nut_od_mm",             in.nut_od_mm },
        { "nut_l_mm",              in.nut_l_mm },
        { "slip_dia_mm",           slipDia },
        { "slip_clearance_mm",     kSlipClearanceMm },
        { "shaft_thru_dia_mm",     shaftDia },
        { "mounting_pcd_mm",       pcd },
        { "mounting_pattern",      in.mounting_pattern },
        { "mounting_hole_dia_mm",  in.mounting_hole_dia_mm },
        { "hole_positions",        holePositions },
        { "axis_dir",              { adir.X(), adir.Y(), adir.Z() } },
    };
    ToolingMeta tooling;
    tooling.tool_type        = "boring_bar;drill";
    tooling.tool_dia_mm      = slipDia;
    tooling.tool_length_mm   = in.nut_l_mm * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double nutVol   = M_PI * (slipDia/2.0)*(slipDia/2.0) * in.nut_l_mm;
    const double shaftVol = M_PI * (shaftDia/2.0)*(shaftDia/2.0)
                          * (kThruPocketDepth);   // beyond nut pocket only
    const double mountVol = 4.0 * M_PI
                          * (in.mounting_hole_dia_mm/2.0)
                          * (in.mounting_hole_dia_mm/2.0) * kMountingHoleDepth;
    tooling.stock_removed_mm3 = nutVol + shaftVol + mountVol;
    tooling.est_cycle_time_s  = std::max(5.0, in.nut_l_mm * 0.4 + 4.0 * 2.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "boring_bar" },
              { "purpose",   "nut OD slip pocket" },
              { "tool_dia_mm", slipDia },
              { "depth_mm",    in.nut_l_mm } },
            { { "tool_type", "drill" },
              { "purpose",   "shaft thru-bore" },
              { "tool_dia_mm", shaftDia },
              { "depth_mm",    kThruPocketDepth } },
            { { "tool_type", "drill" },
              { "purpose",   "4 mounting holes" },
              { "tool_dia_mm", in.mounting_hole_dia_mm },
              { "depth_mm",    kMountingHoleDepth },
              { "pass_count",  4 } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::ball_screw_nut_pocket applied: nut_od={} nut_l={} pcd={} pattern={}",
                  in.nut_od_mm, in.nut_l_mm, pcd, in.mounting_pattern);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay) ────────────────────────────────────────

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
            { "source",            "metadata_replay" },
            { "subfeature_count",  f.pattern.value("subfeature_count", 0) },
        };
        out.push_back(r);
    }
    return out;
}

}  // namespace koocadcam::skill::ball_screw_nut_pocket
