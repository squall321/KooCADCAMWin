// @lat: [[engine/skills#ball_screw_nut_pocket]]

#include "ball_screw_nut_pocket.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

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

// ── Recognition (metadata replay + geometric fallback) ──────────────────
//
// Geometric signature of a ball-screw nut pocket:
//   - 1 large -Z (or arbitrary) cylindrical face (nut OD pocket, large radius)
//   - 1 smaller concentric cylindrical face (shaft thru-bore)
//   - 4 small cylindrical mounting holes arranged at 90° on a common PCD
//     centered on the nut/shaft axis.

namespace {

struct CylSig {
    int    faceIdx;
    gp_Pnt center;         // axis location point
    gp_Dir axDir;
    double radius;
};

std::vector<CylSig> collectCyls(const Workpiece& wp)
{
    std::vector<CylSig> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cy = surf.Cylinder();
        out.push_back({ i, cy.Axis().Location(), cy.Axis().Direction(), cy.Radius() });
    }
    return out;
}

std::vector<RecognizedFeature> geometric_fallback(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    if (wp.shape().IsNull()) return out;

    const auto cyls = collectCyls(wp);
    if (cyls.size() < 6) return out;  // nut + shaft + 4 mounts

    // Group by axis direction (≈ same).  Pick the dominant direction.
    // For the canonical case axis = -Z, all cylinders share Z direction.
    // We try each cylinder as a potential center.
    for (size_t cIdx = 0; cIdx < cyls.size(); ++cIdx) {
        const CylSig& center = cyls[cIdx];
        // Identify "co-axis" cylinders (same axis line as center).
        std::vector<CylSig> coax;
        std::vector<CylSig> ringed;
        for (const auto& c : cyls) {
            if (std::abs(std::abs(c.axDir.Dot(center.axDir)) - 1.0) > 1e-3) continue;
            // distance between axis lines (project center difference perp to axis)
            const gp_Vec d(center.center, c.center);
            const gp_Vec ax(center.axDir.X(), center.axDir.Y(), center.axDir.Z());
            const gp_Vec perp = d - ax * d.Dot(ax);
            const double dist = perp.Magnitude();
            if (dist < 0.5) coax.push_back(c);
            else            ringed.push_back(c);
        }
        if (coax.size() < 2) continue;

        // Pick the largest coax radius = nut OD pocket.
        double nutR = 0.0, shaftR = 0.0;
        for (const auto& c : coax) {
            if (c.radius > nutR) {
                shaftR = nutR;
                nutR = c.radius;
            } else if (c.radius > shaftR) {
                shaftR = c.radius;
            }
        }
        if (nutR <= 0.0 || nutR < 4.0) continue;

        // Mounting holes: need 4 cylinders, same radius, equidistant from
        // the center axis (the PCD).
        if (ringed.size() < 4) continue;

        // Compute distance of each ringed cyl from the center axis line.
        struct RingedHole { double dist; double radius; double angle; };
        std::vector<RingedHole> holes;
        for (const auto& c : ringed) {
            const gp_Vec d(center.center, c.center);
            const gp_Vec ax(center.axDir.X(), center.axDir.Y(), center.axDir.Z());
            const gp_Vec perp = d - ax * d.Dot(ax);
            const double dist = perp.Magnitude();
            holes.push_back({ dist, c.radius, std::atan2(perp.Y(), perp.X()) });
        }
        // Sort by distance, pick the 4 closest with identical distance.
        std::sort(holes.begin(), holes.end(),
                  [](const RingedHole& a, const RingedHole& b){ return a.dist < b.dist; });
        // Find the cluster of 4 with similar distance (PCD/2) and similar radius.
        int bestStart = -1;
        for (size_t i = 0; i + 3 < holes.size(); ++i) {
            const double d0 = holes[i].dist;
            const double r0 = holes[i].radius;
            bool ok = true;
            for (int k = 1; k < 4; ++k) {
                if (std::abs(holes[i + k].dist   - d0) > 0.5) { ok = false; break; }
                if (std::abs(holes[i + k].radius - r0) > 0.3) { ok = false; break; }
            }
            if (ok) { bestStart = static_cast<int>(i); break; }
        }
        if (bestStart < 0) continue;

        const double pcd   = 2.0 * holes[bestStart].dist;
        const double holeR = holes[bestStart].radius;
        const double nutOD = 2.0 * nutR;

        // Determine pattern: square4 = angles at 0/90/180/270; diamond4 at 45.
        const double a0deg = holes[bestStart].angle * 180.0 / M_PI;
        const double a0    = std::fmod(std::abs(a0deg) + 22.5, 90.0);
        const std::string pattern = (a0 < 22.5) ? "square4" : "diamond4";

        json recovered = {
            { "position_x_mm",        center.center.X() },
            { "position_y_mm",        center.center.Y() },
            { "axis_dir",             { -center.axDir.X(), -center.axDir.Y(), -center.axDir.Z() } },
            { "nut_od_mm",            nutOD - 0.1 },   // remove slip clearance approximation
            { "nut_l_mm",             0.0 },           // depth not directly recoverable from a face
            { "mounting_pattern",     pattern },
            { "mounting_pcd_mm",      pcd },
            { "mounting_hole_dia_mm", 2.0 * holeR },
            { "shaft_thru_dia_mm",    2.0 * shaftR },
        };
        json matched = {
            { "source",         "geometric_fallback" },
            { "nut_face_id",    coax.front().faceIdx },
            { "coax_count",     static_cast<int>(coax.size()) },
            { "mount_count",    4 },
            { "detected_pcd",   pcd },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.65, matched });
        return out;  // one match per workpiece
    }
    return out;
}

}  // namespace

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
    if (out.empty()) {
        out = geometric_fallback(wp);
    }
    return out;
}

}  // namespace koocadcam::skill::ball_screw_nut_pocket
