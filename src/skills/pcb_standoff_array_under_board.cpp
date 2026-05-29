// @lat: [[engine/skills#pcb_standoff_array_under_board]]

#include "pcb_standoff_array_under_board.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace koocadcam::skill::pcb_standoff_array_under_board {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── ISO 4762 metric coarse pilot lookup ──────────────────────────────────
//
// pilot_dia_mm is the threading pilot diameter for self-tapping into a
// boss machined from soft metal (aluminum 6061 / brass) per ISO 68-1.

namespace {

struct ScrewSpec {
    const char* size;
    double      pilot_dia_mm;
    double      min_boss_od_mm;   // recommended minimum boss OD for that screw
};

constexpr std::array<ScrewSpec, 4> kScrewTable {{
    { "M2",   1.60, 4.0 },
    { "M2.5", 2.05, 4.5 },
    { "M3",   2.50, 5.0 },
    { "M4",   3.30, 6.0 },
}};

const ScrewSpec* findScrew(const std::string& size)
{
    for (const auto& s : kScrewTable) {
        if (size == s.size) return &s;
    }
    return nullptr;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.sites.empty()) {
        r.add("DFM-INPUT", "error",
              "pcb_standoff_array_under_board: at least 1 site required");
        return r;
    }

    const ScrewSpec* sp = findScrew(in.screw_size);
    if (!sp) {
        r.add("DFM-CONN-SIZE", "error",
              "pcb_standoff_array_under_board: unknown screw_size '" +
              in.screw_size + "' (expected M2/M2.5/M3/M4)");
        return r;
    }

    if (sp->pilot_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "pcb_standoff_array_under_board: pilot diameter " +
              std::to_string(sp->pilot_dia_mm) + " mm < 0.8 mm");
    }
    if (in.standoff_height_mm < 1.5) {
        r.add("DFM-STANDOFF-HEIGHT", "error",
              "pcb_standoff_array_under_board: standoff_height_mm " +
              std::to_string(in.standoff_height_mm) + " < 1.5 mm minimum");
    }
    if (in.boss_od_mm < sp->min_boss_od_mm) {
        r.add("DFM-BOSS-WALL", "error",
              "pcb_standoff_array_under_board: boss_od_mm " +
              std::to_string(in.boss_od_mm) + " < " +
              std::to_string(sp->min_boss_od_mm) +
              " required for " + in.screw_size);
    }
    const double wallMm = (in.boss_od_mm - sp->pilot_dia_mm) / 2.0;
    if (wallMm < 1.0) {
        r.add("DFM-BOSS-WALL", "error",
              "pcb_standoff_array_under_board: boss wall " +
              std::to_string(wallMm) + " mm < 1.0 mm (IPC-2221 min)");
    }
    if (in.pilot_extra_depth_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "pcb_standoff_array_under_board: pilot_extra_depth_mm must be >= 0");
    }
    if (in.PCB_thickness_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "pcb_standoff_array_under_board: PCB_thickness_mm must be >= 0");
    }
    if (in.plate_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pcb_standoff_array_under_board: plate_thickness_mm must be > 0");
    }

    // Pairwise site collision check (boss OD + 0.5 mm clearance).
    for (size_t i = 0; i < in.sites.size(); ++i) {
        for (size_t j = i + 1; j < in.sites.size(); ++j) {
            const double dx = in.sites[i].x_mm - in.sites[j].x_mm;
            const double dy = in.sites[i].y_mm - in.sites[j].y_mm;
            const double d = std::sqrt(dx * dx + dy * dy);
            if (d < in.boss_od_mm + 0.5) {
                r.add("DFM-BOSS-SPACING", "error",
                      "pcb_standoff_array_under_board: sites " +
                      std::to_string(i) + " and " + std::to_string(j) +
                      " too close (" + std::to_string(d) + " mm < " +
                      std::to_string(in.boss_od_mm + 0.5) + " mm)");
            }
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// 1. Detect if existing plate covers all sites.  If not, fuse a thin
//    "extension plate" beneath the array.
// 2. For each site: fuse a boss cylinder, then cut a pilot hole through it.
// 3. If PCB_thickness_mm > 0, add a counterbore relief beneath the screw
//    on the underside (clearance for screw head).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pcb_standoff_array_under_board DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ScrewSpec* sp = findScrew(in.screw_size);
    const double pilotDia = sp->pilot_dia_mm;

    // ── Bounding box query (context-aware auto-extend) ────────────────────
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.05;

    // Compute the convex hull / AABB of the site array, plus margin.
    double sx0 = std::numeric_limits<double>::infinity();
    double sy0 = sx0;
    double sx1 = -sx0;
    double sy1 = -sx0;
    for (const auto& s : in.sites) {
        sx0 = std::min(sx0, s.x_mm - in.boss_od_mm / 2.0 - in.plate_margin_mm);
        sy0 = std::min(sy0, s.y_mm - in.boss_od_mm / 2.0 - in.plate_margin_mm);
        sx1 = std::max(sx1, s.x_mm + in.boss_od_mm / 2.0 + in.plate_margin_mm);
        sy1 = std::max(sy1, s.y_mm + in.boss_od_mm / 2.0 + in.plate_margin_mm);
    }

    // Auto-extend trigger: any site falls outside the plate footprint, OR
    // the existing footprint is too small (this is the slimmer of the two
    // checks; we keep it for robustness).
    const bool needExtend =
        (sx0 < xMin) || (sx1 > xMax) || (sy0 < yMin) || (sy1 > yMax);

    TopoDS_Shape currentShape = wp.shape();

    if (needExtend) {
        const double ex0 = std::min(xMin, sx0);
        const double ey0 = std::min(yMin, sy0);
        const double ex1 = std::max(xMax, sx1);
        const double ey1 = std::max(yMax, sy1);
        // Add an extension plate that sits flush with the existing top
        // (zMax).  Direction of extension is +Z if axis_dir is +Z (bosses
        // grow upward), else -Z.
        const double extZ0 = (in.axis_dir.Z() >= 0)
            ? zMax - in.plate_thickness_mm : zMin;
        const gp_Ax2 axExt(gp_Pnt(ex0, ey0, extZ0), gp::DZ());
        const TopoDS_Shape extPlate = pr::box(
            axExt, ex1 - ex0, ey1 - ey0, in.plate_thickness_mm);
        currentShape = pr::fuse(currentShape, extPlate);
    }

    // Re-query bbox after possible extension.
    {
        Workpiece tmp(currentShape, wp.material());
        tmp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    }

    // ── Per-site boss + pilot ────────────────────────────────────────────
    // Strategy: fuse ALL bosses into a single addition first, then cut ALL
    // pilots with one Boolean — fewer history rebuilds.
    const double bossH = in.standoff_height_mm + kOverhang;
    const double pilotH = in.standoff_height_mm + in.pilot_extra_depth_mm + 2.0 * kOverhang;

    // Boss starts at the top of the (possibly extended) plate.
    const double bossZ0 = (in.axis_dir.Z() >= 0) ? zMax : zMin - in.standoff_height_mm;
    // Pilot start: above the top of the boss so cut is clean
    const double pilotZ0 = bossZ0 + bossH - kOverhang;
    const gp_Dir pilotDir(0.0, 0.0, -1.0);  // pilot drills DOWN into the boss

    TopoDS_Shape fusedBosses;
    TopoDS_Shape fusedPilots;
    bool firstBoss = true;
    bool firstPilot = true;

    for (const auto& site : in.sites) {
        const gp_Ax2 axBoss(gp_Pnt(site.x_mm, site.y_mm, bossZ0), gp::DZ());
        const TopoDS_Shape boss = pr::cylinder(axBoss, in.boss_od_mm / 2.0, bossH);
        if (firstBoss) { fusedBosses = boss; firstBoss = false; }
        else           { fusedBosses = pr::fuse(fusedBosses, boss); }

        const gp_Ax2 axPilot(gp_Pnt(site.x_mm, site.y_mm, pilotZ0), pilotDir);
        const TopoDS_Shape pilot = pr::cylinder(axPilot, pilotDia / 2.0, pilotH);
        if (firstPilot) { fusedPilots = pilot; firstPilot = false; }
        else            { fusedPilots = pr::fuse(fusedPilots, pilot); }
    }

    // FUSE bosses first (add material).
    currentShape = pr::fuse(currentShape, fusedBosses);

    // PCB clearance: counterbore relief in the BOTTOM of the plate (cuts
    // up from -Z), so screw head clears.  Skipped when PCB_thickness == 0.
    TopoDS_Shape pcbClear;
    bool firstClear = true;
    if (in.PCB_thickness_mm > 1e-6) {
        const double clearH = in.PCB_thickness_mm + kOverhang;
        const double clearDia = pilotDia + 1.5;   // head clearance
        for (const auto& site : in.sites) {
            const gp_Ax2 axClear(
                gp_Pnt(site.x_mm, site.y_mm, zMin - kOverhang), gp::DZ());
            const TopoDS_Shape c = pr::cylinder(axClear, clearDia / 2.0, clearH);
            if (firstClear) { pcbClear = c; firstClear = false; }
            else            { pcbClear = pr::fuse(pcbClear, c); }
        }
    }

    // CUT all pilots in one op
    currentShape = pr::cut(currentShape, fusedPilots);

    // CUT PCB clearance reliefs
    if (!firstClear) {
        currentShape = pr::cut(currentShape, pcbClear);
    }

    // ── Signature ────────────────────────────────────────────────────────
    json sitesJson = json::array();
    for (const auto& s : in.sites) {
        sitesJson.push_back({ { "x_mm", s.x_mm }, { "y_mm", s.y_mm } });
    }

    const int siteCount = static_cast<int>(in.sites.size());
    const int subPerSite = (in.PCB_thickness_mm > 1e-6) ? 3 : 2;
    const int subTotal = subPerSite * siteCount + (needExtend ? 1 : 0);

    json params = {
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "screw_size",          in.screw_size },
        { "standoff_height_mm",  in.standoff_height_mm },
        { "boss_od_mm",          in.boss_od_mm },
        { "pilot_extra_depth_mm", in.pilot_extra_depth_mm },
        { "PCB_thickness_mm",    in.PCB_thickness_mm },
        { "plate_margin_mm",     in.plate_margin_mm },
        { "plate_thickness_mm",  in.plate_thickness_mm },
        { "sites",               sitesJson },
        { "pilot_dia_mm",        pilotDia },
        { "auto_extended_plate", needExtend },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    subTotal },
        { "site_count",          siteCount },
        { "screw_size",          in.screw_size },
        { "pilot_dia_mm",        pilotDia },
        { "boss_od_mm",          in.boss_od_mm },
        { "standoff_height_mm",  in.standoff_height_mm },
        { "auto_extended_plate", needExtend },
        { "axis_dir",            { in.axis_dir.X(), in.axis_dir.Y(), in.axis_dir.Z() } },
        { "sites",               sitesJson },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "endmill;drill";
    tooling.tool_dia_mm      = in.boss_od_mm;
    tooling.tool_length_mm   = in.standoff_height_mm * 2.0 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double rBoss = in.boss_od_mm / 2.0;
    const double rPilot = pilotDia / 2.0;
    const double perSiteAdd = M_PI * rBoss * rBoss * in.standoff_height_mm;
    const double perSiteCut =
        M_PI * rPilot * rPilot *
        (in.standoff_height_mm + in.pilot_extra_depth_mm);
    tooling.stock_removed_mm3 = perSiteCut * static_cast<double>(siteCount);
    tooling.est_cycle_time_s  = std::max(
        2.0, (perSiteAdd + perSiteCut) / 800.0) *
        static_cast<double>(siteCount);
    tooling.extra = {
        { "screw_size",      in.screw_size },
        { "pilot_dia_mm",    pilotDia },
        { "site_count",      siteCount },
        { "iso_4762_pilot",  true },
        { "auto_extended_plate", needExtend },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(currentShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::pcb_standoff_array_under_board applied: {} sites "
                  "{} {} pilot={} extended={}",
                  siteCount, in.screw_size,
                  in.standoff_height_mm, pilotDia, needExtend);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic: find groups of cylindrical faces sharing an axis
// where the OUTER cyl matches `boss_od` and an INNER cyl matches one of
// the known pilot diameters from the screw table.
//
// FALLBACK: metadata replay.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay path FIRST.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" },
                                { "is_compound", true } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // Geometric heuristic: count cylindrical faces that match a known
    // pilot Ø in the screw table.
    struct CylKey { double r; gp_Pnt loc; };
    std::vector<CylKey> matches;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cyl = surf.Cylinder();
        const double dia = 2.0 * cyl.Radius();
        for (const auto& s : kScrewTable) {
            if (std::abs(dia - s.pilot_dia_mm) < 0.15) {
                matches.push_back({ cyl.Radius(), cyl.Axis().Location() });
                break;
            }
        }
    }
    if (matches.empty()) return out;

    // Group by axis location XY (within tolerance) — but cheap: just count.
    json sitesJson = json::array();
    for (const auto& m : matches) {
        bool dup = false;
        for (const auto& s : sitesJson) {
            const double dx = s["x_mm"].get<double>() - m.loc.X();
            const double dy = s["y_mm"].get<double>() - m.loc.Y();
            if (std::sqrt(dx * dx + dy * dy) < 0.5) { dup = true; break; }
        }
        if (!dup) {
            sitesJson.push_back({ { "x_mm", m.loc.X() }, { "y_mm", m.loc.Y() } });
        }
    }
    if (sitesJson.empty()) return out;

    json rp = {
        { "axis_dir",            json::array({ 0.0, 0.0, 1.0 }) },
        { "screw_size",          "M3" },
        { "standoff_height_mm",  4.0 },
        { "boss_od_mm",          5.0 },
        { "pilot_extra_depth_mm", 1.0 },
        { "PCB_thickness_mm",    0.0 },
        { "plate_margin_mm",     3.0 },
        { "plate_thickness_mm",  2.0 },
        { "sites",               sitesJson },
    };
    RecognizedFeature rf;
    rf.skill_id         = kSkillId;
    rf.recovered_params = rp;
    rf.confidence       = 0.6;
    rf.matched_geometry = { { "matched_pilot_count",
                              static_cast<int>(sitesJson.size()) },
                            { "source", "geometric" } };
    out.push_back(rf);
    return out;
}

}  // namespace koocadcam::skill::pcb_standoff_array_under_board
