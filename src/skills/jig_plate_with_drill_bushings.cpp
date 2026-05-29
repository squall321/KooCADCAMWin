// @lat: [[engine/skills#jig_plate_with_drill_bushings]]

#include "jig_plate_with_drill_bushings.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::jig_plate_with_drill_bushings {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// DFM cites DIN 179 (Drill bushings — bored type) and ASME Y14.5.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bushing_id_mm < 0.8) {
        r.add("DFM-002", "error",
              "jig_plate_with_drill_bushings: bushing_id " +
              std::to_string(in.bushing_id_mm) + " mm < 0.8 mm");
    }
    if (in.bushing_od_mm <= in.bushing_id_mm) {
        r.add("DFM-BUSHING-WALL", "error",
              "jig_plate_with_drill_bushings: bushing_od " +
              std::to_string(in.bushing_od_mm) + " ≤ bushing_id " +
              std::to_string(in.bushing_id_mm) + " — invalid bushing geometry");
    }
    // DIN 179: bushing wall ≥ 1 mm — i.e. (OD − ID)/2 ≥ 1 mm
    const double wallMm = (in.bushing_od_mm - in.bushing_id_mm) / 2.0;
    if (in.bushing_od_mm > in.bushing_id_mm && wallMm < 1.0) {
        r.add("DFM-BUSHING-WALL", "error",
              "jig_plate_with_drill_bushings: bushing wall " +
              std::to_string(wallMm) + " mm < DIN 179 min 1 mm");
    }
    if (in.flange_od_mm <= in.bushing_od_mm) {
        r.add("DFM-FLANGE", "error",
              "jig_plate_with_drill_bushings: flange_od " +
              std::to_string(in.flange_od_mm) + " ≤ bushing_od " +
              std::to_string(in.bushing_od_mm) + " — no shoulder");
    }
    if (in.flange_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "jig_plate_with_drill_bushings: flange_depth must be > 0");
    }
    if (in.chamfer_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "jig_plate_with_drill_bushings: chamfer_mm must be ≥ 0");
    }
    if (in.sites.empty()) {
        r.add("DFM-INPUT", "error",
              "jig_plate_with_drill_bushings: at least 1 bushing site required");
    }

    // ASME Y14.5: pairwise spacing ≥ flange_od + 1 mm so seats do not overlap.
    for (size_t i = 0; i < in.sites.size(); ++i) {
        for (size_t j = i + 1; j < in.sites.size(); ++j) {
            const double dx = in.sites[i].x_mm - in.sites[j].x_mm;
            const double dy = in.sites[i].y_mm - in.sites[j].y_mm;
            const double d  = std::sqrt(dx * dx + dy * dy);
            if (d < in.flange_od_mm + 1.0) {
                r.add("DFM-BUSHING-SPACING", "error",
                      "jig_plate_with_drill_bushings: sites " +
                      std::to_string(i) + " and " + std::to_string(j) +
                      " too close (" + std::to_string(d) +
                      " mm < flange_od+1 = " +
                      std::to_string(in.flange_od_mm + 1.0) + " mm)");
            }
        }
    }

    // Plate thickness must accommodate at least flange + 2 mm guide.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness > 0.0 && thickness < in.flange_depth_mm + 2.0) {
        r.add("DFM-BUSHING-THICKNESS", "error",
              "jig_plate_with_drill_bushings: plate thickness " +
              std::to_string(thickness) + " < flange_depth + 2 mm");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Chain per site: pilot through cyl + seat counterbore + chamfer cone.
// FUSE all three concentric cutters, then a single Boolean CUT.  We loop
// over all sites and accumulate one big cutter compound via repeated fuse,
// then do one cut at the end.  This avoids N STEP-history rebuilds.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "jig_plate_with_drill_bushings DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError(
        "jig_plate_with_drill_bushings: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Build one large fused cutter spanning all sites.  We start the chain
    // with the very first site's pilot, then FUSE in each subsequent
    // primitive.  This is the same idiom counterbore.cpp uses for its 2
    // concentric cylinders.
    TopoDS_Shape fused;
    bool first = true;

    for (const auto& site : in.sites) {
        gp_Pnt toolStart;
        if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
            const double entryZ = (adir.Z() < 0) ? zMax : zMin;
            toolStart = gp_Pnt(site.x_mm, site.y_mm,
                               entryZ - adir.Z() * kOverhang);
        } else {
            toolStart = gp_Pnt(site.x_mm, site.y_mm,
                               (zMin + zMax) / 2.0 - adir.Z() * kOverhang);
        }

        const gp_Ax2 ax(toolStart, adir);
        // Sub-feature 1: pilot — through plate
        const TopoDS_Shape pilot =
            pr::cylinder(ax, in.bushing_id_mm / 2.0, thickness + 2.0 * kOverhang);
        // Sub-feature 2: seat — counterbore at entry
        const TopoDS_Shape seat =
            pr::cylinder(ax, in.bushing_od_mm / 2.0,
                         in.flange_depth_mm + kOverhang);
        // Sub-feature 3: flange counterbore (wider) — flange_od at entry,
        // a shallow recess so the flange of the bushing sits flush.
        const double flangeRecessDepth = std::max(0.1, in.chamfer_mm + 0.5);
        const TopoDS_Shape flangeSeat =
            pr::cylinder(ax, in.flange_od_mm / 2.0,
                         flangeRecessDepth + kOverhang);
        // Sub-feature 4: chamfer (a 45° cone) at entry to ease bushing press-in.
        const double chamferTopR = in.bushing_od_mm / 2.0 + in.chamfer_mm;
        const double chamferBotR = in.bushing_od_mm / 2.0;
        TopoDS_Shape chamfer;
        if (in.chamfer_mm > 1e-6) {
            chamfer = pr::coneFrustum(ax, chamferTopR, chamferBotR, in.chamfer_mm);
        }

        TopoDS_Shape siteCutter = pr::fuse(pilot, seat);
        siteCutter = pr::fuse(siteCutter, flangeSeat);
        if (!chamfer.IsNull()) siteCutter = pr::fuse(siteCutter, chamfer);

        if (first) { fused = siteCutter; first = false; }
        else       { fused = pr::fuse(fused, siteCutter); }
    }

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // ── Signature ────────────────────────────────────────────────────────
    json sitesJson = json::array();
    for (const auto& s : in.sites) {
        sitesJson.push_back({ { "x_mm", s.x_mm }, { "y_mm", s.y_mm } });
    }
    json params = {
        { "entry_face_id",   *entryId },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "bushing_id_mm",   in.bushing_id_mm },
        { "bushing_od_mm",   in.bushing_od_mm },
        { "flange_od_mm",    in.flange_od_mm },
        { "flange_depth_mm", in.flange_depth_mm },
        { "chamfer_mm",      in.chamfer_mm },
        { "sites",           sitesJson },
    };
    const int subPerSite = (in.chamfer_mm > 1e-6) ? 4 : 3;
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     subPerSite * static_cast<int>(in.sites.size()) },
        { "site_count",           static_cast<int>(in.sites.size()) },
        { "subfeatures_per_site", subPerSite },
        { "bushing_id_mm",        in.bushing_id_mm },
        { "bushing_od_mm",        in.bushing_od_mm },
        { "flange_od_mm",         in.flange_od_mm },
        { "flange_depth_mm",      in.flange_depth_mm },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
        { "sites",                sitesJson },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "drill;counterbore;chamfer";
    tooling.tool_dia_mm      = in.flange_od_mm;
    tooling.tool_length_mm   = thickness * 1.5 + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double rPilot  = in.bushing_id_mm / 2.0;
    const double rSeat   = in.bushing_od_mm / 2.0;
    const double rFlange = in.flange_od_mm  / 2.0;
    const double flangeRecess = std::max(0.1, in.chamfer_mm + 0.5);
    const double perSiteVol =
        M_PI * rPilot * rPilot * thickness +
        M_PI * (rSeat   * rSeat   - rPilot * rPilot) * in.flange_depth_mm +
        M_PI * (rFlange * rFlange - rSeat  * rSeat ) * flangeRecess;
    tooling.stock_removed_mm3 = perSiteVol * static_cast<double>(in.sites.size());
    tooling.est_cycle_time_s  = std::max(2.0, perSiteVol / 1000.0) *
                                static_cast<double>(in.sites.size());
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },       { "tool_dia_mm", in.bushing_id_mm } },
            { { "tool_type", "counterbore" }, { "tool_dia_mm", in.bushing_od_mm },
              { "depth_mm", in.flange_depth_mm } },
            { { "tool_type", "counterbore" }, { "tool_dia_mm", in.flange_od_mm },
              { "depth_mm", flangeRecess } },
            { { "tool_type", "chamfer" },     { "tool_dia_mm",
              in.bushing_od_mm + 2.0 * in.chamfer_mm },
              { "depth_mm", in.chamfer_mm } },
        } },
        { "site_count",      static_cast<int>(in.sites.size()) },
        { "bushing_standard", "DIN_179_bored_type" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::jig_plate_with_drill_bushings applied: {} sites, "
                  "id={} od={} flange_od={}",
                  in.sites.size(), in.bushing_id_mm, in.bushing_od_mm,
                  in.flange_od_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic (PRIMARY): a jig plate site looks like a counterbore
// where there's an EXTRA outer cylindrical recess (the flange seat).  We
// look for triples (pilot, seat, flange) of concentric coaxial cylinders.
//
// FALLBACK: metadata replay from wp.features() when no triples are found.

namespace {

struct CylInfo {
    int    faceIdx = -1;
    double radius  = 0.0;
    gp_Ax1 axis;
};

std::vector<CylInfo> collectCylinders(const Workpiece& wp)
{
    std::vector<CylInfo> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cyl = surf.Cylinder();
        out.push_back({ i, cyl.Radius(), cyl.Axis() });
    }
    return out;
}

bool sameAxis(const CylInfo& a, const CylInfo& b, double angTolDeg = 0.5,
              double posTolMm = 1e-2)
{
    const gp_Dir da = a.axis.Direction();
    const gp_Dir db = b.axis.Direction();
    const double dot = std::abs(da.X() * db.X() + da.Y() * db.Y() + da.Z() * db.Z());
    if (dot < std::cos(angTolDeg * M_PI / 180.0)) return false;
    const gp_Pnt& pa = a.axis.Location();
    const gp_Pnt& pb = b.axis.Location();
    gp_Vec v(pa, pb);
    gp_Vec axV(da.X(), da.Y(), da.Z());
    gp_Vec perp = v - axV * v.Dot(axV);
    return perp.Magnitude() < posTolMm;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay path FIRST (highest confidence).
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

    // Geometric heuristic: look for groups of ≥3 coaxial cylinders with
    // distinct radii (pilot < seat < flange).  Per site we expect 3 cyl
    // faces — group cylinders by shared axis.
    const auto cyls = collectCylinders(wp);
    if (cyls.size() < 3) return out;

    std::vector<bool> consumed(cyls.size(), false);
    std::vector<std::vector<int>> groups;
    for (size_t i = 0; i < cyls.size(); ++i) {
        if (consumed[i]) continue;
        std::vector<int> g; g.push_back(static_cast<int>(i));
        consumed[i] = true;
        for (size_t j = i + 1; j < cyls.size(); ++j) {
            if (consumed[j]) continue;
            if (sameAxis(cyls[i], cyls[j])) {
                g.push_back(static_cast<int>(j));
                consumed[j] = true;
            }
        }
        if (g.size() >= 3) groups.push_back(g);
    }
    if (groups.empty()) return out;

    json sitesJson = json::array();
    double bushingId = 0.0, bushingOd = 0.0, flangeOd = 0.0;
    for (const auto& g : groups) {
        // Sort by radius
        std::vector<double> radii;
        for (int idx : g) radii.push_back(cyls[idx].radius);
        std::sort(radii.begin(), radii.end());
        if (radii.size() < 3) continue;
        bushingId = 2.0 * radii.front();
        bushingOd = 2.0 * radii[1];
        flangeOd  = 2.0 * radii.back();
        const gp_Pnt loc = cyls[g[0]].axis.Location();
        sitesJson.push_back({ { "x_mm", loc.X() }, { "y_mm", loc.Y() } });
    }
    if (sitesJson.empty()) return out;

    json rp = {
        { "axis_dir",        json::array({ 0.0, 0.0, -1.0 }) },
        { "bushing_id_mm",   bushingId },
        { "bushing_od_mm",   bushingOd },
        { "flange_od_mm",    flangeOd },
        { "flange_depth_mm", 3.0 },   // not recoverable from BREP alone
        { "chamfer_mm",      0.5 },
        { "sites",           sitesJson },
    };
    RecognizedFeature rf;
    rf.skill_id         = kSkillId;
    rf.recovered_params = rp;
    rf.confidence       = 0.7;
    rf.matched_geometry = { { "site_groups", static_cast<int>(sitesJson.size()) },
                            { "source", "geometric" } };
    out.push_back(rf);
    return out;
}

}  // namespace koocadcam::skill::jig_plate_with_drill_bushings
