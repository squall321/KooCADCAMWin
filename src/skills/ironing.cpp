// @lat: [[engine/skills#ironing]]

#include "ironing.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <optional>

namespace koocadcam::skill::ironing {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Best-effort identification of the cup's outer cylindrical wall.
// Returns the face index, axis location, radius, and z-span [zLow, zHigh].
struct CupWall
{
    int     faceIdx = -1;
    gp_Pnt  axisLocation;
    gp_Dir  axisDir;
    double  outerRadius   = 0.0;
    double  innerRadius   = 0.0;   // inferred; defaults to outerRadius (no inner found)
    double  zLow          = 0.0;   // outer cylinder bottom Z
    double  zHigh         = 0.0;   // outer cylinder top Z
    double  innerZLow     = 0.0;   // inner cavity floor Z (above zLow because of cup bottom)
    double  innerZHigh    = 0.0;
};

std::optional<CupWall> findCupWall(const Workpiece& wp,
                                   bool usePositionHint,
                                   double hintX, double hintY)
{
    // Gather all vertical-axis cylinders.
    struct C { int fIdx; double radius; gp_Pnt loc; gp_Dir dir; double zLo; double zHi; };
    std::vector<C> verts;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        BRepAdaptor_Surface surf(wp.face(fIdx));
        const gp_Cylinder cyl = surf.Cylinder();
        const gp_Dir adir = cyl.Axis().Direction();
        if (std::abs(adir.Z()) < 0.9) continue;

        std::vector<double> zCircles;
        for (TopExp_Explorer exp(wp.face(fIdx), TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            zCircles.push_back(crv.Circle().Location().Z());
        }
        if (zCircles.size() < 2) continue;
        const double zLo = *std::min_element(zCircles.begin(), zCircles.end());
        const double zHi = *std::max_element(zCircles.begin(), zCircles.end());
        verts.push_back({ fIdx, cyl.Radius(), cyl.Axis().Location(), adir, zLo, zHi });
    }
    if (verts.empty()) return std::nullopt;

    // Group by approximate axis location (within 1 mm).  For each group,
    // identify outer (= larger radius) and inner (= smaller radius).
    std::vector<bool> used(verts.size(), false);
    int bestGroupOuter = -1;
    int bestGroupInner = -1;
    double bestOuterR = 0.0;
    for (size_t i = 0; i < verts.size(); ++i) {
        if (used[i]) continue;
        const auto& vi = verts[i];
        if (usePositionHint) {
            if (std::hypot(vi.loc.X() - hintX, vi.loc.Y() - hintY) > 5.0)
                continue;
        }
        double maxR = vi.radius;
        int    maxIdx = (int)i;
        double minR = vi.radius;
        int    minIdx = (int)i;
        used[i] = true;
        for (size_t j = i + 1; j < verts.size(); ++j) {
            if (used[j]) continue;
            const auto& vj = verts[j];
            if (vi.loc.Distance(vj.loc) > 1.0) continue;
            used[j] = true;
            if (vj.radius > maxR) { maxR = vj.radius; maxIdx = (int)j; }
            if (vj.radius < minR) { minR = vj.radius; minIdx = (int)j; }
        }
        // Take the group with the largest outer radius as the cup.
        if (maxR > bestOuterR) {
            bestOuterR = maxR;
            bestGroupOuter = maxIdx;
            bestGroupInner = (minIdx == maxIdx) ? -1 : minIdx;
        }
    }
    if (bestGroupOuter < 0) return std::nullopt;

    const auto& v = verts[bestGroupOuter];
    CupWall w;
    w.faceIdx     = v.fIdx;
    w.axisLocation= v.loc;
    w.axisDir     = v.dir;
    w.outerRadius = v.radius;
    w.zLow        = v.zLo;
    w.zHigh       = v.zHi;
    if (bestGroupInner >= 0) {
        const auto& vi = verts[bestGroupInner];
        w.innerRadius = vi.radius;
        w.innerZLow   = vi.zLo;
        w.innerZHigh  = vi.zHi;
    } else {
        // No inner cylinder visible — assume thin wall = smallest bbox extent.
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double t = std::min({ xMax - xMin, yMax - yMin, zMax - zMin });
        w.innerRadius = std::max(0.0, w.outerRadius - t);
        // Without an inner cylinder, default the cavity span = the outer.
        w.innerZLow   = v.zLo + t;     // assume cup bottom of thickness t
        w.innerZHigh  = v.zHi;
    }
    return w;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.wall_reduction_pct <= 0.0 || in.wall_reduction_pct > 100.0) {
        r.add("DFM-INPUT", "error",
              "ironing wall_reduction_pct " +
              std::to_string(in.wall_reduction_pct) +
              " out of (0, 100]");
    }
    if (in.wall_reduction_pct > 50.0) {
        r.add("DFM-IR-PASS", "error",
              "ironing reduction " + std::to_string(in.wall_reduction_pct) +
              "% > 50% per pass — wall fracture risk");
    }

    // Need a cup workpiece — find a vertical cylindrical wall first.
    auto cup = findCupWall(wp, in.use_position_hint,
                           in.cup_position_x_hint_mm,
                           in.cup_position_y_hint_mm);
    if (!cup) {
        r.add("DFM-INPUT", "error",
              "ironing: no cup-like cylindrical wall found in workpiece");
        return r;
    }

    const double currentWall =
        std::max(0.0, cup->outerRadius - cup->innerRadius);
    const double newWall = currentWall * (1.0 - in.wall_reduction_pct / 100.0);
    if (newWall < 0.4) {
        r.add("DFM-IR-MIN-WALL", "error",
              "ironing: resulting wall " + std::to_string(newWall) +
              " mm < 0.4 mm — wall fracture risk");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Increase the inner radius of the cup by (outerR - innerR) * reduction/100
// (i.e. the wall thickness reduces by that percentage).  Implementation:
// cut a slightly larger inner cylinder from the workpiece, restricted to
// the cup's vertical span.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "ironing DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto cupOpt = findCupWall(wp, in.use_position_hint,
                              in.cup_position_x_hint_mm,
                              in.cup_position_y_hint_mm);
    if (!cupOpt) throw SkillError("ironing: cup wall not found");
    const CupWall& cup = *cupOpt;

    const double currentWall = std::max(0.0, cup.outerRadius - cup.innerRadius);
    const double wallReduction = currentWall * (in.wall_reduction_pct / 100.0);
    const double newInnerRadius = cup.innerRadius + wallReduction;
    if (newInnerRadius >= cup.outerRadius) {
        throw SkillError("ironing: computed inner radius >= outer — invalid");
    }

    // Cut tool: a cylinder of newInnerRadius spanning the cup's CAVITY
    // vertical extent (from inner cavity floor up to the cup's top lip).
    // Critically we DON'T extend the cutter down to the outer cylinder's
    // zLow because that would slice through the cup's bottom disc.  The
    // cutter starts at innerZLow (cavity floor) which preserves the bottom.
    const double overhang = 0.05;
    const gp_Ax2 cutAx(
        gp_Pnt(cup.axisLocation.X(), cup.axisLocation.Y(),
               cup.innerZLow),
        gp::DZ());
    TopoDS_Shape cutter;
    try {
        const double cutterH = (cup.zHigh - cup.innerZLow) + overhang;
        cutter = pr::cylinder(cutAx, newInnerRadius, cutterH);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("ironing: cutter build failed: ") + ex.what());
    }
    TopoDS_Shape result;
    try {
        result = pr::cut(wp.shape(), cutter);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("ironing: cut failed: ") + ex.what());
    }

    const double newWall = cup.outerRadius - newInnerRadius;

    // Signature.
    json params = {
        { "wall_reduction_pct",       in.wall_reduction_pct },
        { "use_position_hint",        in.use_position_hint },
        { "cup_position_x_hint_mm",   in.cup_position_x_hint_mm },
        { "cup_position_y_hint_mm",   in.cup_position_y_hint_mm },
    };
    json pattern = {
        { "kind",                     kSkillId },
        { "outer_radius_mm",          cup.outerRadius },
        { "pre_wall_thickness_mm",    currentWall },
        { "post_wall_thickness_mm",   newWall },
        { "wall_reduction_pct",       in.wall_reduction_pct },
        { "cup_position_x_mm",        cup.axisLocation.X() },
        { "cup_position_y_mm",        cup.axisLocation.Y() },
        { "cup_z_low_mm",             cup.zLow },
        { "cup_z_high_mm",            cup.zHigh },
        { "cylindrical_wall_pair",    true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "ironing_die";
    tooling.tool_dia_mm       = 2.0 * cup.outerRadius;
    tooling.tool_length_mm    = (cup.zHigh - cup.zLow) + 5.0;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;     // forming, not removal (rigorously)
    tooling.est_cycle_time_s  = 3.0 + in.wall_reduction_pct * 0.1;
    tooling.extra["machining_constraint"] =
        "Ironing — cup pushed through a die of slightly smaller ID; "
        "wall thins uniformly; lubrication essential";

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(result, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::ironing applied: wall {} → {} ({}% reduction)",
                  currentWall, newWall, in.wall_reduction_pct);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Identify a vertical-axis outer cylinder paired with a slightly smaller
// inner cylindrical cavity at the same axis location.  Confidence rises
// if the workpiece feature history contains a prior `ironing` step OR if
// the post-wall is < 75% of the sheet thickness (= clear evidence of
// thinning by ironing rather than mere deep-draw).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    auto cup = findCupWall(wp, false, 0.0, 0.0);
    if (!cup) return out;
    if (cup->outerRadius <= cup->innerRadius) return out;
    const double wall = cup->outerRadius - cup->innerRadius;
    if (wall <= 0.0) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    // Reference "sheet thickness" = some other bbox extent (the cup wall
    // must be thinner than the reference for ironing to be evident).
    const double bboxMin = std::min({ xMax - xMin, yMax - yMin, zMax - zMin });

    // History-aware: check if there's already an ironing entry → high confidence.
    double historyConf = 0.0;
    for (const auto& f : wp.features()) {
        if (f.skill_id == kSkillId) { historyConf = 0.25; break; }
    }

    // Standard ironing: wall < bbox-min by clear margin (> 20%).
    double confidence = 0.5;
    if (bboxMin > 0.0 && wall < bboxMin * 0.8) confidence += 0.2;
    confidence += historyConf;
    confidence = std::clamp(confidence, 0.0, 0.95);

    json recovered = {
        { "wall_reduction_pct",       0.0 },    // unknown without history
        { "use_position_hint",        true },
        { "cup_position_x_hint_mm",   cup->axisLocation.X() },
        { "cup_position_y_hint_mm",   cup->axisLocation.Y() },
    };
    json matched = {
        { "outer_radius_mm",      cup->outerRadius },
        { "inner_radius_mm",      cup->innerRadius },
        { "post_wall_mm",         wall },
        { "cup_face_id",          cup->faceIdx },
        { "cup_z_low_mm",         cup->zLow },
        { "cup_z_high_mm",        cup->zHigh },
        { "bbox_min_mm",          bboxMin },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, confidence, matched });
    return out;
}

}  // namespace koocadcam::skill::ironing
