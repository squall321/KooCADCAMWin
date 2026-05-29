// @lat: [[engine/skills#locator_pin_set]]

#include "locator_pin_set.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::locator_pin_set {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// DFM cites DIN 6321 (Round locating pins), ASME B94.6 (Pins for plug
// gauges, dowel-pin sizing) and the Carr Lane fixture design handbook.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.post_dia_mm < 0.8) {
        r.add("DFM-002", "error",
              "locator_pin_set: post_dia " + std::to_string(in.post_dia_mm) +
              " mm < 0.8 mm");
    }
    if (in.shoulder_dia_mm <= in.post_dia_mm) {
        r.add("DFM-PIN-SHOULDER", "error",
              "locator_pin_set: shoulder_dia " +
              std::to_string(in.shoulder_dia_mm) + " ≤ post_dia " +
              std::to_string(in.post_dia_mm) + " — no shoulder formed");
    }
    if (in.post_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "locator_pin_set: post_height must be > 0");
    }
    if (in.shoulder_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "locator_pin_set: shoulder_height must be > 0");
    }
    // DIN 6321 — post slenderness ≤ 6 (post_height / post_dia)
    const double slenderness =
        (in.post_dia_mm > 0.0) ? (in.post_height_mm / in.post_dia_mm) : 0.0;
    if (slenderness > 6.0) {
        r.add("DFM-PIN-SLENDER", "error",
              "locator_pin_set: post slenderness " +
              std::to_string(slenderness) + " > 6 (DIN 6321 buckling limit)");
    }
    if (in.lead_in_chamfer_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "locator_pin_set: lead_in_chamfer must be ≥ 0");
    }
    if (in.lead_in_chamfer_mm > in.post_dia_mm / 2.0) {
        r.add("DFM-PIN-CHAMFER", "error",
              "locator_pin_set: lead_in_chamfer " +
              std::to_string(in.lead_in_chamfer_mm) +
              " > post_radius — chamfer larger than pin");
    }
    // Groove geometry
    if (in.groove_dia_mm <= 0.0 || in.groove_dia_mm >= in.post_dia_mm) {
        r.add("DFM-PIN-GROOVE", "error",
              "locator_pin_set: groove_dia " + std::to_string(in.groove_dia_mm) +
              " must satisfy 0 < groove_dia < post_dia");
    }
    if (in.groove_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "locator_pin_set: groove_width must be > 0");
    }
    // DIN 471 retaining ring grooves: minimum wall ((post_dia − groove_dia)/2)
    // ≥ 0.4 mm so the groove edge doesn't tear under E-clip preload.
    const double grooveWall = (in.post_dia_mm - in.groove_dia_mm) / 2.0;
    if (in.groove_dia_mm > 0.0 && in.groove_dia_mm < in.post_dia_mm &&
        grooveWall < 0.4) {
        r.add("DFM-PIN-GROOVE", "error",
              "locator_pin_set: groove wall " + std::to_string(grooveWall) +
              " < DIN 471 min 0.4 mm");
    }
    if (in.groove_z_offset_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "locator_pin_set: groove_z_offset must be ≥ 0");
    }
    if (in.groove_z_offset_mm + in.groove_width_mm > in.post_height_mm) {
        r.add("DFM-PIN-GROOVE", "error",
              "locator_pin_set: groove extends past post top");
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// 4-step compound chain:
//   1. FUSE  shoulder cylinder onto plate
//   2. FUSE  post cylinder onto result (above shoulder)
//   3. CUT   45° lead-in chamfer (cone frustum) from top of post
//   4. CUT   groove (annular ring) from post side

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "locator_pin_set DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto baseId = wp.resolve(in.base_face);
    if (!baseId) throw SkillError(
        "locator_pin_set: base_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const gp_Dir adir = in.axis_dir;

    // Pin grows from entry plane along +adir.  We assume +Z base for slice 8.
    if (std::abs(adir.X()) > 1e-6 || std::abs(adir.Y()) > 1e-6) {
        throw SkillError(
            "locator_pin_set: only axis-aligned axis_dir supported in slice 8");
    }
    const double baseZ = (adir.Z() > 0) ? zMax : zMin;
    const double pinSign = (adir.Z() > 0) ? 1.0 : -1.0;
    const double kOverhang = 0.05;

    // Sub-feature 1: shoulder cylinder, base on plate top.
    gp_Pnt shoulderBase(in.position_x_mm, in.position_y_mm,
                        baseZ - pinSign * kOverhang);
    const gp_Ax2 shoulderAx(shoulderBase, adir);
    const TopoDS_Shape shoulder = pr::cylinder(
        shoulderAx, in.shoulder_dia_mm / 2.0,
        in.shoulder_height_mm + kOverhang);

    // Sub-feature 2: post cylinder, base at top of shoulder.
    const double postBaseZ = baseZ + pinSign * in.shoulder_height_mm;
    gp_Pnt postBase(in.position_x_mm, in.position_y_mm, postBaseZ);
    const gp_Ax2 postAx(postBase, adir);
    const TopoDS_Shape post = pr::cylinder(
        postAx, in.post_dia_mm / 2.0, in.post_height_mm);

    // Fuse shoulder + post onto the plate.
    TopoDS_Shape current = pr::fuse(wp.shape(), shoulder);
    current = pr::fuse(current, post);

    // Sub-feature 3: lead-in chamfer at the very top of the post.  Model
    // it as a cone frustum that, when subtracted, removes the OUTER
    // corner of the top of the post.  The cone has:
    //   • larger radius (post_r + overhang) at the TOP (above the post)
    //   • smaller radius (post_r − chamfer) at the BASE (chamfer_mm
    //     below the top of the post)
    // gp_Ax2's V direction is +Z; coneFrustum height extrudes from base
    // upward by `height`.  We position the base at post_top − chamfer.
    // The cone's outer surface meets the post outer wall at the base
    // (r=post_r−chamfer < post_r → cone is inside post at base) and
    // exits laterally by the top.  The Boolean cut therefore removes a
    // diagonal slice across the top corner — exactly a 45° chamfer.
    if (in.lead_in_chamfer_mm > 1e-6) {
        const double topZ  = postBaseZ + pinSign * in.post_height_mm;
        const double chamferBaseZ = topZ - pinSign * in.lead_in_chamfer_mm;
        gp_Pnt chamferOrigin(in.position_x_mm, in.position_y_mm,
                             chamferBaseZ);
        const gp_Ax2 chamferAx(chamferOrigin, adir);
        const double rBase = std::max(0.05,
                                      in.post_dia_mm / 2.0 - in.lead_in_chamfer_mm);
        const double rTop  = in.post_dia_mm / 2.0 + kOverhang;
        const TopoDS_Shape chamferCone =
            pr::coneFrustum(chamferAx, rBase, rTop,
                            in.lead_in_chamfer_mm + kOverhang);
        current = pr::cut(current, chamferCone);
    }

    // Sub-feature 4: groove (annular ring) for E-clip retaining ring.
    {
        const double grooveBaseZ = postBaseZ + pinSign * in.groove_z_offset_mm;
        gp_Pnt grooveBase(in.position_x_mm, in.position_y_mm,
                          grooveBaseZ - pinSign * 0.0);
        const gp_Ax2 grooveAx(grooveBase, adir);
        // outer = post_r + a bit (so we cut through the post outer wall);
        // inner = groove_dia/2 (= groove root); height = groove_width
        const TopoDS_Shape grooveCutter =
            pr::annularRing(grooveAx,
                            in.post_dia_mm / 2.0 + kOverhang,
                            in.groove_dia_mm / 2.0,
                            in.groove_width_mm);
        current = pr::cut(current, grooveCutter);
    }

    const TopoDS_Shape newShape = current;

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "base_face_id",        *baseId },
        { "position_x_mm",       in.position_x_mm },
        { "position_y_mm",       in.position_y_mm },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
        { "shoulder_dia_mm",     in.shoulder_dia_mm },
        { "shoulder_height_mm",  in.shoulder_height_mm },
        { "post_dia_mm",         in.post_dia_mm },
        { "post_height_mm",      in.post_height_mm },
        { "lead_in_chamfer_mm",  in.lead_in_chamfer_mm },
        { "groove_dia_mm",       in.groove_dia_mm },
        { "groove_width_mm",     in.groove_width_mm },
        { "groove_z_offset_mm",  in.groove_z_offset_mm },
    };
    const int subCount = (in.lead_in_chamfer_mm > 1e-6) ? 4 : 3;
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    subCount },
        { "post_dia_mm",         in.post_dia_mm },
        { "post_height_mm",      in.post_height_mm },
        { "shoulder_dia_mm",     in.shoulder_dia_mm },
        { "shoulder_height_mm",  in.shoulder_height_mm },
        { "groove_dia_mm",       in.groove_dia_mm },
        { "groove_width_mm",     in.groove_width_mm },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "turn;groove;chamfer";
    tooling.tool_dia_mm      = in.shoulder_dia_mm;
    tooling.tool_length_mm   = in.shoulder_height_mm + in.post_height_mm + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 400.0;
    tooling.feed_per_tooth_mm = 0.04;
    const double rPost     = in.post_dia_mm     / 2.0;
    const double rShoulder = in.shoulder_dia_mm / 2.0;
    const double rGroove   = in.groove_dia_mm   / 2.0;
    // Material ADDED = shoulder volume + post volume
    const double addedVol =
        M_PI * rShoulder * rShoulder * in.shoulder_height_mm +
        M_PI * rPost     * rPost     * in.post_height_mm;
    // Material REMOVED = chamfer band + groove ring
    const double removedVol =
        M_PI * ((rPost + in.lead_in_chamfer_mm) * (rPost + in.lead_in_chamfer_mm)
                - rPost * rPost) * in.lead_in_chamfer_mm +
        M_PI * (rPost * rPost - rGroove * rGroove) * in.groove_width_mm;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  = std::max(3.0, addedVol / 2000.0);
    tooling.extra = {
        { "added_volume_mm3", addedVol },
        { "tool_sequence", {
            { { "tool_type", "turn" },    { "operation", "shoulder_OD" },
              { "tool_dia_mm", in.shoulder_dia_mm } },
            { { "tool_type", "turn" },    { "operation", "post_OD" },
              { "tool_dia_mm", in.post_dia_mm } },
            { { "tool_type", "chamfer" }, { "operation", "lead_in_top" },
              { "chamfer_mm",  in.lead_in_chamfer_mm } },
            { { "tool_type", "groove" },  { "operation", "e_clip_groove" },
              { "groove_dia_mm",   in.groove_dia_mm },
              { "groove_width_mm", in.groove_width_mm } },
        } },
        { "pin_standard",       "DIN_6321" },
        { "retaining_standard", "DIN_471" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::locator_pin_set applied: post {}×{}, shoulder {}×{}",
                  in.post_dia_mm, in.post_height_mm,
                  in.shoulder_dia_mm, in.shoulder_height_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic PRIMARY: look for coaxial cylinder triplets where
// the radii follow groove < post < shoulder.  Metadata replay FALLBACK.

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

    // Metadata replay first.
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

    // Geometric heuristic.  Group coaxial cylinders, look for ≥ 3 distinct
    // radii (groove < post < shoulder).
    const auto cyls = collectCylinders(wp);
    if (cyls.size() < 3) return out;

    std::vector<bool> consumed(cyls.size(), false);
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
        if (g.size() < 3) continue;
        std::vector<double> radii;
        for (int idx : g) radii.push_back(cyls[idx].radius);
        std::sort(radii.begin(), radii.end());
        // Distinct radii: at least 3 unique
        std::vector<double> uniq;
        for (double r : radii) {
            if (uniq.empty() || std::abs(r - uniq.back()) > 1e-3) uniq.push_back(r);
        }
        if (uniq.size() < 3) continue;
        const gp_Pnt loc = cyls[g[0]].axis.Location();
        json rp = {
            { "position_x_mm",       loc.X() },
            { "position_y_mm",       loc.Y() },
            { "axis_dir",            json::array({ 0.0, 0.0, 1.0 }) },
            { "groove_dia_mm",       2.0 * uniq.front() },
            { "post_dia_mm",         2.0 * uniq[uniq.size() / 2] },
            { "shoulder_dia_mm",     2.0 * uniq.back() },
            { "post_height_mm",      20.0 },        // not recoverable
            { "shoulder_height_mm",  3.0 },
            { "groove_width_mm",     1.5 },
            { "groove_z_offset_mm",  1.0 },
            { "lead_in_chamfer_mm",  1.0 },
        };
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = rp;
        rf.confidence       = 0.7;
        rf.matched_geometry = { { "coaxial_cyl_count",
                                  static_cast<int>(g.size()) },
                                { "source", "geometric" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::locator_pin_set
