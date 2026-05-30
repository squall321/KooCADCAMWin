// @lat: [[engine/skills#box_section_with_endplate]]

#include "box_section_with_endplate.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::box_section_with_endplate {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.length_mm <= 0.0 || in.width_mm <= 0.0 || in.height_mm <= 0.0 ||
        in.wall_t_mm <= 0.0 || in.endplate_t_mm <= 0.0 || in.bolt_dia_mm <= 0.0)
    {
        r.add("DFM-BOX-INPUT", "error",
              "box_section_with_endplate: all dimensions must be > 0");
        return r;
    }
    if (in.bolt_grid.first <= 0 || in.bolt_grid.second <= 0) {
        r.add("DFM-BOX-INPUT", "error",
              "box_section_with_endplate: bolt_grid components must be > 0");
        return r;
    }

    if (in.wall_t_mm < 1.6) {
        r.add("DFM-BOX-WALL", "error",
              "box_section_with_endplate: wall_t " +
              std::to_string(in.wall_t_mm) +
              " mm < 1.6 mm (EN 10219 SHS/RHS minimum)");
    }
    if (in.wall_t_mm * 2.0 >= std::min(in.width_mm, in.height_mm)) {
        r.add("DFM-BOX-WALL", "error",
              "box_section_with_endplate: 2 × wall_t exceeds smallest dim — "
              "no internal cavity");
    }

    if (in.endplate_t_mm < in.wall_t_mm * 1.5) {
        r.add("DFM-BOX-ENDPLATE-T", "warning",
              "box_section_with_endplate: endplate_t " +
              std::to_string(in.endplate_t_mm) +
              " mm < 1.5 × wall_t — corner weld leg geometry constrained");
    }

    // Bolt edge / pitch (AISC J3.4): min edge_dist >= 2 × bolt_dia for sheared edges.
    if (in.edge_dist_mm < 2.0 * in.bolt_dia_mm) {
        r.add("DFM-BOX-BOLT-PITCH", "error",
              "box_section_with_endplate: edge_dist " +
              std::to_string(in.edge_dist_mm) +
              " mm < 2 × bolt_dia (" + std::to_string(2.0 * in.bolt_dia_mm) +
              " mm) per AISC J3.4");
    }

    // Bolt pattern must fit on the endplate.
    const double availW = in.width_mm  - 2.0 * in.edge_dist_mm;
    const double availH = in.height_mm - 2.0 * in.edge_dist_mm;
    if (in.bolt_grid.first  > 1 && availW < in.bolt_dia_mm) {
        r.add("DFM-BOX-BOLT-PITCH", "error",
              "box_section_with_endplate: bolt grid wider than endplate footprint");
    }
    if (in.bolt_grid.second > 1 && availH < in.bolt_dia_mm) {
        r.add("DFM-BOX-BOLT-PITCH", "error",
              "box_section_with_endplate: bolt grid taller than endplate footprint");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "box_section_with_endplate DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double L  = in.length_mm;
    const double W  = in.width_mm;
    const double H  = in.height_mm;
    const double Tw = in.wall_t_mm;
    const double Tp = in.endplate_t_mm;

    // Sub-feature 1: outer box (centered in Y/Z).
    const gp_Pnt outerOrigin(0.0, -W / 2.0, -H / 2.0);
    const TopoDS_Shape outerBox = pr::box(
        gp_Ax2(outerOrigin, gp::DZ()), L, W, H);

    // Sub-feature 2: inner cavity (slightly longer so cut goes clean through).
    const gp_Pnt innerOrigin(-0.1, -(W / 2.0 - Tw), -(H / 2.0 - Tw));
    const TopoDS_Shape innerBox = pr::box(
        gp_Ax2(innerOrigin, gp::DZ()),
        L + 0.2, W - 2.0 * Tw, H - 2.0 * Tw);

    TopoDS_Shape hollow = pr::cut(outerBox, innerBox);

    // Sub-feature 3 & 4: front + rear endplates fused.
    const gp_Pnt frontOrigin(-Tp, -W / 2.0, -H / 2.0);
    const TopoDS_Shape frontPlate = pr::box(
        gp_Ax2(frontOrigin, gp::DZ()), Tp, W, H);
    const gp_Pnt rearOrigin(L, -W / 2.0, -H / 2.0);
    const TopoDS_Shape rearPlate = pr::box(
        gp_Ax2(rearOrigin, gp::DZ()), Tp, W, H);

    TopoDS_Shape assembled = pr::fuse(hollow, frontPlate);
    assembled = pr::fuse(assembled, rearPlate);

    // Sub-features 5..N: bolt holes through endplates.
    const int nx = in.bolt_grid.first;
    const int ny = in.bolt_grid.second;
    const double r  = in.bolt_dia_mm / 2.0;
    const double yStart = -W / 2.0 + in.edge_dist_mm;
    const double zStart = -H / 2.0 + in.edge_dist_mm;
    const double yStep  = (nx > 1) ? ((W - 2.0 * in.edge_dist_mm) / (nx - 1)) : 0.0;
    const double zStep  = (ny > 1) ? ((H - 2.0 * in.edge_dist_mm) / (ny - 1)) : 0.0;

    std::vector<TopoDS_Shape> drills;
    json holePos = json::array();
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            const double y = yStart + i * yStep;
            const double z = zStart + j * zStep;
            // Front plate drill (axis along +X, going from x=-Tp-0.1 forward).
            const gp_Ax2 fAx(gp_Pnt(-Tp - 0.1, y, z), gp::DX());
            drills.push_back(pr::cylinder(fAx, r, Tp + 0.2));
            // Rear plate drill.
            const gp_Ax2 bAx(gp_Pnt(L - 0.1, y, z), gp::DX());
            drills.push_back(pr::cylinder(bAx, r, Tp + 0.2));
            holePos.push_back({ { "y", y }, { "z", z } });
        }
    }
    if (!drills.empty()) {
        assembled = pr::cutMany(assembled, drills);
    }

    const int nBoltsPerPlate = nx * ny;
    const int subfeatureCount = 4 + 2 * nBoltsPerPlate;

    json params = {
        { "length_mm",     L },
        { "width_mm",      W },
        { "height_mm",     H },
        { "wall_t_mm",     Tw },
        { "endplate_t_mm", Tp },
        { "bolt_grid",     { nx, ny } },
        { "bolt_dia_mm",   in.bolt_dia_mm },
        { "edge_dist_mm",  in.edge_dist_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "subfeature_count",      subfeatureCount },
        { "length_mm",             L },
        { "width_mm",              W },
        { "height_mm",             H },
        { "wall_t_mm",             Tw },
        { "endplate_t_mm",         Tp },
        { "bolt_grid",             { nx, ny } },
        { "bolt_dia_mm",           in.bolt_dia_mm },
        { "n_bolts_per_plate",     nBoltsPerPlate },
        { "n_endplates",           2 },
        { "n_bolt_holes_total",    2 * nBoltsPerPlate },
        { "hole_positions",        holePos },
    };

    const double areaMm2 = W * H - (W - 2.0 * Tw) * (H - 2.0 * Tw);
    const double tubeVol = areaMm2 * L;
    const double plateVol = 2.0 * Tp * W * H;
    const double weightKg = (tubeVol + plateVol) * 7.85e-6;

    ToolingMeta tooling;
    tooling.tool_type         = "tube_form;saw_cut;weld;drill";
    tooling.tool_dia_mm       = in.bolt_dia_mm;
    tooling.tool_length_mm    = L;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.08;
    tooling.stock_removed_mm3 = static_cast<double>(2 * nBoltsPerPlate) *
                                M_PI * r * r * Tp +
                                (innerBox.IsNull() ? 0.0 :
                                 (L) * (W - 2 * Tw) * (H - 2 * Tw));
    tooling.est_cycle_time_s  = 60.0 + 5.0 * (2 * nBoltsPerPlate);
    tooling.extra = {
        { "subfeature_sequence", {
            { { "name", "outer_box" }     },
            { { "name", "inner_cavity" }  },
            { { "name", "front_endplate" }},
            { { "name", "rear_endplate" } },
            { { "name", "bolt_holes" },
              { "count", 2 * nBoltsPerPlate },
              { "dia_mm", in.bolt_dia_mm } },
        } },
        { "section_area_mm2",    areaMm2 },
        { "approx_weight_kg",    weightKg },
        { "standard",            "EN_10219_RHS_AWS_D1.1" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(assembled, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::box_section_with_endplate applied: L={} W={} H={} tw={} tp={} bolts={}",
                  L, W, H, Tw, Tp, 2 * nBoltsPerPlate);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay + geometric fallback) ──────────────────
//
// Geometric signature of a box section + endplate:
//   - 2 large planar end faces (±X normal) at x = -Tp and x = L + Tp ranges
//     (the endplate front/rear faces).
//   - 4 outer side faces (±Y / ±Z normals) for the tube outer.
//   - 4 inner cavity side faces (also planar) at offset wall_t inward — these
//     have OPPOSITE normals from the corresponding outer faces.
//   - N circular edges (bolt holes) penetrating each endplate (cylindrical
//     faces with axis ∥ X-axis).
// We use ALL of these to fingerprint a box section.

namespace {

// Count cylindrical faces whose axis is parallel to X (bolt-hole axis).
int countXAxisCylinders(const Workpiece& wp, double& outRadiusSum,
                        std::vector<double>& outRadii)
{
    int n = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cy = surf.Cylinder();
        const gp_Dir axDir = cy.Axis().Direction();
        if (std::abs(std::abs(axDir.X()) - 1.0) > 1e-3) continue;
        ++n;
        outRadiusSum += cy.Radius();
        outRadii.push_back(cy.Radius());
    }
    return n;
}

std::vector<RecognizedFeature> geometric_fallback(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    if (wp.shape().IsNull()) return out;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double extX = xMax - xMin;
    const double extY = yMax - yMin;
    const double extZ = zMax - zMin;
    if (extX <= 0.0 || extY <= 0.0 || extZ <= 0.0) return out;

    // Box section is "long" — extX > 2 × max(extY, extZ).
    if (extX < 2.0 * std::max(extY, extZ)) return out;

    // Look for cylindrical bolt holes (axis ∥ X).
    double radSum = 0.0;
    std::vector<double> radii;
    const int nCyl = countXAxisCylinders(wp, radSum, radii);
    if (nCyl < 4) return out;  // at least 4 bolt holes total (2 per plate × 2 plates)

    // All bolt-hole radii should be similar (regular pattern).
    std::sort(radii.begin(), radii.end());
    const double rMed = radii[radii.size() / 2];
    int sameRad = 0;
    for (double r : radii)
        if (std::abs(r - rMed) < 0.3) ++sameRad;
    if (sameRad < 4) return out;

    // Identify endplate planar faces.
    int endplateFront = 0, endplateRear = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(std::abs(n.X()) - 1.0) > 1e-3) continue;
        const gp_Pnt c = wp.faceCenter(i);
        if (c.X() < xMin + 0.1 * extX) ++endplateFront;
        if (c.X() > xMax - 0.1 * extX) ++endplateRear;
    }
    if (endplateFront == 0 || endplateRear == 0) return out;

    const double width  = extY;
    const double height = extZ;
    const double length_estimate = extX;
    const int boltsPerPlate = std::max(1, nCyl / 2);

    json recovered = {
        { "length_mm",     length_estimate },
        { "width_mm",      width  },
        { "height_mm",     height },
        { "wall_t_mm",     std::min(width, height) * 0.05 },
        { "endplate_t_mm", std::min(width, height) * 0.1  },
        { "bolt_grid",     { std::max(1, boltsPerPlate / 2),
                             std::max(1, boltsPerPlate / 2) } },
        { "bolt_dia_mm",   2.0 * rMed },
        { "edge_dist_mm",  std::min(width, height) * 0.25 },
    };
    json matched = {
        { "source",            "geometric_fallback" },
        { "x_cylinder_count",  nCyl },
        { "bolt_radius_med",   rMed },
        { "endplate_front",    endplateFront },
        { "endplate_rear",     endplateRear },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.65, matched });
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
                                { "is_compound", true } };
        out.push_back(rf);
    }
    if (out.empty()) {
        out = geometric_fallback(wp);
    }
    return out;
}

}  // namespace koocadcam::skill::box_section_with_endplate
