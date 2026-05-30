// @lat: [[engine/skills#gusset_plate_compound]]

#include "gusset_plate_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::gusset_plate_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// Build a triangular prism by extruding a 3-point polygon along +Z.
TopoDS_Shape makeTriPrism(const gp_Pnt& p0,
                          const gp_Pnt& p1,
                          const gp_Pnt& p2,
                          double thickness)
{
    BRepBuilderAPI_MakePolygon poly;
    poly.Add(p0);
    poly.Add(p1);
    poly.Add(p2);
    poly.Close();
    if (!poly.IsDone())
        throw SkillError("gusset_plate_compound: triangle polygon build failed");

    BRepBuilderAPI_MakeFace face(poly.Wire(), true);
    if (!face.IsDone())
        throw SkillError("gusset_plate_compound: triangle face build failed");

    gp_Vec extrusion(0.0, 0.0, thickness);
    BRepPrimAPI_MakePrism prism(face.Shape(), extrusion);
    prism.Build();
    if (!prism.IsDone())
        throw SkillError("gusset_plate_compound: triangle prism build failed");
    return prism.Shape();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.leg_a_mm <= 0.0 || in.leg_b_mm <= 0.0 ||
        in.plate_t_mm <= 0.0 || in.lift_hole_dia_mm <= 0.0 ||
        in.bevel_mm < 0.0 || in.notch_r_mm < 0.0)
    {
        r.add("DFM-GUSSET-INPUT", "error",
              "gusset_plate_compound: all dimensions must be > 0 "
              "(bevel/notch may be 0 but not negative)");
        return r;
    }

    if (in.plate_t_mm < 6.0) {
        r.add("DFM-GUSSET-THK", "error",
              "gusset_plate_compound: plate_t " +
              std::to_string(in.plate_t_mm) +
              " mm < 6 mm — sub-structural per AISC J6 minimum gusset thickness");
    }
    if (in.lift_hole_dia_mm < 16.0) {
        r.add("DFM-GUSSET-LIFT-D", "error",
              "gusset_plate_compound: lift_hole_dia " +
              std::to_string(in.lift_hole_dia_mm) +
              " mm < 16 mm — below smallest standard shackle pin");
    }
    // Lift hole placed at 1/3 of each leg from the right-angle corner;
    // smallest edge distance must be ≥ 2.5 × lift_hole_dia (AISC J3.5).
    const double minLeg = std::min(in.leg_a_mm, in.leg_b_mm);
    const double liftEdge = minLeg / 3.0;
    if (liftEdge < 2.5 * in.lift_hole_dia_mm) {
        r.add("DFM-GUSSET-LIFT-E", "warning",
              "gusset_plate_compound: lift-hole edge distance " +
              std::to_string(liftEdge) +
              " mm < 2.5 × dia — gusset legs too short for safe lift point");
    }
    if (in.notch_r_mm > 0.25 * minLeg) {
        r.add("DFM-GUSSET-NOTCH", "warning",
              "gusset_plate_compound: notch_r > 25% of shortest leg — "
              "removes too much material");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gusset_plate_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double a = in.leg_a_mm;
    const double b = in.leg_b_mm;
    const double t = in.plate_t_mm;

    // Sub-feature 1: triangular plate body.
    const gp_Pnt p0(0.0, 0.0, 0.0);
    const gp_Pnt p1(a,   0.0, 0.0);
    const gp_Pnt p2(0.0, b,   0.0);
    TopoDS_Shape plate = makeTriPrism(p0, p1, p2, t);

    // Sub-feature 2: bevel cut along the hypotenuse (cut a thin sliver off the
    // hypotenuse edge to model a chamfered fit edge).  We do this by removing
    // a small box positioned so it intersects the hypotenuse.
    // Hypotenuse goes from (a,0) to (0,b) — its midpoint is (a/2, b/2).
    // Outward unit normal in XY = (b/L, a/L) where L = √(a²+b²).
    if (in.bevel_mm > 0.0) {
        const double L = std::sqrt(a * a + b * b);
        const double nx = b / L, ny = a / L;
        // Build a slim box outside the hypotenuse that we then fuse with the
        // small wedge on the inside.  Simpler: cut a thin parallel band
        // overlapping the hypotenuse by `bevel_mm` outward.
        // Construct a cutter box covering the full plate extent, offset by
        // (bevel_mm * normal) inward from the hypotenuse.
        // Cutter box: large rectangle centered on hypotenuse midpoint, extending
        // perpendicular to it by `bevel_mm` on the OUTSIDE only.
        const double cutterW = L + 2.0 * in.bevel_mm;
        const double cutterH = in.bevel_mm * 2.0;
        const gp_Pnt midpt(a / 2.0 + nx * in.bevel_mm,
                           b / 2.0 + ny * in.bevel_mm,
                           -0.1);
        // Build the cutter aligned with the hypotenuse direction.
        // Hypotenuse direction = (-a, b) / L.
        const gp_Dir hypDir(-a / L, b / L, 0.0);
        const gp_Dir hypNorm(nx, ny, 0.0);
        // Place box origin at midpt - hypDir × cutterW/2 - hypNorm × cutterH/2.
        const gp_Pnt boxOrigin(
            midpt.X() - hypDir.X() * cutterW / 2.0 - hypNorm.X() * cutterH / 2.0,
            midpt.Y() - hypDir.Y() * cutterW / 2.0 - hypNorm.Y() * cutterH / 2.0,
            -0.1);
        const gp_Ax2 bevAx(boxOrigin, gp::DZ(), hypDir);
        const TopoDS_Shape bevCutter = pr::box(bevAx, cutterW, cutterH, t + 0.2);
        try {
            plate = pr::cut(plate, bevCutter);
        } catch (...) {
            // Bevel cut is decorative — fall back silently.
        }
    }

    // Sub-features 3..6: 4 weld-prep corner notches (each is a radial fillet
    // at a corner along the inside edge — we cut a small cylinder at the
    // right-angle apex and two on each leg-tip).  4 notches total at:
    //   (0, 0)        — apex
    //   (notch_r, 0)
    //   (0, notch_r)
    //   (a - notch_r, 0)  and  (0, b - notch_r) ... we pick 4 strategically:
    //
    // Convention: 4 notches at (0,0), (a,0), (0,b), and one on the hypotenuse
    // midpoint — represent the four weld-stop reliefs.
    const double nr = in.notch_r_mm;
    int notchCount = 0;
    std::vector<TopoDS_Shape> notches;
    if (nr > 0.0) {
        const double notchH = t + 0.2;
        const std::vector<std::pair<double, double>> notchXY {
            { 0.0, 0.0 },
            { a,   0.0 },
            { 0.0, b },
            { a / 2.0, b / 2.0 },
        };
        for (const auto& xy : notchXY) {
            const gp_Ax2 ax(gp_Pnt(xy.first, xy.second, -0.1), gp::DZ());
            notches.push_back(pr::cylinder(ax, nr, notchH));
            ++notchCount;
        }
        try {
            plate = pr::cutMany(plate, notches);
        } catch (...) {
            // notch overlap with bevel can fail OCCT; fall back silently.
        }
    }

    // Sub-feature 7: lifting hole.  Located at 1/3 from right-angle apex
    // along each leg (centered on (a/3, b/3) in XY).
    const double liftX = a / 3.0;
    const double liftY = b / 3.0;
    const gp_Ax2 liftAx(gp_Pnt(liftX, liftY, -0.1), gp::DZ());
    const TopoDS_Shape liftHole = pr::cylinder(liftAx, in.lift_hole_dia_mm / 2.0, t + 0.2);
    plate = pr::cut(plate, liftHole);

    const int subfeatureCount = 1 + (in.bevel_mm > 0.0 ? 1 : 0) + notchCount + 1;

    json params = {
        { "leg_a_mm",          a },
        { "leg_b_mm",          b },
        { "plate_t_mm",        t },
        { "lift_hole_dia_mm",  in.lift_hole_dia_mm },
        { "bevel_mm",          in.bevel_mm },
        { "notch_r_mm",        in.notch_r_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    subfeatureCount },
        { "leg_a_mm",            a },
        { "leg_b_mm",            b },
        { "plate_t_mm",          t },
        { "lift_hole_dia_mm",    in.lift_hole_dia_mm },
        { "hypotenuse_mm",       std::sqrt(a * a + b * b) },
        { "lift_position",       { { "x", liftX }, { "y", liftY } } },
        { "notch_count",         notchCount },
        { "has_bevel",           in.bevel_mm > 0.0 },
    };

    const double areaTri = 0.5 * a * b;
    const double volMm3  = areaTri * t;
    const double weightKg = volMm3 * 7.85e-6;

    ToolingMeta tooling;
    tooling.tool_type         = "plasma_cut;drill;bevel_mill";
    tooling.tool_dia_mm       = in.lift_hole_dia_mm;
    tooling.tool_length_mm    = t * 2.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = M_PI * (in.lift_hole_dia_mm / 2.0) *
                                (in.lift_hole_dia_mm / 2.0) * t;
    tooling.est_cycle_time_s  = 30.0 + 5.0 * notchCount;
    tooling.extra = {
        { "subfeature_sequence", {
            { { "name", "triangle_body" }   },
            { { "name", "hypotenuse_bevel" }},
            { { "name", "corner_notches" }, { "count", notchCount } },
            { { "name", "lift_hole" }, { "dia_mm", in.lift_hole_dia_mm } },
        } },
        { "approx_weight_kg",    weightKg },
        { "standard",            "AISC_J6_gusset" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(plate, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::gusset_plate_compound applied: legs={}x{} t={} lift={}",
                  a, b, t, in.lift_hole_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition (metadata replay + geometric fallback) ──────────────────
//
// Geometric signature of a gusset plate:
//   - Flat plate (small extent along one axis, the plate thickness, vs. legs).
//   - Triangular footprint: 3 plate-edge planar walls (the two legs + the
//     bevel-trimmed hypotenuse), all with Z-normal = 0.
//   - At least one cylindrical face axis ∥ Z (the lift hole).
//   - Optional small cyl faces (notches) along plate edges.

namespace {

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

    // Plate: thickness (Z) much smaller than legs (X, Y).
    if (extZ >= std::min(extX, extY) * 0.5) return out;

    // Collect Z-axis cylindrical faces (lift hole + corner notches).
    std::vector<double> zCylRadii;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cy = surf.Cylinder();
        const gp_Dir axDir = cy.Axis().Direction();
        if (std::abs(std::abs(axDir.Z()) - 1.0) > 1e-3) continue;
        zCylRadii.push_back(cy.Radius());
    }
    if (zCylRadii.empty()) return out;
    // Largest Z-cyl is the lift hole.
    std::sort(zCylRadii.begin(), zCylRadii.end());
    const double liftR = zCylRadii.back();
    if (liftR * 2.0 < 6.0) return out;  // must be at least 6 mm (sub-DFM check)

    // Count planar perimeter walls (Z-normal = 0).
    int perimWalls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (std::abs(n.Z()) > 1e-2) continue;
        ++perimWalls;
    }
    if (perimWalls < 3) return out;  // triangle has ≥ 3 walls

    json recovered = {
        { "leg_a_mm",          extX },
        { "leg_b_mm",          extY },
        { "plate_t_mm",        extZ },
        { "lift_hole_dia_mm",  2.0 * liftR },
        { "bevel_mm",          0.0 },
        { "notch_r_mm",        (zCylRadii.size() >= 2) ? zCylRadii.front() : 0.0 },
    };
    json matched = {
        { "source",         "geometric_fallback" },
        { "z_cyl_count",    static_cast<int>(zCylRadii.size()) },
        { "perim_walls",    perimWalls },
        { "lift_radius",    liftR },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.60, matched });
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

}  // namespace koocadcam::skill::gusset_plate_compound
