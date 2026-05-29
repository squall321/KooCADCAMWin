// @lat: [[engine/skills#coil_spring_seat_compound]]

#include "coil_spring_seat_compound.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::coil_spring_seat_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// DIN 2098 / SAE J1122 compression-spring mounting gates.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.spring_od <= 0.0 || in.free_height <= 0.0 || in.pilot_dia <= 0.0) {
        r.add("DFM-INPUT", "error",
              "coil_spring_seat_compound: spring_od, free_height, pilot_dia "
              "must all be > 0");
        return r;
    }
    if (in.slip_fit_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "coil_spring_seat_compound: slip_fit_mm must be >= 0");
        return r;
    }

    if (in.spring_od < 2.0 || in.spring_od > 200.0) {
        r.add("DFM-SPRING-RANGE", "error",
              "coil_spring_seat_compound: spring_od " +
              std::to_string(in.spring_od) +
              " mm outside DIN 2098 production range [2, 200]");
    }

    if (in.pilot_dia < 0.8) {
        r.add("DFM-002", "error",
              "coil_spring_seat_compound: pilot_dia " +
              std::to_string(in.pilot_dia) + " mm < min drill 0.8 mm");
    }

    // Pilot must clear the spring's inner coil — conservative bound
    // pilot_dia <= 0.5 × spring_od (spring inner dia ≈ 0.7 × OD for typ
    // wire/OD ratios; we use 0.5 to leave a margin for coil deformation).
    if (in.pilot_dia > 0.5 * in.spring_od) {
        r.add("DFM-SPRING-PILOT", "error",
              "coil_spring_seat_compound: pilot_dia " +
              std::to_string(in.pilot_dia) +
              " > 0.5 × spring_od (" +
              std::to_string(0.5 * in.spring_od) +
              ") — pilot would foul inner coils");
    }

    // Spring slenderness — DIN 2098: free_height should be at least the
    // OD × 0.6 to avoid block-coil-only pocket nonsense.
    if (in.free_height < in.spring_od * 0.6) {
        r.add("DFM-SPRING-SLENDER", "warning",
              "coil_spring_seat_compound: free_height " +
              std::to_string(in.free_height) +
              " < 0.6 × spring_od — coil_count likely < 3, abnormal");
    }

    // Workpiece thickness margin: pilot goes ~ free_height + 4 mm into
    // the material; we need at least 1 mm wall remaining below pilot.
    if (!wp.shape().IsNull()) {
        double xMin, yMin, zMin, xMax, yMax, zMax;
        wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
        const double thickness = zMax - zMin;
        const double pilotDepth = in.free_height + 4.0;
        if (thickness < pilotDepth + 1.0) {
            r.add("DFM-SPRING-WALL", "error",
                  "coil_spring_seat_compound: stock thickness " +
                  std::to_string(thickness) +
                  " < pilot_depth + 1 mm wall (" +
                  std::to_string(pilotDepth + 1.0) + ")");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Sub-feature chain:
//   1. Spring pocket cylinder  (slip-fit OD, depth = free_height + 1 mm)
//   2. Pilot pin hole          (deeper than pocket — leaves annular shoulder)
//   Both cutters are concentric and overlapping → fuse first, single cut.

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "coil_spring_seat_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto faceId = wp.resolve(in.face_id);
    if (!faceId) throw SkillError(
        "coil_spring_seat_compound: face_id datum unresolved");

    // Derived geometry
    const double pocketRadius = (in.spring_od + 2.0 * in.slip_fit_mm) / 2.0;
    const double pocketDepth  = in.free_height + 1.0;       // +1 mm head room
    const double pilotDepth   = in.free_height + 4.0;       // +4 mm engagement
    const double pilotRadius  = in.pilot_dia / 2.0;

    // Compute entry point.  Use bbox top for the common case (axis = -Z).
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    constexpr double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt entryPt(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        const double entryZ = (adir.Z() < 0) ? zMax : zMin;
        entryPt = gp_Pnt(in.position_x_mm, in.position_y_mm,
                         entryZ - adir.Z() * kOverhang);
    }

    // gp_Ax2(P, V): V is LOCAL Z (axial drilling direction).
    const gp_Ax2 ax(entryPt, adir);

    // Sub-feature 1: spring pocket cylinder
    const TopoDS_Shape pocketTool =
        pr::cylinder(ax, pocketRadius, pocketDepth + kOverhang);

    // Sub-feature 2: pilot pin hole (concentric, deeper)
    const TopoDS_Shape pilotTool =
        pr::cylinder(ax, pilotRadius, pilotDepth + kOverhang);

    // Fuse concentric overlapping cylinders → one cutter → one cut.
    const TopoDS_Shape fusedCutter = pr::fuse(pocketTool, pilotTool);
    const TopoDS_Shape newShape    = pr::cut(wp.shape(), fusedCutter);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "face_id",        *faceId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { adir.X(), adir.Y(), adir.Z() } },
        { "spring_od",      in.spring_od },
        { "free_height",    in.free_height },
        { "pilot_dia",      in.pilot_dia },
        { "slip_fit_mm",    in.slip_fit_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           3 },
        { "subfeatures", json::array({
            { { "op", "spring_pocket_cyl" },
              { "radius_mm", pocketRadius }, { "depth_mm", pocketDepth } },
            { { "op", "pilot_pin_hole" },
              { "radius_mm", pilotRadius },  { "depth_mm", pilotDepth } },
            { { "op", "shoulder_annulus" },
              { "outer_r_mm", pocketRadius }, { "inner_r_mm", pilotRadius },
              { "z_from_entry_mm", pocketDepth } },
        }) },
        { "spring_od",                  in.spring_od },
        { "free_height",                in.free_height },
        { "pilot_dia",                  in.pilot_dia },
        { "pocket_radius_mm",           pocketRadius },
        { "pocket_depth_mm",            pocketDepth },
        { "pilot_depth_mm",             pilotDepth },
        { "shoulder_height_mm",         pilotDepth - pocketDepth },
        { "cylindrical_face_count",     2 },
        { "annular_shoulder_present",   true },
        { "bottom_planar_face_present", true },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "endmill;drill";
    tooling.tool_dia_mm       = 2.0 * pocketRadius;
    tooling.tool_length_mm    = pilotDepth * 1.5 + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double pocketVol = M_PI * pocketRadius * pocketRadius * pocketDepth;
    const double pilotVol  = M_PI * pilotRadius  * pilotRadius
                           * (pilotDepth - pocketDepth);
    tooling.stock_removed_mm3 = pocketVol + pilotVol;
    tooling.est_cycle_time_s  = std::max(2.0, pilotDepth / 30.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "endmill" }, { "tool_dia_mm", 2.0 * pocketRadius },
              { "depth_mm", pocketDepth } },
            { { "tool_type", "drill" },   { "tool_dia_mm", in.pilot_dia },
              { "depth_mm", pilotDepth } },
        } },
        { "feature_standard", "DIN 2098 coil-spring mounting pocket" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::coil_spring_seat_compound applied: od={} free={} pilot={}",
                  in.spring_od, in.free_height, in.pilot_dia);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + geometric fallback ────────────────────
//
// Geometric signature of a coil-spring pocket + pilot:
//   - TWO coaxial cylindrical faces, both blind (closed bottom).
//   - "pocket" cylinder: larger radius (= spring_od/2 + slip), shorter depth.
//   - "pilot"  cylinder: smaller radius (= pilot_dia/2), deeper.
//   - pilot.radius < 0.5 * pocket.radius (DFM gate matches synthesis).
//   - Both share an axis line within tolerance.

namespace {

struct CylDesc {
    int    faceIdx;
    gp_Ax1 axis;
    double radius;
    double axialMin;
    double axialMax;
    gp_Pnt entryCenter;
    gp_Pnt deepCenter;
};

bool sameAxisInfinite(const gp_Ax1& a, const gp_Ax1& b,
                      double angTolDeg = 0.5, double posTolMm = 0.05)
{
    const gp_Dir da = a.Direction(), db = b.Direction();
    const double dot = std::abs(da.X()*db.X() + da.Y()*db.Y() + da.Z()*db.Z());
    if (dot < std::cos(angTolDeg * M_PI / 180.0)) return false;
    gp_Vec v(a.Location(), b.Location());
    gp_Vec axV(da.X(), da.Y(), da.Z());
    gp_Vec perp = v - axV * v.Dot(axV);
    return perp.Magnitude() < posTolMm;
}

std::vector<CylDesc> collectCyls(const Workpiece& wp)
{
    std::vector<CylDesc> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface surf(wp.face(i));
        const gp_Cylinder cy = surf.Cylinder();
        CylDesc d;
        d.faceIdx = i;
        d.axis    = cy.Axis();
        d.radius  = cy.Radius();
        std::vector<gp_Pnt> centers;
        for (TopExp_Explorer exp(wp.face(i), TopAbs_EDGE); exp.More(); exp.Next()) {
            BRepAdaptor_Curve crv(TopoDS::Edge(exp.Current()));
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(d.axis.Direction())) - 1.0) > 1e-3)
                continue;
            if (std::abs(c.Radius() - d.radius) > 1e-3) continue;
            centers.push_back(c.Location());
        }
        if (centers.size() < 2) continue;
        const gp_Dir adir = d.axis.Direction();
        const gp_Pnt aOrg = d.axis.Location();
        auto proj = [&](const gp_Pnt& p){
            return (p.X()-aOrg.X())*adir.X()+(p.Y()-aOrg.Y())*adir.Y()+(p.Z()-aOrg.Z())*adir.Z();
        };
        auto cmp = [&](const gp_Pnt& a, const gp_Pnt& b){ return proj(a) < proj(b); };
        const auto mn = std::min_element(centers.begin(), centers.end(), cmp);
        const auto mx = std::max_element(centers.begin(), centers.end(), cmp);
        d.axialMin = proj(*mn);
        d.axialMax = proj(*mx);
        d.entryCenter = *mn;
        d.deepCenter  = *mx;
        out.push_back(d);
    }
    return out;
}

std::vector<RecognizedFeature> geometric_fallback(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto cyls = collectCyls(wp);
    if (cyls.size() < 2) return out;
    std::vector<bool> consumed(cyls.size(), false);

    for (size_t i = 0; i < cyls.size(); ++i) {
        if (consumed[i]) continue;
        for (size_t j = i + 1; j < cyls.size(); ++j) {
            if (consumed[j]) continue;
            const CylDesc& a = cyls[i];
            const CylDesc& b = cyls[j];
            if (!sameAxisInfinite(a.axis, b.axis)) continue;
            const CylDesc& pocket = (a.radius > b.radius) ? a : b;
            const CylDesc& pilot  = (a.radius > b.radius) ? b : a;
            if (pocket.radius - pilot.radius < 1.0) continue;
            if (pilot.radius > 0.5 * pocket.radius)  continue;   // DFM gate
            if (pilot.radius < 0.4)                  continue;
            const double pocketLen = pocket.axialMax - pocket.axialMin;
            const double pilotLen  = pilot.axialMax  - pilot.axialMin;
            // Pilot extends deeper than the pocket.
            if (pilot.axialMax <= pocket.axialMax + 0.05) continue;
            if (pocketLen < 1.0 || pilotLen < pocketLen) continue;
            // Range gate from DFM (synthesis).
            const double sprOD = 2.0 * pocket.radius;
            if (sprOD < 2.0 || sprOD > 200.0) continue;

            gp_Vec drillVec(pocket.entryCenter, pilot.deepCenter);
            if (drillVec.Magnitude() < 1e-6) continue;
            drillVec.Normalize();

            const double freeHeight = std::max(pocketLen - 1.0, 0.5);
            json recovered = {
                { "position_x_mm", pocket.entryCenter.X() },
                { "position_y_mm", pocket.entryCenter.Y() },
                { "axis_dir",      { drillVec.X(), drillVec.Y(), drillVec.Z() } },
                { "spring_od",     sprOD },
                { "free_height",   freeHeight },
                { "pilot_dia",     2.0 * pilot.radius },
                { "slip_fit_mm",   0.0 },
            };
            json matched = {
                { "source",          "geometric_fallback" },
                { "pocket_face_id",  pocket.faceIdx },
                { "pilot_face_id",   pilot.faceIdx },
                { "pocket_depth_mm", pocketLen },
                { "pilot_depth_mm",  pilotLen },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.60, matched });
            consumed[i] = true;
            consumed[j] = true;
            break;
        }
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
            { "source", "metadata_replay" },
            { "is_compound", true },
            { "subfeature_count", 3 },
        };
        out.push_back(r);
    }
    if (out.empty()) {
        out = geometric_fallback(wp);
    }
    return out;
}

}  // namespace koocadcam::skill::coil_spring_seat_compound
