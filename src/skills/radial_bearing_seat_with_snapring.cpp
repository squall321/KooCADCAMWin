// @lat: [[engine/skills#radial_bearing_seat_with_snapring]]

#include "radial_bearing_seat_with_snapring.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::radial_bearing_seat_with_snapring {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── DIN 471 snap-ring thickness table ────────────────────────────────────

double snapRingThicknessFor(double outer_dia_mm)
{
    if (outer_dia_mm >= 10.0 && outer_dia_mm < 18.0) return 1.0;
    if (outer_dia_mm >= 18.0 && outer_dia_mm < 30.0) return 1.2;
    if (outer_dia_mm >= 30.0 && outer_dia_mm < 50.0) return 1.5;
    if (outer_dia_mm >= 50.0 && outer_dia_mm <= 80.0) return 2.0;
    return 0.0;
}

// Derived geometry helper.
namespace {

constexpr double kChamferAxial_mm = 0.5;   // 0.5 × 45° lead-in
constexpr double kChamferAngleDeg = 45.0;
constexpr double kGroovePos_mm    = 1.5;   // groove axial position from entry

double grooveOuterDia(double outer_dia_mm, double t)   // DIN 471 d2 ≈ od + 0.7t
{
    return outer_dia_mm + 0.7 * t;
}

double grooveWidth(double t)                           // DIN 471 w ≈ 1.5t
{
    return 1.5 * t;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.outer_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": outer_dia_mm must be > 0");
    }
    if (in.inner_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": inner_dia_mm must be > 0");
    }
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": depth_mm must be > 0");
    }
    if (in.outer_dia_mm > 0.0 && in.inner_dia_mm > 0.0 &&
        in.inner_dia_mm >= in.outer_dia_mm) {
        r.add("DFM-SEAT-GEOM", "error", std::string(kSkillId) +
              ": inner_dia (" + std::to_string(in.inner_dia_mm) +
              ") must be < outer_dia (" + std::to_string(in.outer_dia_mm) +
              ") — shoulder cannot retain bearing");
    }
    if (in.inner_dia_mm > 0.0 && in.inner_dia_mm < 0.8) {
        r.add("DFM-002", "error", std::string(kSkillId) +
              ": inner_dia " + std::to_string(in.inner_dia_mm) +
              " mm < min 0.8 mm");
    }

    const double t = snapRingThicknessFor(in.outer_dia_mm);
    if (t <= 0.0 && in.outer_dia_mm > 0.0) {
        r.add("DFM-DIN471-RANGE", "error", std::string(kSkillId) +
              ": outer_dia " + std::to_string(in.outer_dia_mm) +
              " mm outside DIN 471 supported range (10-80 mm)");
    } else if (t > 0.0) {
        const double minDepth = kGroovePos_mm + 2.0 * grooveWidth(t);
        if (in.depth_mm < minDepth) {
            r.add("DFM-DIN471-DEPTH", "error", std::string(kSkillId) +
                  ": depth " + std::to_string(in.depth_mm) +
                  " mm < required " + std::to_string(minDepth) +
                  " mm for snap-ring groove + clearance");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = std::string(kSkillId) + " DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError(std::string(kSkillId) +
                                   ": entry_face datum unresolved");

    // Derived geometry
    const double t            = snapRingThicknessFor(in.outer_dia_mm);
    const double grooveOD     = grooveOuterDia(in.outer_dia_mm, t);
    const double grooveW      = grooveWidth(t);
    const double outerR       = in.outer_dia_mm / 2.0;
    const double innerR       = in.inner_dia_mm / 2.0;
    const double grooveR      = grooveOD / 2.0;

    // Build cut tool starting on entry plane.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    constexpr double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;
    gp_Pnt entry(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
    if (!(std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6 && adir.Z() < 0)) {
        // Generalised back-off along reversed axis.
        const double bboxDiag = std::sqrt(
            (xMax - xMin) * (xMax - xMin) +
            (yMax - yMin) * (yMax - yMin) +
            (zMax - zMin) * (zMax - zMin));
        entry = gp_Pnt(
            in.position_x_mm - adir.X() * (bboxDiag + kEntryOverhang),
            in.position_y_mm - adir.Y() * (bboxDiag + kEntryOverhang),
            (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kEntryOverhang));
    }

    // Sub-feature 1: outer bore — full depth.
    const gp_Ax2 outerAx(entry, adir);
    const TopoDS_Shape outerBore =
        pr::cylinder(outerAx, outerR, in.depth_mm + kEntryOverhang);

    // Sub-feature 2: inner shoulder step (deeper than outer bore — punches
    // a smaller hole through the shoulder).  Modelled as an additional
    // cylinder fused into the tool stack so the boolean cut leaves a clean
    // shoulder step where the two diameters meet.
    const double shoulderExtra = std::max(0.5, in.depth_mm * 0.25);
    const TopoDS_Shape shoulderBore =
        pr::cylinder(outerAx, innerR,
                     in.depth_mm + shoulderExtra + kEntryOverhang);

    // Sub-feature 3: snap-ring groove — annular ring sitting at
    // kGroovePos_mm below the entry plane.
    // gp_Ax2(P, V, Vx): V is LOCAL Z (axial).  We offset along adir to put
    // the ring at the correct axial position.
    const gp_Pnt grooveOrigin(
        entry.X() + adir.X() * kGroovePos_mm,
        entry.Y() + adir.Y() * kGroovePos_mm,
        entry.Z() + adir.Z() * kGroovePos_mm);
    const gp_Ax2 grooveAx(grooveOrigin, adir);
    const TopoDS_Shape groove =
        pr::annularRing(grooveAx, grooveR, outerR + 1e-4, grooveW);

    // Sub-feature 4: 0.5 × 45° lead-in chamfer at entry — cone frustum
    // expanding outward from outerR to outerR + 0.5 over kChamferAxial.
    const double chamferTopR = outerR + kChamferAxial_mm;
    const gp_Ax2 chamferAx(entry, adir);
    const TopoDS_Shape chamfer =
        pr::coneFrustum(chamferAx, chamferTopR, outerR, kChamferAxial_mm);

    // Fuse all concentric overlapping tools, then a single cut.
    const TopoDS_Shape fused1 = pr::fuse(outerBore, shoulderBore);
    const TopoDS_Shape fused2 = pr::fuse(fused1,    groove);
    const TopoDS_Shape fused  = pr::fuse(fused2,    chamfer);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // Signature.
    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
        { "outer_dia_mm",    in.outer_dia_mm },
        { "inner_dia_mm",    in.inner_dia_mm },
        { "depth_mm",        in.depth_mm },
        { "snap_ring_std",   in.snap_ring_std },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     4 },
        { "outer_dia_mm",         in.outer_dia_mm },
        { "inner_dia_mm",         in.inner_dia_mm },
        { "depth_mm",             in.depth_mm },
        { "snap_ring_std",        in.snap_ring_std },
        { "snap_ring_thickness_mm", t },
        { "groove_outer_dia_mm",  grooveOD },
        { "groove_width_mm",      grooveW },
        { "groove_axial_pos_mm",  kGroovePos_mm },
        { "chamfer_axial_mm",     kChamferAxial_mm },
        { "chamfer_angle_deg",    kChamferAngleDeg },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "boring_bar;groove_tool;chamfer_tool";
    tooling.tool_dia_mm = in.outer_dia_mm;
    tooling.tool_material = "carbide";
    tooling.flute_count = 1;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 =
        M_PI * outerR * outerR * in.depth_mm +
        M_PI * innerR * innerR * shoulderExtra +
        M_PI * (grooveR * grooveR - outerR * outerR) * grooveW;
    tooling.est_cycle_time_s = std::max(3.0, in.depth_mm / 25.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "boring_bar" },
              { "tool_dia_mm", in.outer_dia_mm },
              { "depth_mm", in.depth_mm } },
            { { "tool_type", "boring_bar" },
              { "tool_dia_mm", in.inner_dia_mm },
              { "depth_mm", in.depth_mm + shoulderExtra } },
            { { "tool_type", "groove_tool" },
              { "groove_outer_dia_mm", grooveOD },
              { "groove_width_mm", grooveW },
              { "axial_position_mm", kGroovePos_mm },
              { "note", "DIN 471 external snap-ring groove" } },
            { { "tool_type", "chamfer_tool" },
              { "axial_mm", kChamferAxial_mm },
              { "angle_deg", kChamferAngleDeg } },
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::radial_bearing_seat_with_snapring applied: od={} id={} d={} t={}",
                  in.outer_dia_mm, in.inner_dia_mm, in.depth_mm, t);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + geometric fallback ────────────────────
//
// Geometric signature of a radial bearing seat with internal snap-ring:
//   - THREE coaxial cylindrical faces sharing the same axis line:
//       * outer bore cylinder  (radius = outer_dia/2)
//       * shoulder bore cylinder (radius = inner_dia/2, deeper than outer)
//       * snap-ring groove cylinder (radius > outer_dia/2, very short axial)
//   - axial ordering along drilling direction:
//       entry → outer bore → shoulder bore extends past outer bore;
//       the groove cylinder is within the outer bore's axial extent.

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
    if (cyls.size() < 3) return out;

    for (size_t i = 0; i < cyls.size(); ++i) {
        for (size_t j = 0; j < cyls.size(); ++j) {
            if (i == j) continue;
            for (size_t k = 0; k < cyls.size(); ++k) {
                if (k == i || k == j) continue;
                const CylDesc& outer    = cyls[i];
                const CylDesc& shoulder = cyls[j];
                const CylDesc& groove   = cyls[k];
                if (!sameAxisInfinite(outer.axis,    shoulder.axis)) continue;
                if (!sameAxisInfinite(outer.axis,    groove.axis))   continue;
                // Radii order: shoulder < outer < groove
                if (!(shoulder.radius + 0.1 < outer.radius)) continue;
                if (!(groove.radius   > outer.radius + 0.1)) continue;
                // Outer bore radius in DIN 471 supported range.
                if (snapRingThicknessFor(2.0 * outer.radius) <= 0.0) continue;
                // Groove axial extent must be SHORT (groove width ≈ 1.5·t_DIN471).
                const double grooveLen = groove.axialMax - groove.axialMin;
                if (grooveLen > 5.0) continue;
                // Outer bore depth: substantial.
                const double outerLen = outer.axialMax - outer.axialMin;
                if (outerLen < 1.0) continue;
                // Shoulder must extend deeper than outer along drilling axis.
                if (shoulder.axialMax <= outer.axialMax + 0.05) continue;
                // Groove axial position should fall within outer bore extent.
                const double grooveMid = (groove.axialMin + groove.axialMax) * 0.5;
                if (grooveMid < outer.axialMin - 0.1) continue;
                if (grooveMid > outer.axialMax + 0.1) continue;

                gp_Vec drillVec(outer.entryCenter, shoulder.deepCenter);
                if (drillVec.Magnitude() < 1e-6) continue;
                drillVec.Normalize();

                json recovered = {
                    { "position_x_mm",  outer.entryCenter.X() },
                    { "position_y_mm",  outer.entryCenter.Y() },
                    { "axis_dir",       { drillVec.X(), drillVec.Y(), drillVec.Z() } },
                    { "outer_dia_mm",   2.0 * outer.radius },
                    { "inner_dia_mm",   2.0 * shoulder.radius },
                    { "depth_mm",       outerLen },
                    { "snap_ring_std",  "DIN471" },
                };
                json matched = {
                    { "source",            "geometric_fallback" },
                    { "outer_face_id",     outer.faceIdx },
                    { "shoulder_face_id",  shoulder.faceIdx },
                    { "groove_face_id",    groove.faceIdx },
                    { "groove_width_mm",   grooveLen },
                    { "groove_radius_mm",  groove.radius },
                };
                out.push_back(RecognizedFeature{ kSkillId, recovered, 0.60, matched });
                return out;  // first valid triple is enough
            }
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
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    if (out.empty()) {
        out = geometric_fallback(wp);
    }
    return out;
}

}  // namespace koocadcam::skill::radial_bearing_seat_with_snapring
