// @lat: [[engine/skills#thrust_bearing_seat_compound]]

#include "thrust_bearing_seat_compound.hpp"

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

namespace koocadcam::skill::thrust_bearing_seat_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.outer_dia_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": outer_dia_mm must be > 0");
    if (in.inner_dia_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": inner_dia_mm must be > 0");
    if (in.face_depth_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": face_depth_mm must be > 0");
    if (in.seat_depth_mm <= 0.0)
        r.add("DFM-INPUT", "error", std::string(kSkillId) +
              ": seat_depth_mm must be > 0");

    if (in.outer_dia_mm > 0.0 && in.inner_dia_mm > 0.0 &&
        in.inner_dia_mm >= in.outer_dia_mm) {
        r.add("DFM-SEAT-GEOM", "error", std::string(kSkillId) +
              ": inner_dia (" + std::to_string(in.inner_dia_mm) +
              ") must be < outer_dia (" + std::to_string(in.outer_dia_mm) + ")");
    }

    const double race_width = in.outer_dia_mm - in.inner_dia_mm;
    if (race_width > 0.0 && race_width < 1.6) {
        r.add("DFM-002", "error", std::string(kSkillId) +
              ": race width (od-id) " + std::to_string(race_width) +
              " mm < min 1.6 mm — insufficient bearing-race width");
    }

    if (in.inner_dia_mm > 0.0 && in.inner_dia_mm < 3.0) {
        r.add("DFM-LOCKNUT", "error", std::string(kSkillId) +
              ": inner_dia " + std::to_string(in.inner_dia_mm) +
              " mm < 3.0 mm (M3 minimum lock-nut shaft thread)");
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

    // Derived raceway geometry.
    const double race_width = in.outer_dia_mm - in.inner_dia_mm;
    const double raceway_groove_width  = race_width / 6.0;
    const double raceway_groove_depth  = race_width / 8.0;
    const double outerR = in.outer_dia_mm / 2.0;
    const double innerR = in.inner_dia_mm / 2.0;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    constexpr double kEntryOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;
    gp_Pnt entry(in.position_x_mm, in.position_y_mm, zMax + kEntryOverhang);
    if (!(std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6 && adir.Z() < 0)) {
        const double bboxDiag = std::sqrt(
            (xMax - xMin) * (xMax - xMin) +
            (yMax - yMin) * (yMax - yMin) +
            (zMax - zMin) * (zMax - zMin));
        entry = gp_Pnt(
            in.position_x_mm - adir.X() * (bboxDiag + kEntryOverhang),
            in.position_y_mm - adir.Y() * (bboxDiag + kEntryOverhang),
            (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kEntryOverhang));
    }

    // Sub-feature 1: shaft-end face cut — short outer cylinder of
    // face_depth_mm at outer_dia.  Models the face turning pass.
    const gp_Ax2 axOuter(entry, adir);
    const TopoDS_Shape faceCut =
        pr::cylinder(axOuter, outerR, in.face_depth_mm + kEntryOverhang);

    // Sub-feature 2: raceway ring groove sitting just below the face.
    const gp_Pnt grooveOrigin(
        entry.X() + adir.X() * in.face_depth_mm,
        entry.Y() + adir.Y() * in.face_depth_mm,
        entry.Z() + adir.Z() * in.face_depth_mm);
    const gp_Ax2 axGroove(grooveOrigin, adir);
    // Raceway groove sits between innerR + 1 race-half and outerR - 1
    // race-half.  We use a thin annular ring just inside the outer race
    // (mid-radius = (innerR+outerR)/2) with the standard width/depth.
    const double midR = (innerR + outerR) / 2.0;
    const TopoDS_Shape raceway =
        pr::annularRing(axGroove,
                        midR + raceway_groove_depth,
                        midR - raceway_groove_depth,
                        raceway_groove_width);

    // Sub-feature 3: lock-nut pilot diameter — deeper cylinder of inner_dia
    // extending through the seat region.
    const TopoDS_Shape locknutPilot =
        pr::cylinder(axOuter, innerR,
                     in.face_depth_mm + in.seat_depth_mm + kEntryOverhang);

    // Sub-feature 4: thrust shoulder is the residual face produced where
    // the outer-cylinder face cut ends and the raw stock continues — it is
    // an EMERGENT face from sub-features 1+3 and not a separate tool.  We
    // log it in the signature as a derived sub-feature.

    const TopoDS_Shape fused1 = pr::fuse(faceCut,  raceway);
    const TopoDS_Shape fused  = pr::fuse(fused1,   locknutPilot);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { adir.X(), adir.Y(), adir.Z() } },
        { "outer_dia_mm",   in.outer_dia_mm },
        { "inner_dia_mm",   in.inner_dia_mm },
        { "face_depth_mm",  in.face_depth_mm },
        { "seat_depth_mm",  in.seat_depth_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           4 },
        { "outer_dia_mm",               in.outer_dia_mm },
        { "inner_dia_mm",               in.inner_dia_mm },
        { "face_depth_mm",              in.face_depth_mm },
        { "seat_depth_mm",              in.seat_depth_mm },
        { "raceway_groove_width_mm",    raceway_groove_width },
        { "raceway_groove_depth_mm",    raceway_groove_depth },
        { "raceway_groove_mid_dia_mm",  2.0 * midR },
        { "race_width_mm",              race_width },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type     = "facing_tool;groove_tool;boring_bar";
    tooling.tool_dia_mm   = in.outer_dia_mm;
    tooling.tool_material = "carbide";
    tooling.flute_count   = 1;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.06;
    tooling.stock_removed_mm3 =
        M_PI * outerR * outerR * in.face_depth_mm +
        M_PI * innerR * innerR * in.seat_depth_mm;
    tooling.est_cycle_time_s = std::max(4.0, in.seat_depth_mm / 20.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "facing_tool" },
              { "dia_mm", in.outer_dia_mm },
              { "depth_mm", in.face_depth_mm } },
            { { "tool_type", "groove_tool" },
              { "groove_mid_dia_mm", 2.0 * midR },
              { "width_mm", raceway_groove_width },
              { "depth_mm", raceway_groove_depth },
              { "note", "Timken thrust raceway" } },
            { { "tool_type", "boring_bar" },
              { "dia_mm", in.inner_dia_mm },
              { "depth_mm", in.face_depth_mm + in.seat_depth_mm },
              { "note", "lock-nut pilot — thread cut separately" } },
        } },
        { "derived_subfeatures", json::array({
            "shaft_end_face",
            "raceway_ring_groove",
            "locknut_pilot_dia",
            "thrust_shoulder",
        }) },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::thrust_bearing_seat_compound applied: od={} id={}",
                  in.outer_dia_mm, in.inner_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + geometric fallback ────────────────────
//
// Geometric signature of a thrust-bearing seat compound:
//   - TWO coaxial cylindrical faces sharing the same axis line:
//       * outer "face cut" cylinder (radius ≈ outer_dia/2, SHORT axially
//         — extent ≈ face_depth_mm);
//       * inner "lock-nut pilot" cylinder (radius ≈ inner_dia/2, LONGER
//         axially — extends through face_depth + seat_depth).
//   - axial ordering: both share the same entry plane; inner extends
//     deeper than outer along the drilling axis (thrust shoulder is the
//     emergent planar face between them).
//   - race width (outer−inner radius difference) ≥ 0.8 mm (DFM-002 floor).
// We tolerate the optional raceway groove as a third cylinder of small
// radial extent; it is recorded in matched_geometry but not required.

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
        d.axialMin    = proj(*mn);
        d.axialMax    = proj(*mx);
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
            if (std::abs(a.radius - b.radius) < 0.5) continue;     // need a real step
            const CylDesc& outer = (a.radius > b.radius) ? a : b;
            const CylDesc& inner = (a.radius > b.radius) ? b : a;

            // Race width DFM floor.
            const double race_width = 2.0 * (outer.radius - inner.radius);
            if (race_width < 1.6) continue;
            // Lock-nut shaft DFM floor (M3).
            if (2.0 * inner.radius < 3.0) continue;

            // Axial geometry: inner cylinder (pilot) should be substantially
            // longer than outer (face cut).
            const double outerLen = outer.axialMax - outer.axialMin;
            const double innerLen = inner.axialMax - inner.axialMin;
            if (outerLen < 0.2) continue;
            if (innerLen < outerLen + 0.5) continue;   // pilot must extend past face cut
            // Both must share entry plane within a tolerance.
            if (std::abs(outer.axialMin - inner.axialMin) > 0.5) continue;

            gp_Vec drillVec(outer.entryCenter, inner.deepCenter);
            if (drillVec.Magnitude() < 1e-6) continue;
            drillVec.Normalize();

            const double face_depth = outerLen;
            const double seat_depth = innerLen - outerLen;

            json recovered = {
                { "position_x_mm",  outer.entryCenter.X() },
                { "position_y_mm",  outer.entryCenter.Y() },
                { "axis_dir",       { drillVec.X(), drillVec.Y(), drillVec.Z() } },
                { "outer_dia_mm",   2.0 * outer.radius },
                { "inner_dia_mm",   2.0 * inner.radius },
                { "face_depth_mm",  face_depth },
                { "seat_depth_mm",  seat_depth },
            };
            json matched = {
                { "source",          "geometric_fallback" },
                { "outer_face_id",   outer.faceIdx },
                { "inner_face_id",   inner.faceIdx },
                { "race_width_mm",   race_width },
                { "face_depth_mm",   face_depth },
                { "seat_depth_mm",   seat_depth },
            };
            // Optional 3rd cylinder = raceway ring groove (mid-radius
            // between inner and outer, short axial extent).
            for (size_t k = 0; k < cyls.size(); ++k) {
                if (k == i || k == j) continue;
                const CylDesc& rg = cyls[k];
                if (!sameAxisInfinite(outer.axis, rg.axis)) continue;
                if (rg.radius <= inner.radius || rg.radius >= outer.radius) continue;
                const double rgLen = rg.axialMax - rg.axialMin;
                if (rgLen > race_width * 0.5) continue;     // groove must be narrow
                matched["raceway_groove_face_id"] = rg.faceIdx;
                matched["raceway_groove_width_mm"] = rgLen;
                matched["raceway_groove_mid_dia_mm"] = 2.0 * rg.radius;
                break;
            }

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

}  // namespace koocadcam::skill::thrust_bearing_seat_compound
