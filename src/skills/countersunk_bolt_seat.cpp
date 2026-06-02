// @lat: [[engine/skills#countersunk_bolt_seat]]

#include "countersunk_bolt_seat.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Circ.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace koocadcam::skill::countersunk_bolt_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── M-spec lookup tables ─────────────────────────────────────────────────
//
// Clearance (ISO 273 medium) for M3..M8 comes from the central thread table
// (_iso_thread_table.hpp).  The local table keeps only sub-M3 sizes (M2,
// M2.5 — absent from the central 11-row catalogue) and the ISO 7046
// flat-head OD column, which is unique to this skill.
namespace {

struct CskEntry {
    const char* size;
    double      clearance_mm;   // valid only for M2/M2.5 rows; 0.0 means use central
    double      head_dia_mm;    // ISO 7046 flat-head OD
};

constexpr std::array<CskEntry, 7> kTable {{
    { "M2",   2.4,  3.8  },
    { "M2.5", 2.9,  4.7  },
    { "M3",   0.0,  5.6  },
    { "M4",   0.0,  7.5  },
    { "M5",   0.0,  9.2  },
    { "M6",   0.0,  11.0 },
    { "M8",   0.0,  14.5 },
}};

const CskEntry* lookupEntry(const std::string& s) {
    for (const auto& e : kTable) if (s == e.size) return &e;
    return nullptr;
}

double clearanceForSize(const std::string& s) {
    if (const auto* thr = thread_table::findMetric(s)) return thr->clearance_medium_mm;
    if (const auto* e = lookupEntry(s)) return e->clearance_mm;
    return 0.0;
}

}  // namespace

double clearanceDiameterFor(const std::string& size)
{
    return clearanceForSize(size);
}

double headDiameterFor(const std::string& size)
{
    const auto* e = lookupEntry(size);
    return e ? e->head_dia_mm : 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.fastener_size.empty()) {
        r.add("DFM-CSK-SIZE", "error",
              "countersunk_bolt_seat: fastener_size is empty");
        return r;
    }
    const auto* e = lookupEntry(in.fastener_size);
    if (!e) {
        r.add("DFM-CSK-SIZE", "error",
              "countersunk_bolt_seat: unknown fastener_size '" +
              in.fastener_size +
              "' (supported: M2/M2.5/M3/M4/M5/M6/M8)");
        return r;
    }

    // DFM-002 — minimum pilot diameter (matches drill_hole floor per ISO 235).
    constexpr double kMinPilotDiaMm = 0.8;  // ISO 235:2016 smallest standard HSS drill
    const double pilotForCheck = clearanceForSize(in.fastener_size);
    if (pilotForCheck < kMinPilotDiaMm) {
        r.add("DFM-002", "error",
              "countersunk_bolt_seat: derived pilot " +
              std::to_string(pilotForCheck) +
              " mm < min 0.8 mm (ISO 235:2016 smallest standard HSS drill)");
    }

    // DFM-CSK-ANGLE — standardised flat-head included angles.
    // Standards reference:
    //   - 82° : ASME B18.6.3 (UNC/UNF flat-head machine screws — inch)
    //   - 90° : ISO 7046-1 / DIN 965 (metric flat-head machine screws)
    //   - 100°: SAE/AS aerospace high-strength flat-head (NAS 4400 series)
    const double a = in.csk_angle_deg;
    if (std::abs(a - 82.0) > 1e-3 && std::abs(a - 90.0) > 1e-3 &&
        std::abs(a - 100.0) > 1e-3) {
        r.add("DFM-CSK-ANGLE", "error",
              "countersunk_bolt_seat: csk_angle " + std::to_string(a) +
              " deg not in {82 (ASME B18.6.3), 90 (ISO 7046-1), 100 (SAE AS aerospace)}");
    }

    // DFM-CSK-THICKNESS — stock must be thick enough that the head sits
    // entirely above the far face.
    // Standards reference: ISO 7046-1:2011 Table 1 (and DIN 965 / ASME
    // B18.6.3 Table 2) tabulate the flat-head height k for each metric
    // size; the ratio k / head_OD is ≈ 0.6 across the M2..M8 range (e.g.
    // M4: k = 2.7 mm, head_OD = 7.5 mm → 0.36; conservative envelope
    // 0.6 covers the spec head height PLUS a small finish allowance and
    // protects against the cone exiting the back face under tilted
    // (non-axial) drilling.
    constexpr double kIsoHeadHeightRatio = 0.6;  // ISO 7046-1:2011 head-height envelope (k + allowance) / head_OD
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness > 0.0 && thickness < e->head_dia_mm * kIsoHeadHeightRatio) {
        r.add("DFM-CSK-THICKNESS", "error",
              "countersunk_bolt_seat: stock thickness " +
              std::to_string(thickness) + " mm < ISO 7046-1 head-height envelope ≈ 0.6 × head_OD (" +
              std::to_string(e->head_dia_mm * kIsoHeadHeightRatio) +
              " mm) — head will sink past the far face");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "countersunk_bolt_seat DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* e = lookupEntry(in.fastener_size);
    const double pilotDia   = clearanceForSize(in.fastener_size);
    const double headDia    = e->head_dia_mm;
    const double angleRad   = in.csk_angle_deg * M_PI / 180.0;
    const double coneDepth  = (headDia - pilotDia) * 0.5 / std::tan(angleRad / 2.0);

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("countersunk_bolt_seat: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    // Default toolStart (general-axis case)
    gp_Pnt toolStartCyl(
        in.position_x_mm - adir.X() * (bboxDiag + kOverhang),
        in.position_y_mm - adir.Y() * (bboxDiag + kOverhang),
        (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kOverhang));
    gp_Pnt entryPlanePoint(toolStartCyl);

    // Common-case shortcut: drilling along ±Z, planar entry.
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        const double entryZ = (adir.Z() < 0) ? zMax : zMin;
        toolStartCyl    = gp_Pnt(in.position_x_mm, in.position_y_mm,
                                 entryZ - adir.Z() * kOverhang);
        entryPlanePoint = gp_Pnt(in.position_x_mm, in.position_y_mm, entryZ);
    }

    // gp_Ax2(P, V_axial, V_x): V is local Z (axial); for a through-hole the
    // pilot extends bboxDiag + 2·overhang along adir from start.
    const gp_Ax2 pilotAx(toolStartCyl,    adir);
    const gp_Ax2 coneAx (entryPlanePoint, adir);

    // ── Sub-feature 1: through pilot drill ───────────────────────────────
    const double pilotHeight = bboxDiag + 2.0 * kOverhang;
    const TopoDS_Shape pilotTool =
        pr::cylinder(pilotAx, pilotDia / 2.0, pilotHeight);

    // ── Sub-feature 2: conical countersink ───────────────────────────────
    // r1 at entry plane = headDia/2; r2 at +coneDepth = pilotDia/2.
    const TopoDS_Shape coneTool =
        pr::coneFrustum(coneAx, headDia / 2.0, pilotDia / 2.0, coneDepth);

    // ── Sub-feature 3 (optional): tiny flat-bottom relief ────────────────
    // 0.5 mm extra cylindrical cut at junction so head doesn't bottom on
    // junction radius.  We model it as a slightly enlarged cylinder at the
    // cone-to-pilot transition.
    std::vector<TopoDS_Shape> tools{ pilotTool, coneTool };
    int subFeatureCount = 2;
    double reliefDepth = 0.0;
    double reliefDia   = 0.0;
    if (in.with_relief) {
        reliefDia   = pilotDia + 0.4;             // 0.2 mm radial relief
        reliefDepth = 0.5;
        // Relief sits just past the cone bottom along adir.
        const gp_Pnt reliefStart(
            entryPlanePoint.X() + adir.X() * coneDepth,
            entryPlanePoint.Y() + adir.Y() * coneDepth,
            entryPlanePoint.Z() + adir.Z() * coneDepth);
        const gp_Ax2 reliefAx(reliefStart, adir);
        tools.push_back(pr::cylinder(reliefAx, reliefDia / 2.0, reliefDepth));
        subFeatureCount = 3;
    }

    // Fuse all concentric overlapping cutters into one, then single cut.
    TopoDS_Shape fused = tools[0];
    for (size_t i = 1; i < tools.size(); ++i) fused = pr::fuse(fused, tools[i]);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { adir.X(), adir.Y(), adir.Z() } },
        { "fastener_size",  in.fastener_size },
        { "csk_angle_deg",  in.csk_angle_deg },
        { "with_relief",    in.with_relief },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    subFeatureCount },
        { "fastener_size",       in.fastener_size },
        { "pilot_dia_mm",        pilotDia },
        { "head_dia_mm",         headDia },
        { "csk_angle_deg",       in.csk_angle_deg },
        { "cone_depth_mm",       coneDepth },
        { "relief_dia_mm",       reliefDia },
        { "relief_depth_mm",     reliefDepth },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
        { "through_pilot",       true },
    };

    ToolingMeta tooling;
    tooling.tool_type   = "drill;countersink";
    tooling.tool_dia_mm = headDia;
    tooling.tool_length_mm = bboxDiag * 1.2 + 5.0;
    tooling.tool_material  = "carbide";
    tooling.flute_count    = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double pilotR = pilotDia / 2.0;
    const double headR  = headDia  / 2.0;
    const double frustVol = (M_PI * coneDepth / 3.0) *
                            (headR * headR + headR * pilotR + pilotR * pilotR);
    const double pilotVol = M_PI * pilotR * pilotR * (zMax - zMin);
    const double reliefVol = in.with_relief
        ? M_PI * (reliefDia / 2.0) * (reliefDia / 2.0) * reliefDepth
        : 0.0;
    tooling.stock_removed_mm3 = pilotVol + std::max(0.0, frustVol - pilotR * pilotR * M_PI * coneDepth) + reliefVol;
    tooling.est_cycle_time_s  = std::max(1.0, (zMax - zMin) / 40.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },       { "tool_dia_mm", pilotDia } },
            { { "tool_type", "countersink" }, { "tool_dia_mm", headDia },
              { "cone_angle_deg", in.csk_angle_deg } },
            { { "tool_type", in.with_relief ? "endmill" : "" },
              { "tool_dia_mm",  reliefDia },
              { "depth_mm",     reliefDepth } },
        } },
        { "fastener_standard",
          (std::abs(in.csk_angle_deg - 82.0) < 1e-3) ? "UNC_82deg" :
          (std::abs(in.csk_angle_deg - 100.0) < 1e-3) ? "AS_aerospace_100deg" :
          "ISO_7046_90deg" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::countersunk_bolt_seat applied: {} csk={}° subs={}",
                  in.fastener_size, in.csk_angle_deg, subFeatureCount);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + geometric fallback ────────────────────

namespace {

struct CylDesc { int faceIdx; gp_Ax1 axis; double radius; gp_Pnt top, bot; };
struct ConeDesc {
    int    faceIdx;
    gp_Ax1 axis;
    double rSmall, rLarge;
    gp_Pnt cSmall, cLarge;
    double semiAngleRad;
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
        CylDesc d; d.faceIdx = i; d.axis = cy.Axis(); d.radius = cy.Radius();
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
        d.top = *std::min_element(centers.begin(), centers.end(),
            [&](const gp_Pnt& A, const gp_Pnt& B){ return proj(A) < proj(B); });
        d.bot = *std::max_element(centers.begin(), centers.end(),
            [&](const gp_Pnt& A, const gp_Pnt& B){ return proj(A) < proj(B); });
        out.push_back(d);
    }
    return out;
}

std::vector<ConeDesc> collectCones(const Workpiece& wp)
{
    std::vector<ConeDesc> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        BRepAdaptor_Surface surf(wp.face(i));
        if (surf.GetType() != GeomAbs_Cone) continue;
        const gp_Cone cn = surf.Cone();
        ConeDesc d;
        d.faceIdx = i;
        d.axis    = cn.Axis();
        d.semiAngleRad = std::abs(cn.SemiAngle());
        std::vector<std::pair<double, gp_Pnt>> circs;
        for (TopExp_Explorer exp(wp.face(i), TopAbs_EDGE); exp.More(); exp.Next()) {
            BRepAdaptor_Curve crv(TopoDS::Edge(exp.Current()));
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(d.axis.Direction())) - 1.0) > 1e-3)
                continue;
            circs.emplace_back(c.Radius(), c.Location());
        }
        if (circs.size() < 2) continue;
        auto mn = std::min_element(circs.begin(), circs.end(),
            [](const auto& a, const auto& b){ return a.first < b.first; });
        auto mx = std::max_element(circs.begin(), circs.end(),
            [](const auto& a, const auto& b){ return a.first < b.first; });
        if (std::abs(mx->first - mn->first) < 0.05) continue;
        d.rSmall = mn->first;   d.cSmall = mn->second;
        d.rLarge = mx->first;   d.cLarge = mx->second;
        out.push_back(d);
    }
    return out;
}

std::vector<RecognizedFeature> geometric_fallback(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    const auto cyls  = collectCyls(wp);
    const auto cones = collectCones(wp);
    if (cyls.empty() || cones.empty()) return out;

    for (const auto& cone : cones) {
        const double saDeg = cone.semiAngleRad * 180.0 / M_PI;
        double cskAngle = 0.0;
        double bestErr  = 5.0;
        for (double cand : { 41.0, 45.0, 50.0 }) {
            const double err = std::abs(cand - saDeg);
            if (err < bestErr) { bestErr = err; cskAngle = 2.0 * cand; }
        }
        if (cskAngle == 0.0) continue;
        for (const auto& cyl : cyls) {
            if (!sameAxisInfinite(cone.axis, cyl.axis)) continue;
            if (std::abs(cone.rSmall - cyl.radius) > 0.1) continue;
            std::string bestSize = "M3";
            double sizeErr = std::numeric_limits<double>::max();
            for (const auto& e : kTable) {
                const double rowClr = clearanceForSize(e.size);
                const double err = std::abs(rowClr - 2.0 * cyl.radius);
                if (err < sizeErr) { sizeErr = err; bestSize = e.size; }
            }
            if (sizeErr > 0.5) continue;
            gp_Vec drillVec(cone.cLarge, cyl.bot);
            if (drillVec.Magnitude() < 1e-6) continue;
            drillVec.Normalize();

            json recovered = {
                { "position_x_mm",  cone.cLarge.X() },
                { "position_y_mm",  cone.cLarge.Y() },
                { "axis_dir",       { drillVec.X(), drillVec.Y(), drillVec.Z() } },
                { "fastener_size",  bestSize },
                { "csk_angle_deg",  cskAngle },
                { "with_relief",    false },
            };
            json matched = {
                { "source",              "geometric_fallback" },
                { "cone_face_id",        cone.faceIdx },
                { "pilot_face_id",       cyl.faceIdx },
                { "cone_semi_angle_deg", saDeg },
                { "size_err_mm",         sizeErr },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.65, matched });
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
        rf.matched_geometry = { { "source", "metadata" },
                                { "is_compound", true } };
        out.push_back(rf);
    }
    if (out.empty()) {
        out = geometric_fallback(wp);
    }
    return out;
}

}  // namespace koocadcam::skill::countersunk_bolt_seat
