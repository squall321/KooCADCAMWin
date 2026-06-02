// @lat: [[engine/skills#socket_head_bolt_seat]]

#include "socket_head_bolt_seat.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
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
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace koocadcam::skill::socket_head_bolt_seat {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

// All three columns (clearance, socket-head OD, socket-head height) for
// M3..M8 are sourced from the central thread table (_iso_thread_table.hpp).
// The local table retains only the sub-M3 sizes (M2, M2.5) that are not in
// the central 11-row catalogue.
struct ShcsEntry {
    const char* size;
    double      clearance_mm;
    double      head_dia_mm;
    double      head_height_mm;
};

constexpr std::array<ShcsEntry, 2> kSubM3Table {{
    { "M2",   2.4,  3.8,  2.0 },
    { "M2.5", 2.9,  4.5,  2.5 },
}};

const ShcsEntry* lookupSubM3(const std::string& s) {
    for (const auto& e : kSubM3Table) if (s == e.size) return &e;
    return nullptr;
}

struct ShcsSpec {
    bool   found;
    double clearance_mm;
    double head_dia_mm;
    double head_height_mm;
};

ShcsSpec lookupSpec(const std::string& s) {
    if (const auto* thr = thread_table::findMetric(s)) {
        return ShcsSpec{ true,
                         thr->clearance_medium_mm,
                         thr->socket_head_dia_mm,
                         thr->socket_head_height_mm };
    }
    if (const auto* e = lookupSubM3(s)) {
        return ShcsSpec{ true, e->clearance_mm, e->head_dia_mm, e->head_height_mm };
    }
    return ShcsSpec{ false, 0.0, 0.0, 0.0 };
}

constexpr double kChamferDepth_mm = 0.3;
constexpr double kSeatExtraDepth_mm = 0.5;  // head_height + 0.5 mm

}  // namespace

double clearanceDiameterFor(const std::string& s)
{ const auto sp = lookupSpec(s); return sp.found ? sp.clearance_mm : 0.0; }

double headDiameterFor(const std::string& s)
{ const auto sp = lookupSpec(s); return sp.found ? sp.head_dia_mm : 0.0; }

double headHeightFor(const std::string& s)
{ const auto sp = lookupSpec(s); return sp.found ? sp.head_height_mm : 0.0; }

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.fastener_size.empty()) {
        r.add("DFM-SHCS-SIZE", "error",
              "socket_head_bolt_seat: fastener_size is empty");
        return r;
    }
    const auto sp = lookupSpec(in.fastener_size);
    if (!sp.found) {
        r.add("DFM-SHCS-SIZE", "error",
              "socket_head_bolt_seat: unknown fastener_size '" +
              in.fastener_size +
              "' (supported: M2/M2.5/M3/M4/M5/M6/M8)");
        return r;
    }

    if (sp.clearance_mm < 0.8) {
        r.add("DFM-002", "error",
              "socket_head_bolt_seat: pilot " +
              std::to_string(sp.clearance_mm) + " < 0.8 mm");
    }

    const double seatDia = sp.head_dia_mm + in.head_slip_mm;
    if (seatDia <= sp.clearance_mm) {
        r.add("DFM-SHCS-MARGIN", "error",
              "socket_head_bolt_seat: seat dia " + std::to_string(seatDia) +
              " ≤ pilot dia " + std::to_string(sp.clearance_mm) +
              " — invalid counterbore");
    }

    const double seatDepth = sp.head_height_mm + kSeatExtraDepth_mm;
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness = zMax - zMin;
    if (thickness > 0.0 && thickness < seatDepth + 1.0) {
        r.add("DFM-SHCS-THICKNESS", "error",
              "socket_head_bolt_seat: stock thickness " +
              std::to_string(thickness) + " mm < counterbore depth + 1 mm (" +
              std::to_string(seatDepth + 1.0) + " mm)");
    }

    if (in.head_slip_mm < 0.0 || in.head_slip_mm > 2.0) {
        r.add("DFM-INPUT", "warning",
              "socket_head_bolt_seat: head_slip_mm " +
              std::to_string(in.head_slip_mm) +
              " outside typical 0..2 mm");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "socket_head_bolt_seat DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto sp = lookupSpec(in.fastener_size);
    const double pilotDia  = sp.clearance_mm;
    const double seatDia   = sp.head_dia_mm + in.head_slip_mm;
    const double seatDepth = sp.head_height_mm + kSeatExtraDepth_mm;

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("socket_head_bolt_seat: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const double kOverhang = 0.05;
    const gp_Dir adir = in.axis_dir;

    gp_Pnt toolStart(
        in.position_x_mm - adir.X() * (bboxDiag + kOverhang),
        in.position_y_mm - adir.Y() * (bboxDiag + kOverhang),
        (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kOverhang));
    gp_Pnt entryPlanePoint(toolStart);

    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        const double entryZ = (adir.Z() < 0) ? zMax : zMin;
        toolStart       = gp_Pnt(in.position_x_mm, in.position_y_mm,
                                 entryZ - adir.Z() * kOverhang);
        entryPlanePoint = gp_Pnt(in.position_x_mm, in.position_y_mm, entryZ);
    }

    const gp_Ax2 pilotAx (toolStart,       adir);
    const gp_Ax2 seatAx  (toolStart,       adir);
    const gp_Ax2 chamfAx (entryPlanePoint, adir);

    // ── Sub-feature 1: through pilot ─────────────────────────────────────
    const TopoDS_Shape pilotTool =
        pr::cylinder(pilotAx, pilotDia / 2.0, bboxDiag + 2.0 * kOverhang);

    // ── Sub-feature 2: counterbore seat ──────────────────────────────────
    const TopoDS_Shape seatTool =
        pr::cylinder(seatAx, seatDia / 2.0, seatDepth + kOverhang);

    // ── Sub-feature 3: chamfer (small 45° cone at entry) ─────────────────
    // r1 at entry plane = seat_dia/2 + chamfer_depth; r2 at chamfer_depth
    // along adir = seat_dia/2.  Models a ~45° entry chamfer.
    const double chamferTopR = seatDia / 2.0 + kChamferDepth_mm;
    const double chamferBotR = seatDia / 2.0;
    const TopoDS_Shape chamferTool =
        pr::coneFrustum(chamfAx, chamferTopR, chamferBotR, kChamferDepth_mm);

    // Fuse concentric cutters, then single cut.
    TopoDS_Shape fused = pr::fuse(pilotTool, seatTool);
    fused = pr::fuse(fused, chamferTool);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "entry_face_id",  *entryId },
        { "position_x_mm",  in.position_x_mm },
        { "position_y_mm",  in.position_y_mm },
        { "axis_dir",       { adir.X(), adir.Y(), adir.Z() } },
        { "fastener_size",  in.fastener_size },
        { "head_slip_mm",   in.head_slip_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    3 },
        { "fastener_size",       in.fastener_size },
        { "pilot_dia_mm",        pilotDia },
        { "seat_dia_mm",         seatDia },
        { "seat_depth_mm",       seatDepth },
        { "chamfer_depth_mm",    kChamferDepth_mm },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
        { "through_pilot",       true },
    };

    ToolingMeta tooling;
    tooling.tool_type     = "drill;counterbore;chamfer";
    tooling.tool_dia_mm   = seatDia;
    tooling.tool_length_mm = bboxDiag * 1.2 + 5.0;
    tooling.tool_material = "carbide";
    tooling.flute_count   = 2;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.05;
    const double pilotR = pilotDia / 2.0;
    const double seatR  = seatDia  / 2.0;
    const double pilotVol = M_PI * pilotR * pilotR * (zMax - zMin);
    const double seatAnnVol = M_PI * (seatR * seatR - pilotR * pilotR) * seatDepth;
    tooling.stock_removed_mm3 = pilotVol + seatAnnVol;
    tooling.est_cycle_time_s  = std::max(1.0, (zMax - zMin) / 40.0);
    tooling.extra = {
        { "tool_sequence", {
            { { "tool_type", "drill" },       { "tool_dia_mm", pilotDia } },
            { { "tool_type", "counterbore" }, { "tool_dia_mm", seatDia },
              { "depth_mm",  seatDepth } },
            { { "tool_type", "chamfer" },     { "tool_dia_mm", seatDia + 2.0 * kChamferDepth_mm },
              { "depth_mm",  kChamferDepth_mm } },
        } },
        { "fastener_standard", "ISO_4762_DIN_912" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::socket_head_bolt_seat applied: {} seat {}×{}",
                  in.fastener_size, seatDia, seatDepth);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + geometric fallback ────────────────────
//
// Geometric signature of a socket-head bolt seat (counterbore):
//   - TWO coaxial cylindrical faces:
//       * "seat" cylinder (wider, radius matches head_dia/2 within slip);
//       * "pilot" cylinder (narrower, radius matches clearance_mm/2).
//   - the seat is SHORTER axially than the pilot (seat depth ≈ head height).
//   - the two share an axis line within tolerance.

namespace {

struct CylDesc {
    int    faceIdx;
    gp_Ax1 axis;
    double radius;
    double axialMin;
    double axialMax;
    gp_Pnt entryCenter;  // shallow circle center
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
            if (std::abs(a.radius - b.radius) < 0.2) continue;
            const CylDesc& seat  = (a.radius > b.radius) ? a : b;
            const CylDesc& pilot = (a.radius > b.radius) ? b : a;
            // Pilot must be substantially longer than seat (counterbore pattern).
            const double seatLen  = seat.axialMax  - seat.axialMin;
            const double pilotLen = pilot.axialMax - pilot.axialMin;
            if (pilotLen < seatLen * 1.5) continue;
            // Snap to nearest M-size using pilot diameter.  Candidates are
            // M2/M2.5 (local sub-M3 table) plus M3..M30 (central table).
            std::string bestSize = "M4";
            double sizeErr = std::numeric_limits<double>::max();
            for (const auto& e : kSubM3Table) {
                const double err = std::abs(e.clearance_mm - 2.0 * pilot.radius);
                if (err < sizeErr) { sizeErr = err; bestSize = e.size; }
            }
            for (const auto& thr : thread_table::kMetricThreads) {
                const double err = std::abs(thr.clearance_medium_mm - 2.0 * pilot.radius);
                if (err < sizeErr) { sizeErr = err; bestSize = thr.size_key; }
            }
            if (sizeErr > 0.5) continue;
            const auto mat = lookupSpec(bestSize);
            if (!mat.found) continue;
            const double headSlip = std::max(0.0,
                2.0 * seat.radius - mat.head_dia_mm);
            if (headSlip > 2.0) continue;  // outside reasonable counterbore tolerance

            gp_Vec drillVec(seat.entryCenter, pilot.deepCenter);
            if (drillVec.Magnitude() < 1e-6) continue;
            drillVec.Normalize();

            json recovered = {
                { "position_x_mm", seat.entryCenter.X() },
                { "position_y_mm", seat.entryCenter.Y() },
                { "axis_dir",      { drillVec.X(), drillVec.Y(), drillVec.Z() } },
                { "fastener_size", bestSize },
                { "head_slip_mm",  headSlip },
            };
            json matched = {
                { "source",          "geometric_fallback" },
                { "seat_face_id",    seat.faceIdx },
                { "pilot_face_id",   pilot.faceIdx },
                { "seat_depth_mm",   seatLen },
                { "pilot_depth_mm",  pilotLen },
                { "size_err_mm",     sizeErr },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.65, matched });
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
        rf.matched_geometry = { { "source", "metadata" },
                                { "is_compound", true } };
        out.push_back(rf);
    }
    if (out.empty()) {
        out = geometric_fallback(wp);
    }
    return out;
}

}  // namespace koocadcam::skill::socket_head_bolt_seat
