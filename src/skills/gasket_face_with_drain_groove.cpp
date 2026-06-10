// @lat: [[engine/skills#gasket_face_with_drain_groove]]

#include "gasket_face_with_drain_groove.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::gasket_face_with_drain_groove {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Gasket class spec table (ANSI B16.5 + DIN flanges; ≥10 entries) ───────
//
// Each entry: gasket class, recommended Ra (μm), preferred drain radius
// fraction of face_dia (per ASME B16.5 §6.4.4 vented gasket guidance).

namespace {

struct GasketClass {
    const char* class_name;
    double      ra_um;        // recommended face roughness
    double      drain_frac;   // drain_radius / face_dia
};

constexpr std::array<GasketClass, 10> kGasketTable {{
    { "ANSI-150",      3.2, 0.35 },
    { "ANSI-300",      3.2, 0.38 },
    { "ANSI-400",      3.2, 0.40 },
    { "ANSI-600",      1.6, 0.40 },
    { "ANSI-900",      1.6, 0.42 },
    { "ANSI-1500",     1.6, 0.42 },
    { "ANSI-2500",     0.8, 0.43 },
    { "DIN-PN10",      3.2, 0.35 },
    { "DIN-PN16",      3.2, 0.38 },
    { "DIN-PN40",      1.6, 0.40 },
}};

const GasketClass* lookupGasketClass(const std::string& key) {
    for (const auto& e : kGasketTable)
        if (key == e.class_name) return &e;
    return nullptr;
}

bool isKnownRoughness(const std::string& rc) {
    return rc == "smooth" || rc == "stock" || rc == "serrated";
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_dia_mm < 10.0) {
        r.add("DFM-GASKET-DIA", "error",
              "gasket_face: face_dia_mm " + std::to_string(in.face_dia_mm) +
              " < 10 mm (smallest practical gasket prep — ASME B16.5 NPS 1/4)");
    }
    if (in.relief_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gasket_face: relief_depth_mm must be > 0");
    }
    if (in.drain_radius_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "gasket_face: drain_radius_mm must be > 0");
    } else if (in.face_dia_mm > 0.0) {
        const double minR = in.face_dia_mm / 4.0;
        const double maxR = in.face_dia_mm / 2.0 - 1.0;
        if (in.drain_radius_mm < minR || in.drain_radius_mm > maxR) {
            r.add("DFM-GASKET-DRAIN", "error",
                  "drain_radius (" + std::to_string(in.drain_radius_mm) +
                  ") outside [" + std::to_string(minR) +
                  ", " + std::to_string(maxR) + "] — ASME B16.5 vented gasket");
        }
    }
    if (in.drain_width_mm < 0.5) {
        r.add("DFM-GASKET-DRAIN-WIDTH", "warning",
              "drain_width " + std::to_string(in.drain_width_mm) +
              " mm < 0.5 mm — may clog under flow");
    }
    if (in.drain_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "drain_depth_mm must be > 0");
    }
    if (!isKnownRoughness(in.roughness_class)) {
        r.add("DFM-GASKET-ROUGHNESS", "warning",
              "roughness_class '" + in.roughness_class +
              "' not in {smooth, stock, serrated} per ASME B16.5 Table 6");
    }
    auto faceId = wp.resolve(in.face_id);
    if (!faceId) {
        r.add("DFM-INPUT", "error", "face_id datum unresolved");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "gasket_face_with_drain_groove DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }
    auto faceId = wp.resolve(in.face_id);
    if (!faceId)
        throw SkillError("gasket_face_with_drain_groove: face_id unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    const gp_Dir adir = in.axis_dir;
    const double overhang = 0.05;
    const double entryZ = (adir.Z() < 0) ? (zMax + overhang) : (zMin - overhang);
    const double faceR  = in.face_dia_mm / 2.0;
    const double drainOR = in.drain_radius_mm + in.drain_width_mm / 2.0;
    const double drainIR = in.drain_radius_mm - in.drain_width_mm / 2.0;

    const gp_Pnt origin(in.center_x_mm, in.center_y_mm, entryZ);
    const gp_Ax2 ax(origin, adir);

    // Sub-feature 1: FACE RELIEF — shallow circular pocket.
    const TopoDS_Shape relief = pr::cylinder(
        ax, faceR, in.relief_depth_mm + overhang);

    // Sub-feature 2: ID 45° chamfer — cone widening outward at face surface,
    // tiny (0.5 mm) to ease sealing-bead contact.
    const double idR = std::max(1e-3, drainIR);
    const double idChamferRise =
        in.id_chamfer_mm * std::tan(45.0 * M_PI / 180.0);
    const TopoDS_Shape idChamfer = pr::coneFrustum(
        ax,
        /*r1 bottom=*/ idR,
        /*r2 top   =*/ std::max(1e-3, idR - idChamferRise),
        /*height   =*/ in.id_chamfer_mm + overhang);

    // Sub-feature 3: DRAIN GROOVE — concentric annular ring, deeper than relief.
    const TopoDS_Shape drainGroove = pr::annularRing(
        ax,
        /*outerR=*/ drainOR,
        /*innerR=*/ drainIR,
        /*height=*/ in.drain_depth_mm + in.relief_depth_mm + overhang);

    // Fuse-then-cut.
    const TopoDS_Shape f1   = pr::fuse(relief, drainGroove);
    const TopoDS_Shape full = pr::fuse(f1, idChamfer);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), full);

    const GasketClass* spec = lookupGasketClass(in.gasket_class);

    json params = {
        { "face_id_resolved",  *faceId },
        { "center_x_mm",       in.center_x_mm },
        { "center_y_mm",       in.center_y_mm },
        { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
        { "face_dia_mm",       in.face_dia_mm },
        { "relief_depth_mm",   in.relief_depth_mm },
        { "drain_radius_mm",   in.drain_radius_mm },
        { "drain_width_mm",    in.drain_width_mm },
        { "drain_depth_mm",    in.drain_depth_mm },
        { "id_chamfer_mm",     in.id_chamfer_mm },
        { "gasket_class",      in.gasket_class },
        { "roughness_class",   in.roughness_class },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    3 },
        { "spec_key",            in.gasket_class },
        { "face_dia_mm",         in.face_dia_mm },
        { "drain_radius_mm",     in.drain_radius_mm },
        { "drain_width_mm",      in.drain_width_mm },
        { "drain_depth_mm",      in.drain_depth_mm },
        { "relief_depth_mm",     in.relief_depth_mm },
        { "id_chamfer_mm",       in.id_chamfer_mm },
        { "roughness_class",     in.roughness_class },
        { "recommended_ra_um",   spec ? spec->ra_um : 0.0 },
        { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "face_mill;form_tool;chamfer_mill";
    tooling.tool_dia_mm       = in.face_dia_mm;
    tooling.tool_length_mm    = 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 500.0;
    tooling.feed_per_tooth_mm = 0.08;
    tooling.stock_removed_mm3 =
        M_PI * faceR * faceR * in.relief_depth_mm +
        M_PI * (drainOR * drainOR - drainIR * drainIR) * in.drain_depth_mm;
    tooling.est_cycle_time_s  = std::max(4.0, in.face_dia_mm / 10.0);
    tooling.extra = {
        { "subfeature_sequence", {
            { { "kind", "face_relief" },
              { "dia_mm", in.face_dia_mm },
              { "depth_mm", in.relief_depth_mm } },
            { { "kind", "id_chamfer" },
              { "angle_deg", 45 },
              { "size_mm", in.id_chamfer_mm } },
            { { "kind", "concentric_drain_groove" },
              { "radius_mm", in.drain_radius_mm },
              { "width_mm",  in.drain_width_mm },
              { "depth_mm",  in.drain_depth_mm } },
        } },
        { "standard",        "ASME B16.5 §6.4.4 (vented gasket)" },
        { "gasket_class",    in.gasket_class },
        { "roughness_class", in.roughness_class },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::gasket_face_with_drain_groove: face={} relief={} drain r={} w={} d={}",
                  in.face_dia_mm, in.relief_depth_mm,
                  in.drain_radius_mm, in.drain_width_mm, in.drain_depth_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + context-aware planar+drain search ─────

namespace {

// Look for a shallow circular planar relief + a concentric thin annular
// groove deeper than it.  This is the geometric signature of a gasket
// face with a drain groove.

struct CylDescr {
    int        faceIdx;
    double     radius;
    gp_Ax1     axis;
};

std::vector<CylDescr> collectCyls(const Workpiece& wp)
{
    std::vector<CylDescr> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder c = s.Cylinder();
        out.push_back(CylDescr{ i, c.Radius(), c.Axis() });
    }
    return out;
}

bool axesColinear(const gp_Ax1& a, const gp_Ax1& b,
                  double angTolDeg = 1.0, double posTolMm = 0.2)
{
    const gp_Dir da = a.Direction(), db = b.Direction();
    const double dot = std::abs(da.X()*db.X() + da.Y()*db.Y() + da.Z()*db.Z());
    if (dot < std::cos(angTolDeg * M_PI / 180.0)) return false;
    gp_Vec v(a.Location(), b.Location());
    gp_Vec axV(da.X(), da.Y(), da.Z());
    gp_Vec perp = v - axV * v.Dot(axV);
    return perp.Magnitude() < posTolMm;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Metadata replay (preferred).
    for (const auto& feat : wp.features()) {
        if (feat.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = feat.params;
        r.confidence       = 1.0;
        r.matched_geometry = {
            { "source",  "metadata_replay" },
            { "pattern", feat.pattern },
        };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Geometric fallback: ≥ 3 coaxial cylindrical faces (face relief OD +
    // drain OD + drain ID).  We pick the triple where the largest is the
    // outer face and the inner two are the drain walls (close in radius).
    const auto cyls = collectCyls(wp);
    if (cyls.size() < 3) return out;

    for (size_t i = 0; i < cyls.size(); ++i) {
        std::vector<int> coaxialIdx{ static_cast<int>(i) };
        for (size_t j = 0; j < cyls.size(); ++j) {
            if (j == i) continue;
            if (axesColinear(cyls[i].axis, cyls[j].axis))
                coaxialIdx.push_back(static_cast<int>(j));
        }
        if (coaxialIdx.size() < 3) continue;

        std::sort(coaxialIdx.begin(), coaxialIdx.end(),
                  [&](int a, int b) { return cyls[a].radius > cyls[b].radius; });
        const double faceR  = cyls[coaxialIdx[0]].radius;       // largest
        const double drnOR  = cyls[coaxialIdx[1]].radius;       // outer drain
        const double drnIR  = cyls[coaxialIdx[2]].radius;       // inner drain
        if (drnOR - drnIR < 0.2 || drnOR - drnIR > 5.0) continue;
        if (faceR - drnOR < 0.5) continue;
        const double drainR = (drnOR + drnIR) / 2.0;
        const double drainW = drnOR - drnIR;
        const gp_Dir adir   = cyls[coaxialIdx[0]].axis.Direction();
        const gp_Pnt aOrg   = cyls[coaxialIdx[0]].axis.Location();
        json recovered = {
            { "center_x_mm",     aOrg.X() },
            { "center_y_mm",     aOrg.Y() },
            { "axis_dir",        { adir.X(), adir.Y(), adir.Z() } },
            { "face_dia_mm",     2.0 * faceR },
            { "drain_radius_mm", drainR },
            { "drain_width_mm",  drainW },
            { "drain_depth_mm",  0.5 },
            { "relief_depth_mm", 0.05 },
            { "id_chamfer_mm",   0.5 },
            { "gasket_class",    "" },
            { "roughness_class", "stock" },
        };
        json matched = {
            { "source",        "geometric_fallback" },
            { "face_face_id",  cyls[coaxialIdx[0]].faceIdx },
            { "drain_od_id",   cyls[coaxialIdx[1]].faceIdx },
            { "drain_id_id",   cyls[coaxialIdx[2]].faceIdx },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.50, matched });
        break;
    }
    return out;
}

}  // namespace koocadcam::skill::gasket_face_with_drain_groove
