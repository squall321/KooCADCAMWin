// @lat: [[engine/skills#dust_lip_seal_seat_compound]]

#include "dust_lip_seal_seat_compound.hpp"

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

namespace koocadcam::skill::dust_lip_seal_seat_compound {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── SKF / National / CR standard seal-series table (≥10 entries) ─────────
//
// Reference: SKF "CR Seals Handbook" Table 1 (selected rotary shaft seal
// series — nominal shaft × housing × seal width in mm).

namespace {

struct SealSeriesEntry {
    const char* series;        // SKF/CR designation
    double shaft_dia_mm;       // running shaft diameter
    double housing_dia_mm;     // housing bore (= shaft + 2 × seal OD wall)
    double width_mm;           // axial seal thickness
};

constexpr std::array<SealSeriesEntry, 10> kSealSpec {{
    { "CR-10-22-7",  10.0, 22.0, 7.0 },
    { "CR-15-30-7",  15.0, 30.0, 7.0 },
    { "CR-20-32-7",  20.0, 32.0, 7.0 },
    { "CR-25-37-7",  25.0, 37.0, 7.0 },
    { "CR-30-42-7",  30.0, 42.0, 7.0 },
    { "CR-35-50-8",  35.0, 50.0, 8.0 },
    { "CR-40-55-8",  40.0, 55.0, 8.0 },
    { "CR-50-65-8",  50.0, 65.0, 8.0 },
    { "CR-60-80-10", 60.0, 80.0, 10.0 },
    { "CR-80-100-10",80.0,100.0, 10.0 },
}};

const SealSeriesEntry* lookupSealSeries(const std::string& key) {
    for (const auto& e : kSealSpec)
        if (key == e.series) return &e;
    return nullptr;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.bore_dia_mm < 3.0) {
        r.add("DFM-SEAL-DIA", "error",
              "dust_lip_seal_seat_compound: bore_dia_mm " +
              std::to_string(in.bore_dia_mm) +
              " < 3 mm (smallest CR rotary seal shaft per SKF Handbook)");
    }
    if (in.bore_depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "dust_lip_seal_seat_compound: bore_depth_mm must be > 0");
    }
    if (in.bore_depth_mm > 0.0 && in.bore_depth_mm < in.seal_thickness_mm) {
        r.add("DFM-SEAL-DEPTH", "error",
              "bore_depth (" + std::to_string(in.bore_depth_mm) +
              ") < seal_thickness (" + std::to_string(in.seal_thickness_mm) +
              ") — seal cannot fit in housing");
    }
    if (in.lip_relief_depth_mm <= 0.0) {
        r.add("DFM-LIP-RELIEF", "error",
              "lip_relief_depth_mm must be > 0 (dust lip needs an undercut)");
    }
    if (in.lip_relief_depth_mm > in.bore_dia_mm / 4.0) {
        r.add("DFM-LIP-RELIEF", "error",
              "lip_relief_depth (" + std::to_string(in.lip_relief_depth_mm) +
              ") > bore_dia/4 — lip would lose contact area");
    }
    if (in.seal_can_OD_offset_mm < 0.1) {
        r.add("DFM-SEAL-PRESSFIT", "warning",
              "seal_can_OD_offset " + std::to_string(in.seal_can_OD_offset_mm) +
              " mm < 0.1 mm — press-fit interference may be insufficient");
    }
    if (in.nose_chamfer_mm < 0.1) {
        r.add("DFM-NOSE-CHAMFER", "warning",
              "nose_chamfer_mm " + std::to_string(in.nose_chamfer_mm) +
              " mm < 0.1 mm — recommend 0.25-0.5 mm 30° chamfer (SKF §lead-in)");
    }
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) {
        r.add("DFM-INPUT", "error", "entry_face datum unresolved");
    }

    // Context check: housing wall must fit the counterbore.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double maxExtent = std::min(xMax - xMin, yMax - yMin);
    const double cbDia = in.bore_dia_mm + 2.0 * in.seal_can_OD_offset_mm;
    if (cbDia > maxExtent) {
        r.add("DFM-SEAL-FIT", "warning",
              "counterbore OD " + std::to_string(cbDia) +
              " > workpiece footprint " + std::to_string(maxExtent));
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "dust_lip_seal_seat_compound DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId)
        throw SkillError("dust_lip_seal_seat_compound: entry_face unresolved");

    // Resolve world-Z of the entry plane via bounding box.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const gp_Dir adir = in.axis_dir;
    const double overhang = 0.05;
    const double entryZ = (adir.Z() < 0) ? (zMax + overhang) : (zMin - overhang);

    const double mainR = in.bore_dia_mm / 2.0;
    const double cbR   = (in.bore_dia_mm + 2.0 * in.seal_can_OD_offset_mm) / 2.0;
    const double lipR  = (in.bore_dia_mm - 2.0 * in.lip_relief_depth_mm) / 2.0;

    const gp_Pnt origin(in.center_x_mm, in.center_y_mm, entryZ);
    const gp_Ax2 ax(origin, adir);

    // 1. MAIN BORE
    const TopoDS_Shape mainBore = pr::cylinder(
        ax, mainR, in.bore_depth_mm + overhang);

    // 2. COUNTERBORE (seal OD seat at inlet)
    const TopoDS_Shape counterbore = pr::cylinder(
        ax, cbR, in.seal_thickness_mm + overhang);

    // 3. DUST LIP RELIEF: narrower cylinder DEEPER than main bore.
    //
    // Position the lip-relief cylinder starting at bore_depth (just past the
    // main bore floor), so it's an undercut visible only beyond the main bore.
    // Computed as an offset from the entry point along axis_dir.
    const double lipStart = in.bore_depth_mm - 0.5;   // overlap 0.5 mm into bore
    const gp_Pnt lipOrigin(
        in.center_x_mm + adir.X() * lipStart,
        in.center_y_mm + adir.Y() * lipStart,
        entryZ        + adir.Z() * lipStart);
    const gp_Ax2 lipAx(lipOrigin, adir);
    const double lipDepth = 2.0;
    const TopoDS_Shape lipRelief = pr::cylinder(lipAx, lipR, lipDepth);

    // 4. NOSE CHAMFER 30° at entry — cone widening outward at entry plane.
    const double noseRise = in.nose_chamfer_mm * std::tan(30.0 * M_PI / 180.0);
    const TopoDS_Shape noseChamfer = pr::coneFrustum(
        ax,
        /*r1 bottom=*/ mainR + noseRise,
        /*r2 top   =*/ mainR,
        /*height   =*/ in.nose_chamfer_mm + overhang);

    // Combine: fuse all four cutters into one tool, then a single cut.
    const TopoDS_Shape f1   = pr::fuse(mainBore, counterbore);
    const TopoDS_Shape f2   = pr::fuse(f1, lipRelief);
    const TopoDS_Shape full = pr::fuse(f2, noseChamfer);
    const TopoDS_Shape newShape = pr::cut(wp.shape(), full);

    // Look up seal series if provided (context info for tooling).
    const SealSeriesEntry* spec = lookupSealSeries(in.seal_series);

    json params = {
        { "entry_face_id",         *entryId },
        { "center_x_mm",           in.center_x_mm },
        { "center_y_mm",           in.center_y_mm },
        { "axis_dir",              { adir.X(), adir.Y(), adir.Z() } },
        { "bore_dia_mm",           in.bore_dia_mm },
        { "bore_depth_mm",         in.bore_depth_mm },
        { "seal_thickness_mm",     in.seal_thickness_mm },
        { "seal_can_OD_offset_mm", in.seal_can_OD_offset_mm },
        { "lip_relief_depth_mm",   in.lip_relief_depth_mm },
        { "nose_chamfer_mm",       in.nose_chamfer_mm },
        { "seal_series",           in.seal_series },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           4 },
        { "spec_key",                   in.seal_series },
        { "bore_dia_mm",                in.bore_dia_mm },
        { "bore_depth_mm",              in.bore_depth_mm },
        { "counterbore_dia_mm",         2.0 * cbR },
        { "counterbore_depth_mm",       in.seal_thickness_mm },
        { "lip_relief_dia_mm",          2.0 * lipR },
        { "nose_chamfer_mm",            in.nose_chamfer_mm },
        { "nose_chamfer_angle_deg",     30 },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
        { "matches_spec",               spec ? true : false },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "boring_bar;form_tool;chamfer_mill";
    tooling.tool_dia_mm       = 2.0 * cbR;
    tooling.tool_length_mm    = in.bore_depth_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.cutting_speed_sfm = 250.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 =
        M_PI * mainR * mainR * in.bore_depth_mm +
        M_PI * (cbR * cbR - mainR * mainR) * in.seal_thickness_mm +
        M_PI * lipR * lipR * lipDepth;
    tooling.est_cycle_time_s  = std::max(5.0, in.bore_depth_mm / 5.0);
    tooling.extra = {
        { "subfeature_sequence", {
            { { "kind", "main_bore" },
              { "dia_mm", in.bore_dia_mm },
              { "depth_mm", in.bore_depth_mm } },
            { { "kind", "counterbore_seal_can" },
              { "dia_mm", 2.0 * cbR },
              { "depth_mm", in.seal_thickness_mm } },
            { { "kind", "dust_lip_relief" },
              { "dia_mm", 2.0 * lipR },
              { "depth_mm", lipDepth } },
            { { "kind", "nose_chamfer" },
              { "angle_deg", 30 },
              { "size_mm", in.nose_chamfer_mm } },
        } },
        { "standard",    "SKF CR Seals Handbook §Housing Bore + Lead-In" },
        { "seal_series", in.seal_series },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::dust_lip_seal_seat_compound: bore={}×{} cb={}×{} lip={}×{}",
                  in.bore_dia_mm, in.bore_depth_mm,
                  2.0 * cbR, in.seal_thickness_mm,
                  2.0 * lipR, lipDepth);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition: metadata replay + context-aware geometric fallback ──────

namespace {

// Context-aware: look at wp.faces() to find chained coaxial cylinders that
// form the seal-seat signature (3 different radii on one axis).

struct CylRec {
    int    faceIdx;
    double radius;
    gp_Ax1 axis;
};

std::vector<CylRec> collectCylsCtx(const Workpiece& wp)
{
    std::vector<CylRec> out;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        BRepAdaptor_Surface s(wp.face(i));
        const gp_Cylinder c = s.Cylinder();
        out.push_back(CylRec{ i, c.Radius(), c.Axis() });
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

    // Metadata replay path.
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

    // Geometric fallback: need ≥ 3 coaxial cylinders of different radii on
    // the same axis (counterbore > main bore > lip relief).
    const auto cyls = collectCylsCtx(wp);
    if (cyls.size() < 3) return out;

    for (size_t i = 0; i < cyls.size(); ++i) {
        std::vector<int> coaxialIdx{ static_cast<int>(i) };
        for (size_t j = 0; j < cyls.size(); ++j) {
            if (j == i) continue;
            if (axesColinear(cyls[i].axis, cyls[j].axis))
                coaxialIdx.push_back(static_cast<int>(j));
        }
        if (coaxialIdx.size() < 3) continue;
        // Sort by radius descending: largest = counterbore, mid = main bore,
        // smallest = lip relief.
        std::sort(coaxialIdx.begin(), coaxialIdx.end(),
                  [&](int a, int b) { return cyls[a].radius > cyls[b].radius; });
        const double cbR    = cyls[coaxialIdx[0]].radius;
        const double mainR  = cyls[coaxialIdx[1]].radius;
        const double lipR   = cyls[coaxialIdx[2]].radius;
        if (cbR <= mainR || mainR <= lipR) continue;
        // Reject if differences are noise.
        if (cbR - mainR < 0.1) continue;
        if (mainR - lipR < 0.1) continue;
        const gp_Dir adir = cyls[coaxialIdx[1]].axis.Direction();
        const gp_Pnt aOrg = cyls[coaxialIdx[1]].axis.Location();
        json recovered = {
            { "center_x_mm",         aOrg.X() },
            { "center_y_mm",         aOrg.Y() },
            { "axis_dir",            { adir.X(), adir.Y(), adir.Z() } },
            { "bore_dia_mm",         2.0 * mainR },
            { "bore_depth_mm",       5.0 },                  // unknown; default
            { "seal_thickness_mm",   7.0 },
            { "seal_can_OD_offset_mm", cbR - mainR },
            { "lip_relief_depth_mm", mainR - lipR },
            { "nose_chamfer_mm",     0.5 },
            { "seal_series",         "" },
        };
        json matched = {
            { "source",            "geometric_fallback" },
            { "counterbore_face",  cyls[coaxialIdx[0]].faceIdx },
            { "main_bore_face",    cyls[coaxialIdx[1]].faceIdx },
            { "lip_relief_face",   cyls[coaxialIdx[2]].faceIdx },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
        break;  // one match per workpiece in fallback
    }
    return out;
}

}  // namespace koocadcam::skill::dust_lip_seal_seat_compound
