// @lat: [[engine/skills#tapped_hole_metric_spec]]

#include "tapped_hole_metric_spec.hpp"

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::tapped_hole_metric_spec {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

// ── ISO 965-1 tap drill lookup (central thread_table) ─────────────────────

double pilotDiameterFor(const std::string& thread_size)
{
    if (const auto* s = tt::findMetric(thread_size)) return s->tap_pilot_dia_mm;
    return 0.0;
}

namespace {

double pitchFor(const std::string& thread_size)
{
    if (const auto* s = tt::findMetric(thread_size)) return s->pitch_mm;
    return 0.0;
}

std::string sizeForDiameter(double dia_mm, double tol_mm)
{
    const tt::MetricThreadSpec* best = nullptr;
    double bestErr = tol_mm;
    for (const auto& e : tt::kMetricThreads) {
        const double err = std::abs(e.tap_pilot_dia_mm - dia_mm);
        if (err < bestErr) { bestErr = err; best = &e; }
    }
    return best ? std::string(best->size_key) : std::string();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thread_size.empty()) {
        r.add("DFM-METRIC-UNKNOWN", "error",
              "tapped_hole_metric_spec: thread_size is empty");
        return r;
    }
    const double pilotDia = pilotDiameterFor(in.thread_size);
    if (pilotDia <= 0.0) {
        r.add("DFM-METRIC-UNKNOWN", "error",
              "tapped_hole_metric_spec: unknown thread_size '" + in.thread_size +
              "' (ISO 965 supports M3..M30, 11 sizes)");
        return r;
    }
    if (pilotDia < 0.8) {
        r.add("DFM-MARGIN", "error",
              "tapped_hole_metric_spec: pilot " + std::to_string(pilotDia) +
              " mm < min tap-drill 0.8 mm");
    }
    if (!in.through && in.tap_depth_mm <= 0.0) {
        r.add("DFM-TAP-DEPTH", "error",
              "tapped_hole_metric_spec: tap_depth_mm must be > 0 for blind tap");
    }
    if (in.pilot_extra_depth_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "tapped_hole_metric_spec: pilot_extra_depth_mm must be >= 0");
    }
    if (!in.through && in.tap_depth_mm > 0.0 && pilotDia > 0.0) {
        const double ratio = in.tap_depth_mm / pilotDia;
        if (ratio > 3.0) {
            r.add("DFM-TAP-PECK", "warning",
                  "tapped_hole_metric_spec: tap_depth/pilot " +
                  std::to_string(ratio) +
                  " > 3 — chip evacuation problematic, consider peck-tapping");
        }
    }
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "tapped_hole_metric_spec DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double pilotDia = pilotDiameterFor(in.thread_size);
    const double pilotR   = pilotDia / 2.0;
    const double pitch    = pitchFor(in.thread_size);

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));

    const gp_Dir adir = in.axis_dir;
    const double kOverhang = 0.05;

    gp_Pnt toolStart;
    if (std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6) {
        if (adir.Z() < 0)
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMax + kOverhang);
        else
            toolStart = gp_Pnt(in.position_x_mm, in.position_y_mm, zMin - kOverhang);
    } else {
        toolStart = gp_Pnt(
            in.position_x_mm - adir.X() * (bboxDiag + kOverhang),
            in.position_y_mm - adir.Y() * (bboxDiag + kOverhang),
            (zMin + zMax) / 2.0 - adir.Z() * (bboxDiag + kOverhang));
    }

    const gp_Ax2 toolAx(toolStart, adir);

    // CUT 1: pilot drill (through or blind+extra).
    const double pilotLen = in.through
        ? bboxDiag + 2.0 * kOverhang
        : in.tap_depth_mm + in.pilot_extra_depth_mm + kOverhang;
    const TopoDS_Shape pilotTool = pr::cylinder(toolAx, pilotR, pilotLen);

    // CUT 2: entry chamfer at 60° included angle (30° from axis) per
    // standard tap-lead practice.
    const double leg = std::max(in.chamfer_size_mm, 0.0);
    const double chamferDepth = (leg > 0.0) ? leg / std::tan(30.0 * M_PI / 180.0) : 0.0;
    const double chamferBigR  = pilotR + leg;
    TopoDS_Shape chamferTool;
    if (leg > 1e-6 && chamferDepth > 1e-6) {
        chamferTool = pr::coneFrustum(toolAx, chamferBigR, pilotR, chamferDepth);
    }

    TopoDS_Shape fusedCutter = pilotTool;
    if (!chamferTool.IsNull()) {
        fusedCutter = pr::fuse(pilotTool, chamferTool);
    }
    const TopoDS_Shape newShape = pr::cut(wp.shape(), fusedCutter);

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
        { "thread_size",          in.thread_size },
        { "tap_depth_mm",         in.tap_depth_mm },
        { "through",              in.through },
        { "chamfer_size_mm",      in.chamfer_size_mm },
        { "pilot_extra_depth_mm", in.pilot_extra_depth_mm },
        { "pilot_dia_mm",         pilotDia },
        { "pitch_mm",             pitch },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           2 },
        { "thread_size",                in.thread_size },
        { "pilot_dia_mm",               pilotDia },
        { "pitch_mm",                   pitch },
        { "tap_depth_mm",               in.tap_depth_mm },
        { "through",                    in.through },
        { "cylindrical_face_count",     1 },
        { "conical_face_count",         (leg > 0.0) ? 1 : 0 },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "drill;tap";
    tooling.tool_dia_mm      = pilotDia;
    tooling.tool_length_mm   = pilotLen + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = M_PI * pilotR * pilotR * pilotLen;
    tooling.est_cycle_time_s  = std::max(1.0, pilotLen / 30.0);
    tooling.extra = {
        { "iso_spec", "ISO 965-1" },
        { "tap_thread", in.thread_size },
        { "two_cut_sequence", {
            { { "tool_type", "drill" }, { "tool_dia_mm", pilotDia } },
            { { "tool_type", "tap"   }, { "thread_size", in.thread_size },
                                        { "pitch_mm",    pitch },
                                        { "tap_depth_mm", in.tap_depth_mm } },
        } },
        { "note", "Tap thread modelled as metadata only (matches drill_and_tap convention)" },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::tapped_hole_metric_spec applied: {} pilot {:.2f} × {:.2f}",
                  in.thread_size, pilotDia, pilotLen);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Pass 1: metadata replay.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature r;
        r.skill_id         = kSkillId;
        r.recovered_params = f.params;
        r.confidence       = 0.99;
        r.matched_geometry = { { "source", "metadata_replay" } };
        out.push_back(r);
    }
    if (!out.empty()) return out;

    // Pass 2: geometric fallback — scan cylindrical pilot faces, match dia
    // to ISO 965; if a coaxial conical chamfer face is present at the
    // entry, recover chamfer leg & confirm it's a tap-lead chamfer (60°
    // included → 30° from axis per standard tap-drill practice).  Without
    // the chamfer the candidate is still emitted but at lower confidence
    // since it overlaps with bolt_hole_metric_spec for the same pilot.
    //
    // Geometric signature:
    //   * 1 cylindrical pilot face whose radius matches ISO 965 tap-drill
    //     within 0.10 mm tolerance;
    //   * optionally 1 conical face coaxial with the pilot whose small
    //     radius matches the pilot radius (the tap entry chamfer).

    // Collect conical chamfer faces once (small radius, coaxial).
    struct ConeRec { int faceIdx; gp_Ax1 axis; double rSmall; double rLarge;
                     double semiAngleRad; };
    std::vector<ConeRec> cones;
    for (int i = 0; i < wp.faceCount(); ++i) {
        BRepAdaptor_Surface surf(wp.face(i));
        if (surf.GetType() != GeomAbs_Cone) continue;
        const gp_Cone cn = surf.Cone();
        ConeRec r;
        r.faceIdx = i; r.axis = cn.Axis();
        r.semiAngleRad = std::abs(cn.SemiAngle());
        std::vector<double> radii;
        for (TopExp_Explorer exp(wp.face(i), TopAbs_EDGE); exp.More(); exp.Next()) {
            BRepAdaptor_Curve crv(TopoDS::Edge(exp.Current()));
            if (crv.GetType() != GeomAbs_Circle) continue;
            const gp_Circ c = crv.Circle();
            if (std::abs(std::abs(c.Axis().Direction().Dot(r.axis.Direction())) - 1.0) > 1e-3)
                continue;
            radii.push_back(c.Radius());
        }
        if (radii.size() < 2) continue;
        const auto [mn, mx] = std::minmax_element(radii.begin(), radii.end());
        if (std::abs(*mx - *mn) < 0.05) continue;
        r.rSmall = *mn; r.rLarge = *mx;
        cones.push_back(r);
    }
    auto sameAxis = [](const gp_Ax1& a, const gp_Ax1& b){
        const gp_Dir da = a.Direction(), db = b.Direction();
        const double dot = std::abs(da.X()*db.X() + da.Y()*db.Y() + da.Z()*db.Z());
        if (dot < std::cos(0.5 * M_PI / 180.0)) return false;
        gp_Vec v(a.Location(), b.Location());
        gp_Vec axV(da.X(), da.Y(), da.Z());
        gp_Vec perp = v - axV * v.Dot(axV);
        return perp.Magnitude() < 0.05;
    };

    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double dia = 2.0 * cyl.Radius();

        const std::string size = sizeForDiameter(dia, 0.10);
        if (size.empty()) continue;

        // Recover position via circular edges.
        gp_Pnt entryCenter = cyl.Axis().Location();
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            entryCenter = crv.Circle().Location();
            break;
        }

        // Look for coaxial chamfer cone (tap-lead) to boost confidence.
        double chamfer_leg = 0.0;
        int    cone_face_id = -1;
        bool   chamfer_found = false;
        for (const auto& cn : cones) {
            if (!sameAxis(cyl.Axis(), cn.axis)) continue;
            if (std::abs(cn.rSmall - cyl.Radius()) > 0.2) continue;
            chamfer_leg   = std::max(0.0, cn.rLarge - cn.rSmall);
            cone_face_id  = cn.faceIdx;
            chamfer_found = true;
            break;
        }

        const gp_Dir adir = cyl.Axis().Direction();
        // Back-mapped spec table key: the ISO M-size, e.g. "M6".
        const std::string specKeyTable = size;
        // Fit-error: how close measured dia is to the canonical pilot.
        const double fitErr = std::abs(dia - pilotDiameterFor(size));
        json params = {
            { "position_x_mm",     entryCenter.X() },
            { "position_y_mm",     entryCenter.Y() },
            { "axis_dir",          { adir.X(), adir.Y(), adir.Z() } },
            { "thread_size",       size },
            { "spec_key_table",    specKeyTable },
            { "tap_depth_mm",      0.0 },
            { "through",           true },
            { "chamfer_size_mm",   chamfer_leg },
            { "pilot_dia_mm",      dia },
            { "pitch_mm",          pitchFor(size) },
            { "fit_match_err_mm",  fitErr },
        };
        // Tap lead chamfer presence boosts confidence above bolt_hole's pilot
        // signature, since the standard tap-drill flow ALWAYS adds a chamfer.
        double conf = chamfer_found ? 0.65 : 0.55;
        if (fitErr < 0.02) conf += 0.05;
        json matched = {
            { "cyl_face_id",    fIdx },
            { "iso",            "ISO 965-1" },
            { "spec_key_table", specKeyTable },
            { "source",         chamfer_found ? "geometric_fallback" : "cyl_only" },
            { "chamfer_found",  chamfer_found },
            { "fit_err_mm",     fitErr },
        };
        if (chamfer_found) matched["cone_face_id"] = cone_face_id;
        out.push_back(RecognizedFeature{ kSkillId, params, conf, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::tapped_hole_metric_spec
