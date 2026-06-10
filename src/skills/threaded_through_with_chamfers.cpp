// @lat: [[engine/skills#threaded_through_with_chamfers]]

#include "threaded_through_with_chamfers.hpp"

#include "Workpiece.hpp"
#include "_iso_thread_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::threaded_through_with_chamfers {

namespace pr = koocadcam::engine::prim;
namespace tt = koocadcam::skill::thread_table;
using nlohmann::json;

double pilotDiameterFor(const std::string& thread_size)
{
    if (const auto* s = tt::findMetric(thread_size)) return s->tap_pilot_dia_mm;
    return 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const tt::MetricThreadSpec* th = tt::findMetric(in.thread_size);
    if (!th) {
        r.add("DFM-METRIC-UNKNOWN", "error",
              "threaded_through_with_chamfers: unknown thread_size '" +
              in.thread_size + "' (ISO 965 supports M3..M30)");
        return r;
    }
    if (in.chamfer_size_mm < 0.0) {
        r.add("DFM-INPUT", "error", "chamfer_size_mm must be >= 0");
    }
    if (in.add_relief) {
        const double reliefWidth = th->pitch_mm;
        if (reliefWidth < 0.2) {
            r.add("DFM-RELIEF-WIDTH", "error",
                  "threaded_through_with_chamfers: relief_width (" +
                  std::to_string(reliefWidth) + ") < 0.2 mm — below standard "
                  "groove-tool clearance per ISO 4755");
        }
        if (in.relief_offset_mm < th->pitch_mm) {
            r.add("DFM-RELIEF-OFFSET", "warning",
                  "threaded_through_with_chamfers: relief_offset < pitch — "
                  "thread won't have room to start before the relief groove");
        }
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double thickness =
        std::abs(in.axis_dir.X()) * (xMax - xMin) +
        std::abs(in.axis_dir.Y()) * (yMax - yMin) +
        std::abs(in.axis_dir.Z()) * (zMax - zMin);
    if (thickness > 0.0 && thickness < 1.5 * th->tap_pilot_dia_mm) {
        r.add("DFM-THROUGH-LEN", "warning",
              "threaded_through_with_chamfers: stock_thickness " +
              std::to_string(thickness) + " < 1.5 × pilot_dia " +
              std::to_string(1.5 * th->tap_pilot_dia_mm) +
              " — insufficient thread engagement");
    }
    return r;
}

// ── Synthesis: 4-cut compound (pilot + entry chamfer + exit chamfer + relief)

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "threaded_through_with_chamfers DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const tt::MetricThreadSpec* th = tt::findMetric(in.thread_size);
    const double pilotDia = th->tap_pilot_dia_mm;
    const double pilotR   = pilotDia / 2.0;
    const double pitch    = th->pitch_mm;
    const double reliefDia = pilotDia + 2.0 * pitch;   // spec rule
    const double reliefR   = reliefDia / 2.0;
    const double reliefW   = pitch;                    // spec rule

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double bboxDiag = std::sqrt(
        (xMax - xMin) * (xMax - xMin) +
        (yMax - yMin) * (yMax - yMin) +
        (zMax - zMin) * (zMax - zMin));
    const double thicknessAlongAxis =
        std::abs(in.axis_dir.X()) * (xMax - xMin) +
        std::abs(in.axis_dir.Y()) * (yMax - yMin) +
        std::abs(in.axis_dir.Z()) * (zMax - zMin);

    const gp_Dir adir = in.axis_dir;
    const double kOverhang = 0.05;

    // Tool start (entry face plane).
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
    const double throughLen = bboxDiag + 2.0 * kOverhang;

    // CUT 1: pilot through cylinder.
    const TopoDS_Shape pilotTool = pr::cylinder(toolAx, pilotR, throughLen);

    // CUT 2: entry chamfer (60° included, so 30° wall slope from axis).
    const double leg = std::max(in.chamfer_size_mm, 0.0);
    const double chamferDepth = (leg > 0.0) ? leg / std::tan(30.0 * M_PI / 180.0) : 0.0;
    const double chamferBigR  = pilotR + leg;
    TopoDS_Shape entryChamfer;
    if (leg > 1e-6 && chamferDepth > 1e-6) {
        entryChamfer = pr::coneFrustum(toolAx, chamferBigR, pilotR, chamferDepth);
    }

    // CUT 3: exit chamfer — same cone but flipped on the far side.
    TopoDS_Shape exitChamfer;
    if (leg > 1e-6 && chamferDepth > 1e-6 && thicknessAlongAxis > chamferDepth) {
        // Exit face is at toolStart + adir * (kOverhang + thickness).
        // The exit chamfer cone has its "big" radius at the OUTSIDE face,
        // and slopes back inward.  We synthesize by reversing the axis.
        gp_Pnt exitStart(
            toolStart.X() + adir.X() * (kOverhang + thicknessAlongAxis + chamferDepth),
            toolStart.Y() + adir.Y() * (kOverhang + thicknessAlongAxis + chamferDepth),
            toolStart.Z() + adir.Z() * (kOverhang + thicknessAlongAxis + chamferDepth));
        const gp_Dir exitDir(-adir.X(), -adir.Y(), -adir.Z());
        const gp_Ax2 exitAx(exitStart, exitDir);
        exitChamfer = pr::coneFrustum(exitAx, chamferBigR, pilotR, chamferDepth);
    }

    // CUT 4: thread relief — short annular cylinder, slightly larger
    // diameter, positioned `relief_offset_mm` below the entry face.
    TopoDS_Shape reliefTool;
    if (in.add_relief && reliefW >= 0.2) {
        gp_Pnt reliefStart(
            toolStart.X() + adir.X() * (kOverhang + in.relief_offset_mm),
            toolStart.Y() + adir.Y() * (kOverhang + in.relief_offset_mm),
            toolStart.Z() + adir.Z() * (kOverhang + in.relief_offset_mm));
        const gp_Ax2 reliefAx(reliefStart, adir);
        reliefTool = pr::cylinder(reliefAx, reliefR, reliefW);
    }

    // Fuse all concentric cuts into single cutter.
    TopoDS_Shape fused = pilotTool;
    if (!entryChamfer.IsNull()) fused = pr::fuse(fused, entryChamfer);
    if (!exitChamfer.IsNull())  fused = pr::fuse(fused, exitChamfer);
    if (!reliefTool.IsNull())   fused = pr::fuse(fused, reliefTool);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), fused);

    const int subCount = 1 + (!entryChamfer.IsNull() ? 1 : 0)
                            + (!exitChamfer.IsNull()  ? 1 : 0)
                            + (!reliefTool.IsNull()   ? 1 : 0);

    json params = {
        { "position_x_mm",    in.position_x_mm },
        { "position_y_mm",    in.position_y_mm },
        { "axis_dir",         { adir.X(), adir.Y(), adir.Z() } },
        { "thread_size",      in.thread_size },
        { "chamfer_size_mm",  in.chamfer_size_mm },
        { "add_relief",       in.add_relief },
        { "relief_offset_mm", in.relief_offset_mm },
        { "pilot_dia_mm",     pilotDia },
        { "pitch_mm",         pitch },
        { "relief_dia_mm",    reliefDia },
        { "relief_width_mm",  reliefW },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "subfeature_count",           subCount },
        { "thread_size",                in.thread_size },
        { "pilot_dia_mm",               pilotDia },
        { "pitch_mm",                   pitch },
        { "relief_dia_mm",              reliefDia },
        { "relief_width_mm",            reliefW },
        { "cylindrical_face_count",     in.add_relief ? 2 : 1 },
        { "conical_face_count",         (leg > 0.0) ? 2 : 0 },
        { "axis_dir",                   { adir.X(), adir.Y(), adir.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "drill;tap;chamfer;groove";
    tooling.tool_dia_mm      = pilotDia;
    tooling.tool_length_mm   = throughLen + 5.0;
    tooling.tool_material    = "carbide";
    tooling.flute_count      = 2;
    tooling.cutting_speed_sfm = 200.0;
    tooling.feed_per_tooth_mm = 0.04;
    tooling.stock_removed_mm3 = M_PI * pilotR * pilotR * throughLen;
    tooling.est_cycle_time_s  = std::max(2.0, throughLen / 40.0);
    tooling.extra = {
        { "iso_spec", "ISO 965-1 + ISO 13715" },
        { "tap_thread", in.thread_size },
        { "four_cut_sequence", {
            { { "tool_type", "drill" },        { "tool_dia_mm",   pilotDia    } },
            { { "tool_type", "chamfer_mill" }, { "leg_mm",        leg         } },
            { { "tool_type", "tap" },          { "thread_size",   in.thread_size } },
            { { "tool_type", "groove_tool" },  { "relief_dia_mm", reliefDia   },
                                                { "width_mm",      reliefW     } },
        } },
    };

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::threaded_through_with_chamfers applied: {} ({} cuts)",
                  in.thread_size, subCount);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

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

    // Heuristic — pairs of coaxial cylinders where the larger is a thin
    // ring (the relief groove) point to this skill.  Single matching
    // cylinder also nominated at low confidence.
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (!wp.isFaceCylinder(fIdx)) continue;
        const TopoDS_Face& cylFace = wp.face(fIdx);
        BRepAdaptor_Surface surf(cylFace);
        const gp_Cylinder cyl = surf.Cylinder();
        const double dia = 2.0 * cyl.Radius();

        // Match against ISO 965 pilot diameters.
        const tt::MetricThreadSpec* best = nullptr;
        double bestErr = 0.10;
        for (const auto& e : tt::kMetricThreads) {
            const double err = std::abs(e.tap_pilot_dia_mm - dia);
            if (err < bestErr) { bestErr = err; best = &e; }
        }
        if (!best) continue;

        gp_Pnt entryCenter = cyl.Axis().Location();
        for (TopExp_Explorer exp(cylFace, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& e = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) continue;
            entryCenter = crv.Circle().Location();
            break;
        }

        const gp_Dir adir = cyl.Axis().Direction();
        json params = {
            { "position_x_mm", entryCenter.X() },
            { "position_y_mm", entryCenter.Y() },
            { "axis_dir",      { adir.X(), adir.Y(), adir.Z() } },
            { "thread_size",   best->size_key },
            { "pilot_dia_mm",  best->tap_pilot_dia_mm },
            { "pitch_mm",      best->pitch_mm },
            { "add_relief",    false },     // can't see relief from single cyl
        };
        out.push_back(RecognizedFeature{
            kSkillId, params, 0.50,
            { { "cyl_face_id", fIdx }, { "iso", "ISO 965-1" } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::threaded_through_with_chamfers
