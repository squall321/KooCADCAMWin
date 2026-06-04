// @lat: [[engine/skills#crankshaft_journal_with_fillet]]

#include "crankshaft_journal_with_fillet.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Fillets.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::crankshaft_journal_with_fillet {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "crankshaft_journal_with_fillet: face_id out of range");
        return r;
    }
    if (!(in.journal_dia_mm > 0.0) || !(in.journal_length_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "crankshaft_journal_with_fillet: journal_dia/length must be > 0");
        return r;
    }
    if (in.fillet_radius_mm <= 0.5 || in.fillet_radius_mm > 8.0) {
        r.add("DFM-FILLET-FATIGUE", "error",
              "crankshaft_journal_with_fillet: fillet_radius (" +
              std::to_string(in.fillet_radius_mm) +
              " mm) outside SAE J fatigue-spec range (0.5, 8] mm");
    }

    // Need a referenced cylindrical (shaft) face — and the journal must shrink it.
    if (!wp.shape().IsNull()) {
        const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
        const double shaftR = bb.outerRadiusXY();
        if (in.journal_dia_mm >= 2.0 * shaftR) {
            r.add("DFM-JOURNAL-DIA", "error",
                  "crankshaft_journal_with_fillet: journal_dia (" +
                  std::to_string(in.journal_dia_mm) +
                  ") must be < shaft OD (" +
                  std::to_string(2.0 * shaftR) + ") — nothing to remove");
        }
        const double halfL = in.journal_length_mm / 2.0;
        if (in.journal_position_z_mm - halfL < bb.zMin - 1.0 ||
            in.journal_position_z_mm + halfL > bb.zMax + 1.0) {
            r.add("DFM-JOURNAL-Z", "error",
                  "crankshaft_journal_with_fillet: journal Z range outside shaft Z");
        }
    }
    if (in.fillet_radius_mm > in.journal_length_mm / 2.0) {
        r.add("DFM-FILLET-LEN", "error",
              "crankshaft_journal_with_fillet: fillet_radius > journal_length/2 — "
              "fillets would collide at the journal center");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "crankshaft_journal_with_fillet DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double shaftR    = bb.outerRadiusXY();
    const double journalR  = in.journal_dia_mm / 2.0;
    const double halfLen   = in.journal_length_mm / 2.0;
    const double startZ    = in.journal_position_z_mm - halfLen;
    const double endZ      = in.journal_position_z_mm + halfLen;
    constexpr double kOverhangR = 0.5;

    // Step cut: annular ring (journalR -> shaftR + overhang) over halfLen*2.
    const gp_Ax2 ax(gp_Pnt(0.0, 0.0, startZ), gp::DZ());
    const TopoDS_Shape stepCutter =
        pr::annularRing(ax, shaftR + kOverhangR, journalR, in.journal_length_mm);

    const double volBefore = volumeOf(wp.shape());
    const TopoDS_Shape stepped = pr::cut(wp.shape(), stepCutter);

    // Fillet at the two step edges: circular edges at Z = startZ and Z = endZ
    // whose radius equals journalR.  Use predicate-based filletEdges.
    const double zLow  = startZ;
    const double zHigh = endZ;
    const double r     = in.fillet_radius_mm;
    const double JR    = journalR;
    const double SR    = shaftR;

    auto stepEdgePredicate =
        [zLow, zHigh, JR, SR](const TopoDS_Edge& e, const gp_Pnt& mp) {
            BRepAdaptor_Curve crv(e);
            if (crv.GetType() != GeomAbs_Circle) return false;
            const gp_Circ c = crv.Circle();
            // Axis must be along Z.
            if (std::abs(std::abs(c.Axis().Direction().Z()) - 1.0) > 1e-3)
                return false;
            // Center near (0,0).
            const gp_Pnt cen = c.Location();
            if (std::hypot(cen.X(), cen.Y()) > 0.5) return false;
            // Radius must be near journalR (the step-edge circles) — not the
            // shaft OD outer circles.
            const double cr = c.Radius();
            if (std::abs(cr - JR) > 0.05) return false;
            if (std::abs(cr - SR) < 0.05) return false;  // skip outer OD
            // Z near startZ or endZ.
            const double mz = mp.Z();
            return std::abs(mz - zLow) < 0.05 || std::abs(mz - zHigh) < 0.05;
        };

    TopoDS_Shape newShape;
    int filletCount = 0;
    try {
        // Count how many edges would match (for diagnostics).
        for (TopExp_Explorer e(stepped, TopAbs_EDGE); e.More(); e.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(e.Current());
            BRepAdaptor_Curve crv(edge);
            const double mid = (crv.FirstParameter() + crv.LastParameter()) / 2.0;
            const gp_Pnt midPt = crv.Value(mid);
            if (stepEdgePredicate(edge, midPt)) ++filletCount;
        }
        newShape = pr::filletEdges(stepped, r, stepEdgePredicate);
    } catch (const Standard_Failure& ex) {
        spdlog::debug("crankshaft_journal_with_fillet: fillet failed — {}",
                      ex.what());
        // Fall back to stepped shape without fillets so the apply doesn't crash.
        newShape = stepped;
    }

    const double volAfter   = volumeOf(newShape);
    const double removedVol = volBefore - volAfter;

    json params = {
        { "face_id",                in.face_id },
        { "journal_position_z_mm",  in.journal_position_z_mm },
        { "journal_dia_mm",         in.journal_dia_mm },
        { "journal_length_mm",      in.journal_length_mm },
        { "fillet_radius_mm",       in.fillet_radius_mm },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "engine_feature_type",        "crankshaft_journal" },
        { "subfeature_count",           3 },     // 1 cut + 2 fillets
        { "journal_dia_mm",             in.journal_dia_mm },
        { "journal_length_mm",          in.journal_length_mm },
        { "journal_start_z_mm",         startZ },
        { "journal_end_z_mm",           endZ },
        { "fillet_radius_mm",           in.fillet_radius_mm },
        { "fillet_edges_filleted",      filletCount },
        { "derived_volume_removed_mm3", removedVol },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "lathe_od_insert;form_tool_fillet";
    tooling.tool_dia_mm       = in.journal_dia_mm;
    tooling.tool_length_mm    = in.journal_length_mm + 5.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 1;
    tooling.cutting_speed_sfm = 250.0;     // crankshaft 4140 steel
    tooling.feed_per_tooth_mm = 0.12;
    tooling.stock_removed_mm3 = removedVol;
    tooling.est_cycle_time_s  =
        std::max(8.0, in.journal_length_mm * 0.8 + 3.0);
    tooling.extra = {
        { "feature_standard", "SAE J crankshaft main-bearing journal" },
        { "fatigue_critical", true },
        { "fillet_per_end",   in.fillet_radius_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::crankshaft_journal_with_fillet applied: "
                  "JD={}mm L={}mm fillet={}mm filleted={}",
                  in.journal_dia_mm, in.journal_length_mm,
                  in.fillet_radius_mm, filletCount);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition — metadata replay ────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        out.push_back(RecognizedFeature{
            kSkillId, f.params, 1.0,
            { { "source",               "metadata_replay" },
              { "is_compound",          true },
              { "engine_feature_type",  "crankshaft_journal" },
              { "subfeature_count",     3 } }
        });
    }
    return out;
}

}  // namespace koocadcam::skill::crankshaft_journal_with_fillet
