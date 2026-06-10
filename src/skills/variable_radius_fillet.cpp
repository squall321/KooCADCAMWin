// @lat: [[engine/skills#variable_radius_fillet]]

#include "variable_radius_fillet.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Pnt.hxx>
#include <gp_Torus.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::variable_radius_fillet {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (!(in.radius_start_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "variable_radius_fillet: radius_start_mm must be > 0 (got " +
              std::to_string(in.radius_start_mm) + ")");
    }
    if (!(in.radius_end_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "variable_radius_fillet: radius_end_mm must be > 0 (got " +
              std::to_string(in.radius_end_mm) + ")");
    }
    if (!(in.edge_z_band_min < in.edge_z_band_max)) {
        r.add("DFM-INPUT", "error",
              "variable_radius_fillet: edge_z_band_min must be < edge_z_band_max");
    }

    // Wall-thickness heuristic — use bbox min extent as wall proxy.
    if (!wp.shape().IsNull()) {
        const auto bb = pr::optimalBbox(wp.shape());
        const double minExt = std::min({ bb.dx(), bb.dy(), bb.dz() });
        const double maxR   = std::max(in.radius_start_mm, in.radius_end_mm);
        if (minExt > 0.0 && maxR > 0.5 * minExt) {
            r.add("DFM-WALL", "warning",
                  "variable_radius_fillet: max radius " +
                  std::to_string(maxR) +
                  " mm exceeds half min-extent " + std::to_string(minExt));
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Compute bounding-box mid-Z of an edge — used to select edges in a band.
bool edgeBboxMidZ(const TopoDS_Edge& edge, double& outZ)
{
    try {
        BRepAdaptor_Curve curve(edge);
        const double mid = (curve.FirstParameter() + curve.LastParameter()) / 2.0;
        outZ = curve.Value(mid).Z();
        return true;
    } catch (...) {
        return false;
    }
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "variable_radius_fillet DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Collect edges in the Z band.
    std::vector<TopoDS_Edge> selected;
    for (TopExp_Explorer e(wp.shape(), TopAbs_EDGE); e.More(); e.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(e.Current());
        double midZ = 0.0;
        if (!edgeBboxMidZ(edge, midZ)) continue;
        if (midZ < in.edge_z_band_min || midZ > in.edge_z_band_max) continue;
        selected.push_back(edge);
    }
    if (selected.empty()) {
        throw SkillError("variable_radius_fillet: no edges in Z band ["
                         + std::to_string(in.edge_z_band_min) + ", "
                         + std::to_string(in.edge_z_band_max) + "]");
    }

    // Build per-edge, isolating failures so one bad edge doesn't kill the run.
    // OCCT's BRepFilletAPI_MakeFillet supports incremental Add(r1, r2, edge);
    // Build() is the single expensive step.
    int skipped = 0;
    int filleted = 0;
    TopoDS_Shape current = wp.shape();

    for (const auto& edge : selected) {
        try {
            BRepFilletAPI_MakeFillet fb(current);
            fb.Add(in.radius_start_mm, in.radius_end_mm, edge);
            fb.Build();
            if (!fb.IsDone()) {
                ++skipped;
                continue;
            }
            const TopoDS_Shape next = fb.Shape();
            if (next.IsNull()) {
                ++skipped;
                continue;
            }
            current = next;
            ++filleted;
        } catch (const Standard_Failure& ex) {
            spdlog::debug("variable_radius_fillet: edge skipped — {}", ex.what());
            ++skipped;
        } catch (...) {
            ++skipped;
        }
    }

    if (filleted == 0) {
        throw SkillError("variable_radius_fillet: OCCT failed on every edge ("
                         + std::to_string(skipped) + " skipped)");
    }

    // Derived volume removed (negative number — material is taken away).
    double volBefore = 0.0, volAfter = 0.0;
    {
        GProp_GProps gp;
        BRepGProp::VolumeProperties(wp.shape(), gp);
        volBefore = gp.Mass();
        BRepGProp::VolumeProperties(current, gp);
        volAfter = gp.Mass();
    }
    const double derivedVolRemoved = volAfter - volBefore;  // negative

    json params = {
        { "edge_z_band_min", in.edge_z_band_min },
        { "edge_z_band_max", in.edge_z_band_max },
        { "radius_start_mm", in.radius_start_mm },
        { "radius_end_mm",   in.radius_end_mm },
    };
    json pattern = {
        { "kind",                    kSkillId },
        { "is_compound",             true },
        { "edge_count_filleted",     filleted },
        { "edge_count_skipped",      skipped },
        { "radius_start_mm",         in.radius_start_mm },
        { "radius_end_mm",           in.radius_end_mm },
        { "derived_volume_removed",  derivedVolRemoved },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "ball_mill";
    tooling.tool_dia_mm       = std::max(in.radius_start_mm, in.radius_end_mm) * 2.0;
    tooling.tool_length_mm    = std::max(5.0, tooling.tool_dia_mm * 3.0);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 300.0;
    tooling.feed_per_tooth_mm = 0.03;
    tooling.stock_removed_mm3 = std::abs(derivedVolRemoved);
    tooling.est_cycle_time_s  = std::max(2.0, filleted * 1.5);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::variable_radius_fillet: filleted={} skipped={} "
                  "rStart={} rEnd={} dVol={}",
                  filleted, skipped, in.radius_start_mm, in.radius_end_mm,
                  derivedVolRemoved);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// History replay first (high-confidence).  Geometric fallback: look for
// toroidal faces whose minor radius varies along the surface — a variable
// fillet leaves a swept toroidal patch with varying minor radius.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "edge_z_band_min", f.params.value("edge_z_band_min", 0.0) },
            { "edge_z_band_max", f.params.value("edge_z_band_max", 0.0) },
            { "radius_start_mm", f.params.value("radius_start_mm", 1.0) },
            { "radius_end_mm",   f.params.value("radius_end_mm",   1.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric scan — find toroidal faces (variable fillets always leave at
    // least one tor face per filleted edge segment).
    int torFaces = 0;
    double avgMinR = 0.0, minR = 1e30, maxR = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        BRepAdaptor_Surface s(wp.face(i));
        if (s.GetType() != GeomAbs_Torus) continue;
        const double rr = s.Torus().MinorRadius();
        if (rr <= 0.0) continue;
        ++torFaces;
        avgMinR += rr;
        minR = std::min(minR, rr);
        maxR = std::max(maxR, rr);
    }
    if (torFaces == 0) return out;
    avgMinR /= torFaces;

    RecognizedFeature rf;
    rf.skill_id = kSkillId;
    rf.recovered_params = {
        { "edge_z_band_min", 0.0 },
        { "edge_z_band_max", 0.0 },
        { "radius_start_mm", minR },
        { "radius_end_mm",   maxR },
    };
    rf.confidence = (maxR - minR) > 1e-3 ? 0.65 : 0.45;
    rf.matched_geometry = json{
        { "torus_face_count", torFaces },
        { "min_minor_radius", minR },
        { "max_minor_radius", maxR },
        { "avg_minor_radius", avgMinR },
    };
    out.push_back(rf);
    return out;
}

}  // namespace koocadcam::skill::variable_radius_fillet
