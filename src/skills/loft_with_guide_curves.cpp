// @lat: [[engine/skills#loft_with_guide_curves]]

#include "loft_with_guide_curves.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::loft_with_guide_curves {

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

double polygonArea(const std::vector<std::pair<double,double>>& poly)
{
    if (poly.size() < 3) return 0.0;
    double a = 0.0;
    const std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; ++i) {
        const auto& [x1, y1] = poly[i];
        const auto& [x2, y2] = poly[(i + 1) % n];
        a += x1 * y2 - x2 * y1;
    }
    return std::abs(a) * 0.5;
}

double pathLength(const std::vector<gp_Pnt>& pts)
{
    double L = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        L += pts[i].Distance(pts[i - 1]);
    }
    return L;
}

int distinctVertexCount(const std::vector<std::pair<double,double>>& poly)
{
    if (poly.empty()) return 0;
    int n = 1;
    for (std::size_t i = 1; i < poly.size(); ++i) {
        const double dx = poly[i].first  - poly[i - 1].first;
        const double dy = poly[i].second - poly[i - 1].second;
        if (std::sqrt(dx*dx + dy*dy) > 1e-6) ++n;
    }
    const double dxc = poly.front().first  - poly.back().first;
    const double dyc = poly.front().second - poly.back().second;
    if (n >= 2 && std::sqrt(dxc*dxc + dyc*dyc) < 1e-6) --n;
    return n;
}

TopoDS_Wire buildSectionWireAt(const std::vector<std::pair<double,double>>& poly,
                               const gp_Pnt& origin)
{
    std::size_t end = poly.size();
    if (end >= 2 &&
        std::hypot(poly.front().first  - poly.back().first,
                   poly.front().second - poly.back().second) < 1e-6) {
        --end;
    }
    if (end < 3) throw Standard_Failure("section < 3 distinct vertices");

    BRepBuilderAPI_MakeWire wireMaker;
    for (std::size_t i = 0; i < end; ++i) {
        const auto& a = poly[i];
        const auto& b = poly[(i + 1) % end];
        const gp_Pnt P1(origin.X() + a.first, origin.Y() + a.second, origin.Z());
        const gp_Pnt P2(origin.X() + b.first, origin.Y() + b.second, origin.Z());
        if (P1.Distance(P2) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(P1, P2);
        if (!em.IsDone()) throw Standard_Failure("section edge failed");
        wireMaker.Add(em.Edge());
    }
    if (!wireMaker.IsDone()) throw Standard_Failure("section wire failed");
    return wireMaker.Wire();
}

TopoDS_Wire makeSpineWire(const std::vector<gp_Pnt>& pts)
{
    BRepBuilderAPI_MakeWire wireMaker;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        if (pts[i].Distance(pts[i - 1]) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(pts[i - 1], pts[i]);
        if (!em.IsDone()) throw Standard_Failure("spine edge failed");
        wireMaker.Add(em.Edge());
    }
    if (!wireMaker.IsDone()) throw Standard_Failure("spine wire failed");
    return wireMaker.Wire();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.sections.size() != 2) {
        r.add("DFM-INPUT", "error",
              "loft_with_guide_curves: exactly 2 sections required (got " +
              std::to_string(in.sections.size()) + ")");
        return r;
    }
    for (std::size_t i = 0; i < in.sections.size(); ++i) {
        if (distinctVertexCount(in.sections[i]) < 3) {
            r.add("DFM-LOFT-CLOSED", "error",
                  "loft_with_guide_curves: section " + std::to_string(i) +
                  " has < 3 distinct vertices");
        }
    }
    if (in.guide_curve.size() < 2) {
        r.add("DFM-LOFT-GUIDE", "error",
              "loft_with_guide_curves: guide_curve needs ≥ 2 points (got " +
              std::to_string(in.guide_curve.size()) + ")");
    } else if (pathLength(in.guide_curve) <= 1e-9) {
        r.add("DFM-LOFT-GUIDE", "error",
              "loft_with_guide_curves: guide_curve length must be > 0");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "loft_with_guide_curves DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto& guide = in.guide_curve;
    const double L    = pathLength(guide);
    const double A0   = polygonArea(in.sections[0]);
    const double A1   = polygonArea(in.sections[1]);
    const double expectedV = 0.5 * (A0 + A1) * L;

    TopoDS_Shape loftShape;
    bool loftOk = false;
    // Strategy: try MakePipeShell with 2 section wires placed at guide
    // endpoints + spine wire; if that fails, fall back to a simple
    // ThruSections of the same 2 wires.
    try {
        const TopoDS_Wire w0 = buildSectionWireAt(in.sections[0], guide.front());
        const TopoDS_Wire w1 = buildSectionWireAt(in.sections[1], guide.back());
        const TopoDS_Wire spine = makeSpineWire(guide);

        BRepOffsetAPI_MakePipeShell shellMaker(spine);
        shellMaker.Add(w0);
        shellMaker.Add(w1);
        shellMaker.Build();
        if (shellMaker.IsDone()) {
            shellMaker.MakeSolid();
            loftShape = shellMaker.Shape();
            loftOk    = !loftShape.IsNull();
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("loft_with_guide_curves: MakePipeShell threw — {}", ex.what());
    }

    if (!loftOk) {
        // Fallback: ThruSections of the same 2 wires (no guide constraint).
        try {
            const TopoDS_Wire w0 = buildSectionWireAt(in.sections[0], guide.front());
            const TopoDS_Wire w1 = buildSectionWireAt(in.sections[1], guide.back());
            BRepOffsetAPI_ThruSections ts(true, false, 1e-4);
            ts.AddWire(w0);
            ts.AddWire(w1);
            ts.Build();
            if (ts.IsDone()) {
                loftShape = ts.Shape();
                loftOk    = !loftShape.IsNull();
            }
        } catch (const Standard_Failure& ex) {
            spdlog::debug("loft_with_guide_curves: fallback ThruSections threw — {}", ex.what());
            throw SkillError(std::string("loft_with_guide_curves: ") + ex.what());
        }
    }

    const double volBefore = volumeOf(wp.shape());

    TopoDS_Shape newShape;
    if (loftOk) {
        try {
            newShape = pr::fuse(wp.shape(), loftShape);
        } catch (const Standard_Failure&) {
            BRep_Builder builder;
            TopoDS_Compound comp;
            builder.MakeCompound(comp);
            if (!wp.shape().IsNull()) builder.Add(comp, wp.shape());
            builder.Add(comp, loftShape);
            newShape = comp;
        }
    } else {
        newShape = wp.shape();
    }

    const double volAfter = volumeOf(newShape);
    const double addedVol = volAfter - volBefore;

    json secs = json::array();
    for (const auto& sec : in.sections) {
        json s = json::array();
        for (const auto& xy : sec) s.push_back({ xy.first, xy.second });
        secs.push_back(s);
    }
    json guideJ = json::array();
    for (const auto& p : guide) guideJ.push_back({ p.X(), p.Y(), p.Z() });

    json params = {
        { "sections",      secs },
        { "guide_curve",   guideJ },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "section_count",         2 },
        { "path_length_mm",        L },
        { "expected_volume_mm3",   expectedV },
        { "derived_volume_mm3",    addedVol },
        { "loft_done",             loftOk },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "lofted_boss_with_guide";
    tooling.stock_removed_mm3 = -expectedV;
    tooling.est_cycle_time_s  = std::max(1.0, L / 40.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::loft_with_guide_curves: L={} A0={} A1={} added={} done={}",
                  L, A0, A1, addedVol, loftOk);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.95;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    int splineLike = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i) && !wp.isFaceCylinder(i)) ++splineLike;
    }
    if (splineLike >= 1) {
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = json::object();
        rf.confidence       = 0.4;
        rf.matched_geometry = json{ { "spline_face_count", splineLike } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::loft_with_guide_curves
