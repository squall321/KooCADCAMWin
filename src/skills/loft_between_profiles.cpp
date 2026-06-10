// @lat: [[engine/skills#loft_between_profiles]]

#include "loft_between_profiles.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
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

namespace koocadcam::skill::loft_between_profiles {

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

TopoDS_Wire buildSectionWire(const std::vector<std::pair<double,double>>& poly,
                             double z)
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
        const gp_Pnt P1(a.first, a.second, z);
        const gp_Pnt P2(b.first, b.second, z);
        if (P1.Distance(P2) < 1e-9) continue;
        BRepBuilderAPI_MakeEdge em(P1, P2);
        if (!em.IsDone()) throw Standard_Failure("section edge failed");
        wireMaker.Add(em.Edge());
    }
    if (!wireMaker.IsDone()) throw Standard_Failure("section wire failed");
    return wireMaker.Wire();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.sections.size() < 2) {
        r.add("DFM-INPUT", "error",
              "loft_between_profiles: ≥ 2 sections required (got " +
              std::to_string(in.sections.size()) + ")");
        return r;
    }
    if (in.section_z_positions.size() != in.sections.size()) {
        r.add("DFM-LOFT-Z", "error",
              "loft_between_profiles: section_z_positions size (" +
              std::to_string(in.section_z_positions.size()) +
              ") must equal sections size (" +
              std::to_string(in.sections.size()) + ")");
    }
    for (std::size_t i = 0; i < in.sections.size(); ++i) {
        if (distinctVertexCount(in.sections[i]) < 3) {
            r.add("DFM-LOFT-CLOSED", "error",
                  "loft_between_profiles: section " + std::to_string(i) +
                  " has < 3 distinct vertices");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "loft_between_profiles DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // Expected volume (trapezoidal sum):
    //   between section k and k+1 with areas Ak, Ak+1 and Δz = zk+1 − zk,
    //   contribution ≈ (Ak + Ak+1) / 2 × |Δz|.
    double expectedV = 0.0;
    std::vector<double> areas;
    areas.reserve(in.sections.size());
    for (const auto& sec : in.sections) areas.push_back(polygonArea(sec));
    double zSpan = 0.0;
    for (std::size_t k = 1; k < in.sections.size(); ++k) {
        const double dz = std::abs(in.section_z_positions[k] -
                                   in.section_z_positions[k - 1]);
        expectedV += 0.5 * (areas[k - 1] + areas[k]) * dz;
        zSpan += dz;
    }

    TopoDS_Shape loftShape;
    bool loftOk = false;
    try {
        BRepOffsetAPI_ThruSections through(in.is_solid, false, 1e-4);
        for (std::size_t k = 0; k < in.sections.size(); ++k) {
            const TopoDS_Wire w =
                buildSectionWire(in.sections[k], in.section_z_positions[k]);
            through.AddWire(w);
        }
        through.Build();
        if (through.IsDone()) {
            loftShape = through.Shape();
            loftOk    = !loftShape.IsNull();
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("loft_between_profiles: ThruSections threw — {}", ex.what());
        throw SkillError(std::string("loft_between_profiles: ") + ex.what());
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

    json params = {
        { "sections",              secs },
        { "section_z_positions",   in.section_z_positions },
        { "is_solid",              in.is_solid },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "is_compound",           true },
        { "section_count",         static_cast<int>(in.sections.size()) },
        { "path_length_mm",        zSpan },
        { "expected_volume_mm3",   expectedV },
        { "derived_volume_mm3",    addedVol },
        { "loft_done",             loftOk },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "lofted_boss";
    tooling.stock_removed_mm3 = -expectedV;
    tooling.est_cycle_time_s  = std::max(1.0, zSpan / 50.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::loft_between_profiles: secs={} dz={} added={} done={}",
                  in.sections.size(), zSpan, addedVol, loftOk);

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

    // Geometric: a loft typically yields ≥ 1 BSpline-style face (non-planar
    // non-cylindrical). Count those.
    int splineLike = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i) && !wp.isFaceCylinder(i)) ++splineLike;
    }
    if (splineLike >= 1) {
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = json::object();
        rf.confidence       = 0.45;
        rf.matched_geometry = json{ { "spline_face_count", splineLike } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::loft_between_profiles
