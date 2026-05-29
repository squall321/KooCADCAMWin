// @lat: [[engine/skills#blend_morph_two_shapes]]

#include "blend_morph_two_shapes.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::blend_morph_two_shapes {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

TopoDS_Wire buildCircularWire(const gp_Pnt& centre, double z, double radius)
{
    const gp_Pnt c(centre.X(), centre.Y(), z);
    const gp_Ax2 ax(c, gp_Dir(0, 0, 1));
    const gp_Circ circle(ax, radius);
    BRepBuilderAPI_MakeEdge em(circle);
    if (!em.IsDone())
        throw Standard_Failure(
            "blend_morph_two_shapes: circle edge build failed");
    BRepBuilderAPI_MakeWire wm(em.Edge());
    if (!wm.IsDone())
        throw Standard_Failure(
            "blend_morph_two_shapes: circular wire build failed");
    return wm.Wire();
}

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ── validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.radius_a <= 0.0 || in.radius_b <= 0.0) {
        r.add("DFM-INPUT", "error",
              "blend_morph_two_shapes: radius_a and radius_b must be > 0");
        return r;
    }
    if (std::abs(in.z_b - in.z_a) < 1e-6) {
        r.add("DFM-INPUT", "error",
              "blend_morph_two_shapes: z_a and z_b coincide — no blend "
              "extrusion possible");
    }
    if (in.radius_a < 0.5 || in.radius_b < 0.5) {
        r.add("DFM-BLEND-MIN-D", "error",
              "blend_morph_two_shapes: radii < 0.5 mm violate "
              "min-feature DFM gate (a=" + std::to_string(in.radius_a) +
              ", b=" + std::to_string(in.radius_b) + ")");
    }
    const double dz = std::abs(in.z_b - in.z_a);
    if (dz > 0.0) {
        const double slope = std::abs(in.radius_a - in.radius_b) / dz;
        if (slope > 5.0) {
            r.add("DFM-BLEND-SLOPE", "warning",
                  "blend_morph_two_shapes: blend slope " +
                  std::to_string(slope) +
                  ":1 > 5:1 — steep transitional walls "
                  "(DIN 8580 manufacturability)");
        }
    }
    return r;
}

// ── synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "blend_morph_two_shapes DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // 1+2) Build two cross-section wires (the "section sources").
    TopoDS_Wire wireA, wireB;
    try {
        wireA = buildCircularWire(in.centre_a, in.z_a, in.radius_a);
        wireB = buildCircularWire(in.centre_b, in.z_b, in.radius_b);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("blend_morph_two_shapes: ") + ex.what());
    }

    // 3) ThruSections (G¹ tangency requested via isSolid=true, ruled=false).
    TopoDS_Shape blendSolid;
    try {
        BRepOffsetAPI_ThruSections through(true, false, 1.0e-4);
        through.AddWire(wireA);
        through.AddWire(wireB);
        through.Build();
        if (!through.IsDone())
            throw Standard_Failure(
                "blend_morph_two_shapes: ThruSections.Build failed");
        blendSolid = through.Shape();
        if (blendSolid.IsNull())
            throw Standard_Failure(
                "blend_morph_two_shapes: ThruSections produced null");
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("blend_morph_two_shapes: ") + ex.what());
    }

    // 4) Fuse onto the workpiece.
    TopoDS_Shape newShape;
    try {
        newShape = pr::fuse(wp.shape(), blendSolid);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("blend_morph_two_shapes: fuse failed: ") +
                         ex.what());
    }

    const double added = std::max(0.0, volumeOf(blendSolid));
    const double dz    = std::abs(in.z_b - in.z_a);

    json params = {
        { "centre_a", { in.centre_a.X(), in.centre_a.Y(), in.centre_a.Z() } },
        { "radius_a", in.radius_a },
        { "z_a",      in.z_a },
        { "centre_b", { in.centre_b.X(), in.centre_b.Y(), in.centre_b.Z() } },
        { "radius_b", in.radius_b },
        { "z_b",      in.z_b },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     3 },         // section_a + section_b + fuse
        { "blend_height_mm",      dz },
        { "blend_volume_mm3",     added },
        { "continuity",           "G1" },
        { "section_sources",      {
              { "a", { { "centre", { in.centre_a.X(), in.centre_a.Y(), in.z_a } },
                       { "radius", in.radius_a } } },
              { "b", { { "centre", { in.centre_b.X(), in.centre_b.Y(), in.z_b } },
                       { "radius", in.radius_b } } },
          } },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "additive_blend";
    tooling.tool_dia_mm       = 2.0 * std::max(in.radius_a, in.radius_b);
    tooling.tool_length_mm    = dz;
    tooling.tool_material     = "deposited_material";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = 0.0;
    tooling.est_cycle_time_s  = std::max(1.0, added / 500.0);
    tooling.extra = {
        { "operation_chain", {
            "section A: circle wire",
            "section B: circle wire",
            "ThruSections G1 blend",
            "Boolean fuse onto workpiece",
        } },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::blend_morph_two_shapes applied: rA={} rB={} dz={} added={:.2f}",
                  in.radius_a, in.radius_b, dz, added);

    return SkillOutput{ wpNew, sig };
}

// ── recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rp = {
            { "centre_a", f.params.value("centre_a", json::array({0,0,0})) },
            { "radius_a", f.params.value("radius_a", 0.0) },
            { "z_a",      f.params.value("z_a",      0.0) },
            { "centre_b", f.params.value("centre_b", json::array({0,0,0})) },
            { "radius_b", f.params.value("radius_b", 0.0) },
            { "z_b",      f.params.value("z_b",      0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rp, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback: look for ≥ 2 cylindrical faces sharing a vertical
    // axis but at different Z bands — that's the residual signature of a
    // ThruSections blend joining two coaxial cylinders.
    int verticalCyls = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        ++verticalCyls;
    }
    if (verticalCyls >= 2) {
        RecognizedFeature r;
        r.skill_id = kSkillId;
        r.recovered_params = json{
            { "radius_a", 0.0 },
            { "radius_b", 0.0 },
            { "z_a",      0.0 },
            { "z_b",      0.0 },
        };
        r.confidence = 0.55;
        r.matched_geometry = json{
            { "candidate_cyl_face_count", verticalCyls },
            { "note", "geometric blend guess; no history" },
        };
        out.push_back(std::move(r));
    }
    return out;
}

}  // namespace koocadcam::skill::blend_morph_two_shapes
