// @lat: [[engine/skills#weldment_gusset_triangular]]

#include "weldment_gusset_triangular.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::weldment_gusset_triangular {

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

double mag3(const std::array<double, 3>& v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

std::array<double, 3> norm3(const std::array<double, 3>& v)
{
    const double m = mag3(v);
    if (m < 1e-12) return { 0.0, 0.0, 0.0 };
    return { v[0] / m, v[1] / m, v[2] / m };
}

std::array<double, 3> cross3(const std::array<double, 3>& a,
                             const std::array<double, 3>& b)
{
    return { a[1] * b[2] - a[2] * b[1],
             a[2] * b[0] - a[0] * b[2],
             a[0] * b[1] - a[1] * b[0] };
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.gusset_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "weldment_gusset_triangular: gusset_thickness_mm must be > 0");
    }
    if (in.gusset_leg_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "weldment_gusset_triangular: gusset_leg_length_mm must be > 0");
    }
    if (mag3(in.leg_a_xyz) < 1e-9 || mag3(in.leg_b_xyz) < 1e-9) {
        r.add("DFM-INPUT", "error",
              "weldment_gusset_triangular: leg vectors must be non-zero");
    } else {
        const auto na = norm3(in.leg_a_xyz);
        const auto nb = norm3(in.leg_b_xyz);
        const auto c  = cross3(na, nb);
        if (mag3(c) < 1e-6) {
            r.add("DFM-GEOM", "error",
                  "weldment_gusset_triangular: leg_a and leg_b are collinear "
                  "— cannot form a triangle");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "weldment_gusset_triangular DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto vA = norm3(in.leg_a_xyz);
    const auto vB = norm3(in.leg_b_xyz);
    const double L = in.gusset_leg_length_mm;

    const gp_Pnt V0(in.vertex_xyz[0], in.vertex_xyz[1], in.vertex_xyz[2]);
    const gp_Pnt V1(V0.X() + vA[0] * L, V0.Y() + vA[1] * L, V0.Z() + vA[2] * L);
    const gp_Pnt V2(V0.X() + vB[0] * L, V0.Y() + vB[1] * L, V0.Z() + vB[2] * L);

    // Build triangular face.
    TopoDS_Face triFace;
    gp_Vec normalVec;
    try {
        BRepBuilderAPI_MakePolygon poly(V0, V1, V2, /*Close=*/true);
        if (!poly.IsDone()) throw Standard_Failure("gusset polygon");
        BRepBuilderAPI_MakeFace fm(poly.Wire(), true);
        if (!fm.IsDone()) throw Standard_Failure("gusset face");
        triFace = fm.Face();

        // Compute the triangle plane normal for the extrusion direction.
        const std::array<double, 3> e1 = { V1.X() - V0.X(),
                                           V1.Y() - V0.Y(),
                                           V1.Z() - V0.Z() };
        const std::array<double, 3> e2 = { V2.X() - V0.X(),
                                           V2.Y() - V0.Y(),
                                           V2.Z() - V0.Z() };
        const auto n = norm3(cross3(e1, e2));
        normalVec = gp_Vec(n[0], n[1], n[2]);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("weldment_gusset_triangular: ") + ex.what());
    }

    // Extrude the triangle perpendicular to its plane by gusset_thickness.
    // Center the extrusion (half above + half below) for symmetry.
    const gp_Vec halfDown = normalVec.Multiplied(-in.gusset_thickness_mm * 0.5);
    const gp_Pnt V0b = V0.Translated(halfDown);
    const gp_Pnt V1b = V1.Translated(halfDown);
    const gp_Pnt V2b = V2.Translated(halfDown);

    TopoDS_Shape gussetSolid;
    try {
        BRepBuilderAPI_MakePolygon polyB(V0b, V1b, V2b, /*Close=*/true);
        if (!polyB.IsDone()) throw Standard_Failure("gusset base poly");
        BRepBuilderAPI_MakeFace fmB(polyB.Wire(), true);
        if (!fmB.IsDone()) throw Standard_Failure("gusset base face");

        BRepPrimAPI_MakePrism prism(fmB.Face(),
                                    normalVec.Multiplied(in.gusset_thickness_mm));
        prism.Build();
        if (!prism.IsDone()) throw Standard_Failure("gusset prism");
        gussetSolid = prism.Shape();
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("weldment_gusset_triangular: ") + ex.what());
    }

    // Fuse onto workpiece.
    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape;
    if (wp.shape().IsNull()) {
        newShape = gussetSolid;
    } else {
        try {
            newShape = pr::fuse(wp.shape(), gussetSolid);
        } catch (const Standard_Failure&) {
            BRep_Builder builder;
            TopoDS_Compound comp;
            builder.MakeCompound(comp);
            builder.Add(comp, wp.shape());
            builder.Add(comp, gussetSolid);
            newShape = comp;
        }
    }

    // Triangle area = 0.5 |a × b| ·L²  (when a, b are unit, scaled by L on each).
    const auto e1 = std::array<double, 3>{ V1.X() - V0.X(), V1.Y() - V0.Y(), V1.Z() - V0.Z() };
    const auto e2 = std::array<double, 3>{ V2.X() - V0.X(), V2.Y() - V0.Y(), V2.Z() - V0.Z() };
    const double triArea  = 0.5 * mag3(cross3(e1, e2));
    const double expected = triArea * in.gusset_thickness_mm;
    const double volAfter = volumeOf(newShape);
    const double added    = volAfter - volBefore;

    json params = {
        { "vertex_xyz",          { in.vertex_xyz[0], in.vertex_xyz[1], in.vertex_xyz[2] } },
        { "leg_a_xyz",           { in.leg_a_xyz[0],  in.leg_a_xyz[1],  in.leg_a_xyz[2]  } },
        { "leg_b_xyz",           { in.leg_b_xyz[0],  in.leg_b_xyz[1],  in.leg_b_xyz[2]  } },
        { "gusset_thickness_mm", in.gusset_thickness_mm },
        { "gusset_leg_length_mm",in.gusset_leg_length_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "profile_kind",        "triangle" },
        { "profile_dim_mm",      in.gusset_leg_length_mm },
        { "thickness_mm",        in.gusset_thickness_mm  },
        { "triangle_area_mm2",   triArea },
        { "path_length",         in.gusset_thickness_mm  },
        { "derived_volume_mm3",  added },
        { "expected_volume_mm3", expected },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "weldment_gusset";
    tooling.stock_removed_mm3 = -expected;
    tooling.est_cycle_time_s  = 12.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& f : wp.features()) wpNew->addFeature(f);
    wpNew->addFeature(sig);

    spdlog::debug("skill::weldment_gusset_triangular: L={} t={} added={}",
                  in.gusset_leg_length_mm, in.gusset_thickness_mm, added);

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

    // Geometric: triangular thin plate = 5 planar faces, two of which are
    // triangular (3 edges).  Coarse check: at least 5 planar faces total.
    int planar = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFacePlanar(i)) ++planar;
    }
    if (planar >= 5) {
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = nlohmann::json::object();
        rf.confidence       = 0.35;
        rf.matched_geometry = nlohmann::json{ { "planar_face_count", planar } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::weldment_gusset_triangular
