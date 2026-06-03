// @lat: [[engine/skills#weldment_end_cap]]

#include "weldment_end_cap.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Circle.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::weldment_end_cap {

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

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.cap_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "weldment_end_cap: cap_thickness_mm must be > 0");
    }
    if (in.profile_dim_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "weldment_end_cap: profile_dim_mm must be > 0");
    }
    if (mag3(in.tube_axis_dir) < 1e-9) {
        r.add("DFM-INPUT", "error",
              "weldment_end_cap: tube_axis_dir is zero vector");
    }
    if (in.profile_kind != "square" && in.profile_kind != "round") {
        r.add("DFM-KIND", "error",
              "weldment_end_cap: profile_kind must be \"square\" or \"round\" (got \"" +
              in.profile_kind + "\")");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "weldment_end_cap DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt origin(in.tube_axis_origin[0], in.tube_axis_origin[1],
                        in.tube_axis_origin[2]);
    const gp_Dir axisDir(in.tube_axis_dir[0], in.tube_axis_dir[1],
                         in.tube_axis_dir[2]);

    // 1) Build profile face perpendicular to axis_dir at origin.
    const gp_Ax3 ax(origin, axisDir);
    const gp_Pln plane(ax);

    TopoDS_Face profileFace;
    try {
        if (in.profile_kind == "round") {
            const gp_Ax2 axisC(origin, axisDir);
            Handle(Geom_Circle) c = new Geom_Circle(axisC, in.profile_dim_mm * 0.5);
            BRepBuilderAPI_MakeEdge em(c);
            if (!em.IsDone()) throw Standard_Failure("end_cap round edge");
            BRepBuilderAPI_MakeWire wm(em.Edge());
            if (!wm.IsDone()) throw Standard_Failure("end_cap round wire");
            BRepBuilderAPI_MakeFace fm(plane, wm.Wire(), true);
            if (!fm.IsDone()) throw Standard_Failure("end_cap round face");
            profileFace = fm.Face();
        } else {
            // square
            const double h = in.profile_dim_mm * 0.5;
            std::vector<std::pair<double, double>> sq = {
                { -h, -h }, { h, -h }, { h, h }, { -h, h }
            };
            const gp_Vec ux(ax.XDirection());
            const gp_Vec uy(ax.YDirection());
            BRepBuilderAPI_MakeWire wm;
            const std::size_t n = sq.size();
            for (std::size_t i = 0; i < n; ++i) {
                const auto& a = sq[i];
                const auto& b = sq[(i + 1) % n];
                const gp_Pnt P1 = origin.Translated(ux * a.first + uy * a.second);
                const gp_Pnt P2 = origin.Translated(ux * b.first + uy * b.second);
                BRepBuilderAPI_MakeEdge em(P1, P2);
                if (!em.IsDone()) throw Standard_Failure("end_cap sq edge");
                wm.Add(em.Edge());
            }
            if (!wm.IsDone()) throw Standard_Failure("end_cap sq wire");
            BRepBuilderAPI_MakeFace fm(plane, wm.Wire(), true);
            if (!fm.IsDone()) throw Standard_Failure("end_cap sq face");
            profileFace = fm.Face();
        }
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("weldment_end_cap: ") + ex.what());
    }

    // 2) Extrude profile along axis_dir by cap_thickness.
    const gp_Vec extrudeVec(axisDir);
    TopoDS_Shape capShape;
    try {
        BRepPrimAPI_MakePrism prism(profileFace,
                                    extrudeVec.Multiplied(in.cap_thickness_mm));
        prism.Build();
        if (!prism.IsDone()) throw Standard_Failure("end_cap prism");
        capShape = prism.Shape();
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("weldment_end_cap: ") + ex.what());
    }

    // 3) Fuse onto workpiece.
    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape;
    if (wp.shape().IsNull()) {
        newShape = capShape;
    } else {
        try {
            newShape = pr::fuse(wp.shape(), capShape);
        } catch (const Standard_Failure&) {
            BRep_Builder builder;
            TopoDS_Compound comp;
            builder.MakeCompound(comp);
            builder.Add(comp, wp.shape());
            builder.Add(comp, capShape);
            newShape = comp;
        }
    }
    const double volAfter = volumeOf(newShape);
    const double added    = volAfter - volBefore;

    const double D = in.profile_dim_mm;
    const double area = (in.profile_kind == "round")
                            ? M_PI * D * D * 0.25
                            : D * D;
    const double expected = area * in.cap_thickness_mm;

    json params = {
        { "tube_axis_origin", { in.tube_axis_origin[0],
                                in.tube_axis_origin[1],
                                in.tube_axis_origin[2] } },
        { "tube_axis_dir",    { in.tube_axis_dir[0],
                                in.tube_axis_dir[1],
                                in.tube_axis_dir[2] } },
        { "cap_thickness_mm", in.cap_thickness_mm },
        { "profile_kind",     in.profile_kind     },
        { "profile_dim_mm",   in.profile_dim_mm   },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "profile_kind",        in.profile_kind },
        { "profile_dim_mm",      in.profile_dim_mm },
        { "cap_thickness_mm",    in.cap_thickness_mm },
        { "path_length",         in.cap_thickness_mm },
        { "derived_volume_mm3",  added },
        { "expected_volume_mm3", expected },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "weldment_end_cap";
    tooling.stock_removed_mm3 = -expected;
    tooling.est_cycle_time_s  = 8.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& f : wp.features()) wpNew->addFeature(f);
    wpNew->addFeature(sig);

    spdlog::debug("skill::weldment_end_cap: kind={} D={} t={} added={}",
                  in.profile_kind, in.profile_dim_mm, in.cap_thickness_mm, added);

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

    // Geometric: end-cap planar face at extreme of axis — count planars.
    int planarCount = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFacePlanar(i)) ++planarCount;
    }
    if (planarCount >= 2) {
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = nlohmann::json::object();
        rf.confidence       = 0.35;
        rf.matched_geometry = nlohmann::json{ { "planar_face_count", planarCount } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::weldment_end_cap
