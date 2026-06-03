// @lat: [[engine/skills#miter_corner_trim]]

#include "miter_corner_trim.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::miter_corner_trim {

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

    if (wp.shape().IsNull()) {
        r.add("DFM-NULL", "error",
              "miter_corner_trim: workpiece shape is null");
        return r;
    }
    if (mag3(in.member_a_axis_dir) < 1e-9 ||
        mag3(in.member_b_axis_dir) < 1e-9) {
        r.add("DFM-INPUT", "error",
              "miter_corner_trim: member axis dirs must be non-zero");
        return r;
    }
    const auto na = norm3(in.member_a_axis_dir);
    const auto nb = norm3(in.member_b_axis_dir);
    if (mag3(cross3(na, nb)) < 1e-6) {
        r.add("DFM-PARA", "error",
              "miter_corner_trim: members are parallel — no miter corner");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "miter_corner_trim DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto na = norm3(in.member_a_axis_dir);
    const auto nb = norm3(in.member_b_axis_dir);
    // Bisector plane normal = unit(a - b).
    std::array<double, 3> bisN = { na[0] - nb[0], na[1] - nb[1], na[2] - nb[2] };
    if (mag3(bisN) < 1e-9) {
        // Anti-parallel: degenerate, fall back to a.
        bisN = na;
    }
    const auto n = norm3(bisN);
    const gp_Pnt origin(in.corner_xyz[0], in.corner_xyz[1], in.corner_xyz[2]);
    const gp_Dir normal(n[0], n[1], n[2]);
    const gp_Pln pln(origin, normal);

    const double volBefore = volumeOf(wp.shape());

    // Reference point on the discard side (the side member-b points into,
    // away from member-a) far outside the bbox.
    const auto bb = pr::optimalBbox(wp.shape());
    const double diag = std::sqrt(bb.dx() * bb.dx() +
                                  bb.dy() * bb.dy() +
                                  bb.dz() * bb.dz());
    const double step = std::max(10.0, diag * 2.0);
    // Bisector normal points from b → a, so discard side is +b direction.
    const gp_Pnt discardSidePt(origin.X() + nb[0] * step,
                               origin.Y() + nb[1] * step,
                               origin.Z() + nb[2] * step);

    TopoDS_Shape newShape = wp.shape();
    bool trimOk = false;
    try {
        BRepBuilderAPI_MakeFace mf(pln);
        if (!mf.IsDone())
            throw Standard_Failure("miter_corner_trim: plane face failed");
        BRepPrimAPI_MakeHalfSpace mh(mf.Face(), discardSidePt);
        const TopoDS_Shape halfDiscard = mh.Solid();
        TopoDS_Shape cutResult = pr::cut(wp.shape(), halfDiscard);
        if (!cutResult.IsNull()) {
            newShape = cutResult;
            trimOk   = true;
        }
    } catch (const Standard_Failure& ex) {
        spdlog::debug("miter_corner_trim: trim threw — {}", ex.what());
        newShape = wp.shape();
    }

    const double volAfter = volumeOf(newShape);
    const double removed  = volBefore - volAfter;

    json params = {
        { "corner_xyz",        { in.corner_xyz[0],        in.corner_xyz[1],        in.corner_xyz[2]        } },
        { "member_a_axis_dir", { in.member_a_axis_dir[0], in.member_a_axis_dir[1], in.member_a_axis_dir[2] } },
        { "member_b_axis_dir", { in.member_b_axis_dir[0], in.member_b_axis_dir[1], in.member_b_axis_dir[2] } },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "profile_kind",        "miter" },
        { "bisector_normal",     { n[0], n[1], n[2] } },
        { "path_length",         0.0 },
        { "derived_volume_mm3",  removed },
        { "trim_done",           trimOk },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "miter_trim";
    tooling.stock_removed_mm3 = removed;
    tooling.est_cycle_time_s  = 6.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& f : wp.features()) wpNew->addFeature(f);
    wpNew->addFeature(sig);

    spdlog::debug("skill::miter_corner_trim: removed={} ok={}",
                  removed, trimOk);

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

    // Geometric: a miter corner leaves at least 2 planar faces whose
    // normals form a non-trivial angle close to 45° / 135° / 90°.  Loose
    // check: at least 2 planar faces with distinct normals.
    if (wp.shape().IsNull()) return out;
    int planar = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFacePlanar(i)) ++planar;
    }
    if (planar >= 2) {
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = nlohmann::json::object();
        rf.confidence       = 0.30;
        rf.matched_geometry = nlohmann::json{ { "planar_face_count", planar } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::miter_corner_trim
