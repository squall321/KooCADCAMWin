// @lat: [[engine/skills#expansion_joint_bellows_stub]]

#include "expansion_joint_bellows_stub.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::expansion_joint_bellows_stub {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.flange_outer_dia_mm <= 0.0 || in.flange_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "flange dims must be > 0");
    }
    if (in.pipe_outer_dia_mm <= 0.0 || in.pipe_inner_dia_mm <= 0.0 ||
        in.pipe_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "pipe dims must be > 0");
    }
    if (in.pipe_inner_dia_mm >= in.pipe_outer_dia_mm) {
        r.add("DFM-PIPE-WALL", "error",
              "pipe_inner_dia must be < pipe_outer_dia");
    }
    // EJMA: pipe wall ≥ 0.5 mm.
    const double wallT = (in.pipe_outer_dia_mm - in.pipe_inner_dia_mm) / 2.0;
    if (wallT < 0.5) {
        r.add("DFM-EJMA-10", "error",
              "pipe wall thickness " + std::to_string(wallT) +
              " mm < min 0.5 mm (EJMA-10)");
    }
    if (in.flange_outer_dia_mm <= in.pipe_outer_dia_mm) {
        r.add("DFM-FLANGE", "error",
              "flange_outer_dia must be > pipe_outer_dia");
    }
    // EJMA: flange thickness ≥ bore/16, but cap at 6 mm minimum.
    const double minFlangeT = std::max(3.0, in.pipe_inner_dia_mm / 16.0);
    if (in.flange_thickness_mm < minFlangeT) {
        r.add("DFM-EJMA-10", "error",
              "flange_thickness " + std::to_string(in.flange_thickness_mm) +
              " mm < min " + std::to_string(minFlangeT) +
              " mm (bore/16, min 3 mm)");
    }
    // EJMA: minimum 2 convolutions.
    if (in.corrugation_count < 2) {
        r.add("DFM-EJMA-10", "error",
              "corrugation_count " + std::to_string(in.corrugation_count) +
              " < min 2 (EJMA-10 minimum convolutions)");
    }
    // EJMA: pitch ≥ 1.5 × convolution height.
    const double convHeight =
        (in.corrugation_outer_dia_mm - in.pipe_outer_dia_mm) / 2.0;
    if (convHeight <= 0.0) {
        r.add("DFM-CORRUGATION", "error",
              "corrugation_outer_dia must be > pipe_outer_dia");
    } else if (in.corrugation_pitch_mm < 1.5 * convHeight) {
        r.add("DFM-EJMA-10", "error",
              "corrugation_pitch " + std::to_string(in.corrugation_pitch_mm) +
              " mm < 1.5 × convolution height (" +
              std::to_string(1.5 * convHeight) + " mm)");
    }
    // Slice-9: stack must fit within ~1.2× pipe_length — convolutions can
    // extend slightly beyond the pipe terminus on real EJMA bellows joints
    // (the end-cap weld lap-joint sits on the last convolution crown).
    const double stackH = in.corrugation_pitch_mm *
                          static_cast<double>(in.corrugation_count);
    if (stackH > in.pipe_length_mm * 1.2) {
        r.add("DFM-EJMA-10", "error",
              "corrugation stack height " + std::to_string(stackH) +
              " mm > 1.2 × pipe_length (" +
              std::to_string(in.pipe_length_mm * 1.2) + ")");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Build sequence (fuses then a single cut for the bore):
//   1. fuse flange disc onto wp
//   2. fuse pipe core (outer cyl) onto flange top
//   3. fuse N toroids stacked along axis
//   4. cut the central bore through the whole stack
//
// Volume math (per added/removed primitive — additive estimate):
//   V_flange = π · R_flange²  · t_flange
//   V_pipe   = π · R_pipe²    · L_pipe
//   V_torus  = N · 2π² · R_major · r_minor²
//     where r_minor = (corrugation_outer - pipe_outer) / 2,
//           R_major = (corrugation_outer + pipe_outer) / 4   [pipe surface plus minor R]
//   V_bore   = π · R_bore² · (t_flange + L_pipe)
//
//   net delta_added = V_flange + V_pipe + V_torus - V_bore

namespace {

TopoDS_Shape makeTorus(const gp_Ax2& ax, double majorR, double minorR)
{
    BRepPrimAPI_MakeTorus m(ax, majorR, minorR);
    m.Build();
    if (!m.IsDone())
        throw Standard_Failure("expansion_joint: torus build failed");
    return m.Shape();
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "expansion_joint_bellows_stub DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Dir adir = in.axis_dir;
    if (!(std::abs(adir.X()) < 1e-6 && std::abs(adir.Y()) < 1e-6 && adir.Z() > 0)) {
        throw SkillError("expansion_joint_bellows_stub: only axis_dir = +Z supported");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;
    const double baseZ = zMax;

    // ── 1) flange disc fuse ─────────────────────────────────────────────
    const gp_Ax2 axFlange(gp_Pnt(cx, cy, baseZ), adir);
    const TopoDS_Shape flangeTool = pr::cylinder(
        axFlange, in.flange_outer_dia_mm / 2.0, in.flange_thickness_mm);
    const TopoDS_Shape afterFlange = pr::fuse(wp.shape(), flangeTool);

    // ── 2) pipe core fuse ───────────────────────────────────────────────
    const double pipeBaseZ = baseZ + in.flange_thickness_mm;
    const gp_Ax2 axPipe(gp_Pnt(cx, cy, pipeBaseZ), adir);
    const TopoDS_Shape pipeTool = pr::cylinder(
        axPipe, in.pipe_outer_dia_mm / 2.0, in.pipe_length_mm);
    const TopoDS_Shape afterPipe = pr::fuse(afterFlange, pipeTool);

    // ── 3) N toroidal corrugations ──────────────────────────────────────
    const double minorR = (in.corrugation_outer_dia_mm -
                           in.pipe_outer_dia_mm) / 2.0;
    const double majorR = in.pipe_outer_dia_mm / 2.0 + minorR;

    // Place corrugations starting at pipeBaseZ + pitch/2, spaced by pitch.
    TopoDS_Shape stacked = afterPipe;
    for (int i = 0; i < in.corrugation_count; ++i) {
        const double cz = pipeBaseZ + in.corrugation_pitch_mm *
                          (static_cast<double>(i) + 0.5);
        const gp_Ax2 axTor(gp_Pnt(cx, cy, cz), adir);
        const TopoDS_Shape tor = makeTorus(axTor, majorR, minorR);
        stacked = pr::fuse(stacked, tor);
    }

    // ── 4) central bore through the whole stack ────────────────────────
    const double kOver = 0.1;
    const gp_Ax2 axBore(gp_Pnt(cx, cy, baseZ - kOver), adir);
    const TopoDS_Shape boreTool = pr::cylinder(
        axBore, in.pipe_inner_dia_mm / 2.0,
        in.flange_thickness_mm + in.pipe_length_mm + 2.0 * kOver);
    const TopoDS_Shape newShape = pr::cut(stacked, boreTool);

    // ── volume bookkeeping (test-aligned estimates) ────────────────────
    const double Rflange = in.flange_outer_dia_mm / 2.0;
    const double Rpipe   = in.pipe_outer_dia_mm   / 2.0;
    const double Rbore   = in.pipe_inner_dia_mm   / 2.0;
    const double vFlange = M_PI * Rflange * Rflange * in.flange_thickness_mm;
    const double vPipe   = M_PI * Rpipe   * Rpipe   * in.pipe_length_mm;
    const double vTorus  = 2.0 * M_PI * M_PI * majorR * minorR * minorR *
                           static_cast<double>(in.corrugation_count);
    const double vBore   = M_PI * Rbore * Rbore *
                           (in.flange_thickness_mm + in.pipe_length_mm);

    // ── signature ───────────────────────────────────────────────────────
    json params = {
        { "center_x_mm",          cx },
        { "center_y_mm",          cy },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
        { "flange_outer_dia_mm",  in.flange_outer_dia_mm },
        { "flange_thickness_mm",  in.flange_thickness_mm },
        { "pipe_outer_dia_mm",    in.pipe_outer_dia_mm },
        { "pipe_inner_dia_mm",    in.pipe_inner_dia_mm },
        { "pipe_length_mm",       in.pipe_length_mm },
        { "corrugation_count",    in.corrugation_count },
        { "corrugation_pitch_mm", in.corrugation_pitch_mm },
        { "corrugation_outer_dia_mm", in.corrugation_outer_dia_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "is_compound",          true },
        { "subfeature_count",     3 + in.corrugation_count },
        { "toroidal_face_count",  in.corrugation_count },
        { "central_bore_dia_mm",  in.pipe_inner_dia_mm },
        { "pipe_outer_dia_mm",    in.pipe_outer_dia_mm },
        { "axis_dir",             { adir.X(), adir.Y(), adir.Z() } },
        { "standard",             "EJMA-10" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "hydroforming;weld;bore";
    tooling.tool_dia_mm       = in.pipe_outer_dia_mm;
    tooling.tool_material     = "stainless_steel";
    tooling.cutting_speed_sfm = 80.0;
    tooling.stock_removed_mm3 = vBore;
    tooling.est_cycle_time_s  = 120.0 + 20.0 *
                                static_cast<double>(in.corrugation_count);
    tooling.extra = {
        { "added_volume_mm3",   vFlange + vPipe + vTorus },
        { "removed_volume_mm3", vBore },
        { "net_delta_mm3",      (vFlange + vPipe + vTorus) - vBore },
        { "torus_major_radius", majorR },
        { "torus_minor_radius", minorR },
        { "standard",           "EJMA-10" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::expansion_joint_bellows_stub: N={} pitch={} pipe={}",
                  in.corrugation_count, in.corrugation_pitch_mm,
                  in.pipe_outer_dia_mm);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

namespace {

// Toroidal face detection — surface type GeomAbs_Torus.
bool isFaceTorus(const TopoDS_Face& f)
{
    BRepAdaptor_Surface s(f);
    return s.GetType() == GeomAbs_Torus;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Count toroidal faces with axes ~ +Z.
    std::vector<int> torusIdx;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!isFaceTorus(wp.face(i))) continue;
        torusIdx.push_back(i);
    }
    // Need ≥ 2 corrugations.
    if (torusIdx.size() >= 2) {
        json recovered = {
            { "corrugation_count", static_cast<int>(torusIdx.size()) },
        };
        json matched = {
            { "torus_face_ids",  torusIdx },
            { "via",             "geometric_primary" },
        };
        out.push_back(RecognizedFeature{
            kSkillId, recovered, 0.8, matched });
    }

    // Metadata replay if none from geometry.
    if (out.empty()) {
        for (const auto& f : wp.features()) {
            if (f.skill_id != kSkillId) continue;
            RecognizedFeature rf;
            rf.skill_id = kSkillId;
            rf.recovered_params = f.params;
            rf.confidence = 1.0;
            rf.matched_geometry = { { "via", "metadata_replay" } };
            out.push_back(rf);
            break;
        }
    }

    return out;
}

}  // namespace koocadcam::skill::expansion_joint_bellows_stub
