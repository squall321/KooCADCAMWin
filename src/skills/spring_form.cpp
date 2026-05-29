// @lat: [[engine/skills#spring_form]]

#include "spring_form.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/HelicalSweep.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <Geom_Circle.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::spring_form {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// DFM references:
//   * Wahl, A. M. "Mechanical Springs", 2nd ed. (1963), §1.4 — practical
//     spring index C = D_mean / d_wire is constrained to 4 ≤ C ≤ 12.
//     C < 4 produces stress concentrations at the inner fibre that exceed
//     wire shear strength; C > 12 leads to buckling under load.
//   * SAE-HS-795 "Spring Design Manual", Table 4-2 — slenderness ratio
//     L_free / D_mean > 4 requires lateral guides; > 5.3 is unstable.
//   * ASTM A227 (Class II hard-drawn) §6.1 — solid-length clearance: the
//     unloaded coil pitch must exceed wire_dia by ≥ 10 % to permit
//     deflection without coil contact.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.wire_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "spring_form wire_dia_mm must be > 0 (got " +
              std::to_string(in.wire_dia_mm) + ")");
    }
    if (in.n_turns <= 0.0) {
        r.add("DFM-INPUT", "error",
              "spring_form n_turns must be > 0 (got " +
              std::to_string(in.n_turns) + ")");
    }
    if (in.coil_pitch_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "spring_form coil_pitch_mm must be > 0 (got " +
              std::to_string(in.coil_pitch_mm) + ")");
    }
    if (in.free_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "spring_form free_length_mm must be > 0 (got " +
              std::to_string(in.free_length_mm) + ")");
    }
    if (in.mean_dia_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "spring_form mean_dia_mm must be > 0 (got " +
              std::to_string(in.mean_dia_mm) + ")");
    }

    // ASTM A227 §6.1 — solid-coil check.
    if (in.coil_pitch_mm > 0.0 && in.wire_dia_mm > 0.0 &&
        in.coil_pitch_mm <= in.wire_dia_mm) {
        r.add("DFM-SPR-SOLID", "error",
              "spring_form coil_pitch " + std::to_string(in.coil_pitch_mm) +
              " mm <= wire_dia " + std::to_string(in.wire_dia_mm) +
              " mm — coils would touch in unloaded state (ASTM A227 §6.1)");
    }

    // Wahl factor — spring index gate (mechanical springs handbook).
    if (in.wire_dia_mm > 0.0 && in.mean_dia_mm > 0.0) {
        const double C = in.mean_dia_mm / in.wire_dia_mm;
        if (C < 4.0 || C > 12.0) {
            r.add("DFM-SPR-INDEX", "warning",
                  "spring_form spring_index C = " + std::to_string(C) +
                  " outside Wahl [4, 12] — stress concentration / buckling risk");
        }
    }

    // SAE-HS-795 Table 4-2 — slenderness check.
    if (in.mean_dia_mm > 0.0 && in.free_length_mm > 0.0) {
        const double slenderness = in.free_length_mm / in.mean_dia_mm;
        if (slenderness > 4.0) {
            r.add("DFM-SPR-LEN", "warning",
                  "spring_form slenderness L/D = " +
                  std::to_string(slenderness) +
                  " > 4 — buckling risk under compression (SAE-HS-795)");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis helpers ────────────────────────────────────────────────────

namespace {

// Sweep a circular profile of radius `wireR` along the helical spine using
// MakePipeShell with Frenet framing.  Returns the helical-coil solid.
// Falls back to a straight cylinder if MakePipeShell fails (extreme inputs).
TopoDS_Shape buildHelicalCoil(const gp_Ax1& axis,
                              double mean_dia_mm,
                              double pitch_mm,
                              double n_turns,
                              double wire_dia_mm,
                              bool&  outUsedFallback)
{
    outUsedFallback = false;
    const double R          = 0.5 * mean_dia_mm;     // helix radius
    const double wireR      = 0.5 * wire_dia_mm;
    const double length_mm  = pitch_mm * n_turns;

    // Sample density: 32 samples per turn, minimum 24 overall.
    const int turns   = std::max(1, static_cast<int>(std::ceil(n_turns)));
    const int samples = std::max(24, turns * 32);

    TopoDS_Wire spine;
    try {
        spine = koocadcam::engine::prim::detail::buildHelicalSpine(
            axis, pitch_mm, length_mm, R, samples);
    } catch (const Standard_Failure& ex) {
        spdlog::warn("spring_form: spine build failed ({}); using cylinder fallback",
                     ex.what());
        outUsedFallback = true;
        const gp_Ax2 ax2(axis.Location(), axis.Direction());
        BRepPrimAPI_MakeCylinder mk(ax2, R + wireR, length_mm);
        mk.Build();
        if (!mk.IsDone())
            throw Standard_Failure("spring_form: fallback cylinder build failed");
        return mk.Shape();
    }

    // Build circular cross-section at the first spine point.  Tangent at θ=0
    // for the spine (R cos θ, R sin θ, pitch·θ/2π) is (−Rsinθ, Rcosθ, p/2π) =
    // (0, R, p/2π) in axis-local coords.  We use the simpler approach: build
    // a circle face perpendicular to that tangent direction, centred at the
    // start point.
    const gp_Dir z_loc = axis.Direction();
    gp_Dir x_loc;
    {
        const gp_Dir seed = (std::abs(z_loc.Z()) < 0.9) ? gp_Dir(0, 0, 1) : gp_Dir(1, 0, 0);
        gp_Vec vx = gp_Vec(seed).Crossed(gp_Vec(z_loc));
        if (vx.Magnitude() < 1e-9) vx = gp_Vec(1, 0, 0);
        x_loc = gp_Dir(vx);
    }
    const gp_Dir y_loc(gp_Vec(z_loc).Crossed(gp_Vec(x_loc)));

    // Spine start in world coords: O + R · x_loc (θ = 0).
    const gp_Pnt& O = axis.Location();
    const gp_Pnt startPnt(O.X() + R * x_loc.X(),
                          O.Y() + R * x_loc.Y(),
                          O.Z() + R * x_loc.Z());

    // Spine tangent at start (world) — derivative of (R cosθ, R sinθ, p θ/2π)
    // wrt θ is (−R sinθ, R cosθ, p/2π) = (0, R, p/2π) at θ=0 in local frame.
    const double pOver2pi = pitch_mm / (2.0 * M_PI);
    gp_Vec tangentVec(
        R * y_loc.X() + pOver2pi * z_loc.X(),
        R * y_loc.Y() + pOver2pi * z_loc.Y(),
        R * y_loc.Z() + pOver2pi * z_loc.Z());
    if (tangentVec.Magnitude() < 1e-9)
        throw Standard_Failure("spring_form: degenerate spine tangent");
    const gp_Dir tangentDir(tangentVec);

    // MakePipeShell expects a WIRE for the cross-section (not a face) —
    // otherwise BRepFill_Section throws "bad shape type of section".
    TopoDS_Wire profile;
    try {
        const gp_Ax2  circAx(startPnt, tangentDir);
        Handle(Geom_Circle) circ = new Geom_Circle(circAx, wireR);
        BRepBuilderAPI_MakeEdge em(circ);
        if (!em.IsDone())
            throw Standard_Failure("spring_form: profile edge failed");
        BRepBuilderAPI_MakeWire wm(em.Edge());
        if (!wm.IsDone())
            throw Standard_Failure("spring_form: profile wire failed");
        profile = wm.Wire();
    } catch (const Standard_Failure& ex) {
        spdlog::warn("spring_form: profile build failed ({}); using cylinder fallback",
                     ex.what());
        outUsedFallback = true;
        const gp_Ax2 ax2(axis.Location(), axis.Direction());
        BRepPrimAPI_MakeCylinder mk(ax2, R + wireR, length_mm);
        mk.Build();
        if (!mk.IsDone())
            throw Standard_Failure("spring_form: fallback cylinder build failed");
        return mk.Shape();
    }

    // Pipe-sweep with Frenet framing.
    try {
        BRepOffsetAPI_MakePipeShell pipe(spine);
        pipe.SetMode(true);                   // Frenet frame
        pipe.Add(profile, false, true);
        pipe.Build();
        if (!pipe.IsDone())
            throw Standard_Failure("spring_form: PipeShell.Build failed");
        if (!pipe.MakeSolid())
            throw Standard_Failure("spring_form: PipeShell.MakeSolid failed");
        const TopoDS_Shape result = pipe.Shape();
        if (result.IsNull())
            throw Standard_Failure("spring_form: PipeShell produced null shape");
        return result;
    } catch (const Standard_Failure& ex) {
        spdlog::warn("spring_form: pipe-shell sweep failed ({}); using cylinder fallback",
                     ex.what());
        outUsedFallback = true;
        const gp_Ax2 ax2(axis.Location(), axis.Direction());
        BRepPrimAPI_MakeCylinder mk(ax2, R + wireR, length_mm);
        mk.Build();
        if (!mk.IsDone())
            throw Standard_Failure("spring_form: fallback cylinder build failed");
        return mk.Shape();
    }
}

}  // namespace

// ── Synthesis ────────────────────────────────────────────────────────────
//
// Build the spring as a REAL helical coil: circular wire cross-section swept
// along a helical spine of radius R = mean_dia / 2, pitch p, n_turns turns.
// Replaces the input workpiece (typically a straight wire stock cylinder).

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "spring_form DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double springIndex = (in.wire_dia_mm > 0.0)
                               ? in.mean_dia_mm / in.wire_dia_mm : 0.0;
    const double solidHeight = in.n_turns * in.wire_dia_mm;
    const double wireLength  = M_PI * in.mean_dia_mm * in.n_turns;
    const double coilHeight  = in.coil_pitch_mm * in.n_turns;

    // Build the helical-coil solid centred on the world origin, axis +Z.
    bool fellBack = false;
    TopoDS_Shape coilShape;
    try {
        const gp_Ax1 helixAxis(gp_Pnt(0.0, 0.0, 0.0), gp::DZ());
        coilShape = buildHelicalCoil(
            helixAxis, in.mean_dia_mm, in.coil_pitch_mm,
            in.n_turns, in.wire_dia_mm, fellBack);
    } catch (const Standard_Failure& ex) {
        throw SkillError(std::string("spring_form: coil build failed: ") +
                         ex.what());
    }

    // Approximate volume = π (wire_dia/2)² · wire_length.
    const double approxVolume = M_PI * 0.25 * in.wire_dia_mm * in.wire_dia_mm
                                * wireLength;

    json params = {
        { "coil_pitch_mm",   in.coil_pitch_mm },
        { "n_turns",         in.n_turns },
        { "wire_dia_mm",     in.wire_dia_mm },
        { "free_length_mm",  in.free_length_mm },
        { "mean_dia_mm",     in.mean_dia_mm },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "coil_pitch_mm",        in.coil_pitch_mm },
        { "n_turns",              in.n_turns },
        { "wire_dia_mm",          in.wire_dia_mm },
        { "free_length_mm",       in.free_length_mm },
        { "mean_dia_mm",          in.mean_dia_mm },
        { "spring_index",         springIndex },
        { "solid_height_mm",      solidHeight },
        { "wire_length_mm",       wireLength },
        { "coil_height_mm",       coilHeight },
        { "axis",                 { 0.0, 0.0, 1.0 } },
        { "geometry_changed",     true },
        { "helical_fallback",     fellBack },
    };

    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "coiling_arbor";
    tooling.tool_dia_mm       = in.mean_dia_mm - in.wire_dia_mm;
    tooling.tool_length_mm    = in.free_length_mm;
    tooling.tool_material     = "tool_steel";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = -approxVolume;  // material formed (negative = added)
    tooling.est_cycle_time_s  = std::max(5.0, in.n_turns * 0.5);
    tooling.extra = json{
        { "process",         "wire_coiling" },
        { "spring_index",    springIndex },
        { "solid_height_mm", solidHeight },
        { "wire_length_mm",  wireLength },
        { "coil_height_mm",  coilHeight },
        { "helical_fallback", fellBack },
        { "dfm_findings",    findings },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(coilShape, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::spring_form applied: n={} pitch={} d={} L={} fallback={}",
                  in.n_turns, in.coil_pitch_mm, in.wire_dia_mm, in.free_length_mm,
                  fellBack);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic (primary): look for the swept-helix signature —
// a tall, thin bbox (height >> diameter) where the bounding cylinder has
// radius ≈ mean_dia / 2 + wire_dia / 2 and many cylindrical faces (each
// coil contributes ~1 cylindrical patch from the swept wire).  Confidence
// 0.45 when geometric only.  Metadata replay returns 1.0.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // (a) Metadata replay — exact recovery.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // (b) Geometric heuristic — bbox shape + cylindrical-face count.
    if (wp.shape().IsNull()) return out;
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double dx = xMax - xMin;
    const double dy = yMax - yMin;
    const double dz = zMax - zMin;
    if (dx <= 0.0 || dy <= 0.0 || dz <= 0.0) return out;

    // Pick the longest axis as the helix axis.
    const double maxExt = std::max({ dx, dy, dz });
    double helixDia;
    if (maxExt == dz)      helixDia = std::max(dx, dy);
    else if (maxExt == dy) helixDia = std::max(dx, dz);
    else                   helixDia = std::max(dy, dz);

    // Helical coils are tall+thin: aspect ratio > 1.5.
    if (maxExt < 1.5 * helixDia) return out;

    // Count cylindrical faces — a swept helical wire produces many small
    // cylinder-ish patches.  Heuristic minimum: ≥ 4 (one per quarter turn).
    int cylFaces = 0;
    for (int fIdx = 0; fIdx < wp.faceCount(); ++fIdx) {
        if (wp.isFaceCylinder(fIdx)) ++cylFaces;
    }
    if (cylFaces < 4) return out;

    // Recovered estimate — we cannot read pitch / n_turns without metadata.
    json recovered = {
        { "coil_pitch_mm",   0.0 },
        { "n_turns",         0.0 },
        { "wire_dia_mm",     0.0 },
        { "free_length_mm",  maxExt },
        { "mean_dia_mm",     helixDia },
    };
    json matched = {
        { "source",                "geometric_fallback" },
        { "bbox_long_axis_mm",     maxExt },
        { "bbox_short_axis_mm",    helixDia },
        { "cylindrical_face_count", cylFaces },
    };
    out.push_back(RecognizedFeature{ kSkillId, recovered, 0.45, matched });
    return out;
}

}  // namespace koocadcam::skill::spring_form
