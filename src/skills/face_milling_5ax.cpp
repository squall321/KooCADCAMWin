// @lat: [[engine/skills#face_milling_5ax]]

#include "face_milling_5ax.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::face_milling_5ax {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    if (in.depth_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "face_milling_5ax depth must be > 0");
    }
    if (in.depth_mm >= 5.0) {
        r.add("FM-DEPTH", "warning",
              "face_milling_5ax depth " + std::to_string(in.depth_mm) +
              " mm ≥ 5 mm — single-pass face-mill normally limited to ≤ 5 mm");
    }
    // Always emit the 5-axis advisory.  The orchestrator inspects the
    // entry_face normal and decides whether 3-axis, 3+2, or full 5-axis is
    // required.
    r.add("DFM-005", "info",
          "face_milling_5ax requires 5-axis (or 3+2) CAM when the entry "
          "face normal is not parallel to a global machine axis");
    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "face_milling_5ax DFM failed:";
        for (const auto& f : dfm.findings) {
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        }
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("face_milling_5ax: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("face_milling_5ax: entry_face must be planar");

    // Local frame is derived from the actual entry-face normal — not assumed
    // to be +Z.  This is the only structural difference from face_milling.
    const gp_Dir outNorm = wp.faceNormal(*entryId);
    const gp_Pnt faceCtr = wp.faceCenter(*entryId);

    const pr::Bbox3d bb = pr::optimalBbox(wp.shape());
    const double bboxDiag = std::sqrt(bb.dx() * bb.dx()
                                    + bb.dy() * bb.dy()
                                    + bb.dz() * bb.dz());
    // Generous pad: cover bbox diagonal in both in-plane directions.
    const double pad = bboxDiag * 1.1;

    // Pick a refDir perpendicular to outNorm (Gram-Schmidt against the most
    // orthogonal global axis).
    gp_Dir refDir = gp::DX();
    if (std::abs(outNorm.Dot(refDir)) > 0.9) refDir = gp::DY();
    {
        gp_Vec rv(refDir);
        const gp_Vec nv(outNorm);
        rv -= nv * rv.Dot(nv);
        if (rv.Magnitude() < 1e-9) rv = gp_Vec(gp::DZ());
        rv.Normalize();
        refDir = gp_Dir(rv);
    }

    // Cutting axis: into the material (-outNorm).
    const gp_Dir cutMain(-outNorm.X(), -outNorm.Y(), -outNorm.Z());
    const gp_Vec cutMainVec(cutMain);
    const gp_Vec refVec(refDir);
    const gp_Vec yLocalVec = cutMainVec.Crossed(refVec);
    const gp_Dir yLocal(yLocalVec);

    const double overhang = 0.01;
    const gp_Pnt boxOrigin(
        faceCtr.X() + outNorm.X() * overhang
                    - refDir.X() * pad - yLocal.X() * pad,
        faceCtr.Y() + outNorm.Y() * overhang
                    - refDir.Y() * pad - yLocal.Y() * pad,
        faceCtr.Z() + outNorm.Z() * overhang
                    - refDir.Z() * pad - yLocal.Z() * pad);

    const gp_Ax2 boxAx(boxOrigin, cutMain, refDir);

    const TopoDS_Shape skimBox = pr::box(boxAx,
                                         2.0 * pad,
                                         2.0 * pad,
                                         in.depth_mm + overhang);

    const TopoDS_Shape newShape = pr::cut(wp.shape(), skimBox);

    // Detect whether the entry-face normal is aligned with a global axis;
    // record this in the signature for downstream CAPP decisions.
    const double ax = std::abs(outNorm.X());
    const double ay = std::abs(outNorm.Y());
    const double az = std::abs(outNorm.Z());
    const bool isAxisAligned =
        ax > 0.99 || ay > 0.99 || az > 0.99;

    json params = {
        { "entry_face_id",     *entryId },
        { "entry_face_normal", { outNorm.X(), outNorm.Y(), outNorm.Z() } },
        { "depth_mm",          in.depth_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "parallel_face_present", true },
        { "depth_mm",              in.depth_mm },
        { "entry_face_normal",     { outNorm.X(), outNorm.Y(), outNorm.Z() } },
        { "is_axis_aligned",       isAxisAligned },
        { "requires_5axis",        !isAxisAligned },
    };
    // Pass DFM findings into tooling.extra for downstream CAPP.
    json findings = json::array();
    for (const auto& f : dfm.findings)
        findings.push_back({{ "code", f.code },
                            { "severity", f.severity },
                            { "message", f.message }});

    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = 16.0;       // 5-axis face mill — moderate size
    tooling.tool_length_mm    = 40.0;       // longer to clear the tilt
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 400.0;      // slower in tilted orientation
    tooling.feed_per_tooth_mm = 0.08;
    GProp_GProps props;
    BRepGProp::SurfaceProperties(wp.face(*entryId), props);
    tooling.stock_removed_mm3 = props.Mass() * in.depth_mm;
    tooling.est_cycle_time_s  = std::max(3.0, props.Mass() / 800.0);
    tooling.extra["dfm_findings"]         = findings;
    tooling.extra["machining_constraint"] =
        isAxisAligned
            ? std::string("3-axis sufficient (axis-aligned face)")
            : std::string("5-axis or 3+2 CAM required (angled face)");

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::face_milling_5ax applied: depth={} normal=({},{},{}) faces {}→{}",
                  in.depth_mm, outNorm.X(), outNorm.Y(), outNorm.Z(),
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Heuristic: planar faces whose normal is NOT parallel to any global axis.
// We claim them as face_milling_5ax candidates with low confidence (0.45);
// the orthogonal-normal version is owned by face_milling.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n(0.0, 0.0, 1.0);
        try { n = wp.faceNormal(i); }
        catch (...) { continue; }

        const double ax = std::abs(n.X());
        const double ay = std::abs(n.Y());
        const double az = std::abs(n.Z());
        // If parallel to any global axis (within ~0.99), let face_milling
        // own this candidate.
        if (ax > 0.99 || ay > 0.99 || az > 0.99) continue;

        const double area = wp.faceArea(i);
        if (area < 1.0) continue;  // skip micro-faces

        json recovered = {
            { "entry_face_id",     i },
            { "entry_face_normal", { n.X(), n.Y(), n.Z() } },
            // depth_mm is unrecoverable without prior state.
            { "depth_mm",          0.0 },
        };
        json matched = {
            { "face_id",   i },
            { "face_area", area },
        };
        out.push_back(RecognizedFeature{
            kSkillId, recovered, /*confidence*/ 0.45, matched
        });
    }
    return out;
}

}  // namespace koocadcam::skill::face_milling_5ax
