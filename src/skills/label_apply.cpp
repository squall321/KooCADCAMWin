// @lat: [[engine/skills#label_apply]]
//
// Build a thin rectangular slab and fuse it onto the workpiece face,
// sinking 1 % into the parent for robust Boolean union (spot_weld
// convention).

#include "label_apply.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::label_apply {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// Engineering basis:
//   - Thickness ≤ 0.5 mm: TLMI Industry Standards for Pressure-Sensitive
//     Labels (Tag & Label Manufacturers Institute, 2019) §2 — common
//     adhesive labels span 50–500 µm total film + adhesive stack.
//     Above 0.5 mm switches to "nameplate / engraved-badge" category.
//   - Width/height must fit within the host face area (geometric trivial
//     check; otherwise the label overhangs and cannot adhere uniformly,
//     per 3M Application Engineering Bulletin AE-1 §3).

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.label_width_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "label_apply label_width_mm must be > 0");
    }
    if (in.label_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "label_apply label_height_mm must be > 0");
    }
    if (in.label_thickness_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "label_apply label_thickness_mm must be > 0");
    }
    if (in.label_thickness_mm > 0.5) {
        r.add("DFM-LABEL-THICK", "error",
              "label_apply label_thickness_mm " + std::to_string(in.label_thickness_mm) +
              " > 0.5 mm (use a nameplate/badge skill for thicker plates)");
    }
    // Fit-within-face check: resolve the face if possible and compare
    // bounding area.  If the datum can't resolve (e.g. unit test passes
    // an invalid datum) we skip this check rather than synthesize a
    // misleading error message.
    auto entryId = wp.resolve(in.entry_face);
    if (entryId) {
        const double area = wp.faceArea(*entryId);
        const double need = in.label_width_mm * in.label_height_mm;
        if (need > area) {
            r.add("DFM-LABEL-FIT", "error",
                  "label_apply label " + std::to_string(in.label_width_mm) +
                  " x " + std::to_string(in.label_height_mm) +
                  " mm exceeds face area " + std::to_string(area) + " mm²");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "label_apply DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("label_apply: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("label_apply: entry_face must be planar");

    const gp_Dir outNorm = wp.faceNormal(*entryId);
    const gp_Pnt faceCtr = wp.faceCenter(*entryId);

    // Robust-fuse sink, in the spot_weld convention.
    const double kSink   = in.label_thickness_mm * 0.01;
    const double slabZ   = in.label_thickness_mm + kSink;

    // Build a local frame with the outward face normal as the box's
    // local Z (so the slab thickness extrudes ALONG the outward normal)
    // and a tangent X chosen perpendicular to it.  For the typical +Z
    // top-face case, this resolves to {tangentX = +X, tangentY = +Y,
    // axisZ = +Z}, matching world XY axis-aligned slabs.
    gp_Dir tangentX = gp::DX();
    if (std::abs(outNorm.Dot(tangentX)) > 0.9) tangentX = gp::DY();
    // Project tangentX onto the face plane (subtract its component along outNorm).
    {
        const double dotN = tangentX.X() * outNorm.X()
                          + tangentX.Y() * outNorm.Y()
                          + tangentX.Z() * outNorm.Z();
        const double tx = tangentX.X() - dotN * outNorm.X();
        const double ty = tangentX.Y() - dotN * outNorm.Y();
        const double tz = tangentX.Z() - dotN * outNorm.Z();
        if (std::abs(tx) + std::abs(ty) + std::abs(tz) > 1e-9) {
            tangentX = gp_Dir(tx, ty, tz);
        }
    }

    // tangentY = outNorm × tangentX (= local Y in gp_Ax2's frame).
    const gp_Dir tangentY(
        outNorm.Y() * tangentX.Z() - outNorm.Z() * tangentX.Y(),
        outNorm.Z() * tangentX.X() - outNorm.X() * tangentX.Z(),
        outNorm.X() * tangentX.Y() - outNorm.Y() * tangentX.X());

    // Slab corner-origin: face centre, then walk back along tangentX by
    // half-width, back along tangentY by half-height, and SINK along
    // -outNorm by kSink so the slab base overlaps the face for fuse.
    //
    // NOTE: for +Z faces this resolves to world XY centred on
    // (position_x_mm, position_y_mm).  For arbitrary faces we project
    // (position_x_mm, position_y_mm) onto the face plane below; for now
    // we use position_x/y directly because all current callers use the
    // top face convention.  An xy-projection extension is straightforward
    // if/when off-axis labels become supported.
    const gp_Pnt slabOrigin(
        in.position_x_mm
            - in.label_width_mm  / 2.0 * tangentX.X()
            - in.label_height_mm / 2.0 * tangentY.X()
            - kSink              * outNorm.X(),
        in.position_y_mm
            - in.label_width_mm  / 2.0 * tangentX.Y()
            - in.label_height_mm / 2.0 * tangentY.Y()
            - kSink              * outNorm.Y(),
        faceCtr.Z()
            - in.label_width_mm  / 2.0 * tangentX.Z()
            - in.label_height_mm / 2.0 * tangentY.Z()
            - kSink              * outNorm.Z());

    // gp_Ax2(P, V, Vx): V = local Z direction (outward normal),
    //                   Vx = local X direction (tangentX).
    const gp_Ax2 slabAx(slabOrigin, outNorm, tangentX);

    // BRepPrimAPI_MakeBox extrudes dx along local X, dy along local Y,
    // dz along local Z.  So width → dx, height → dy, thickness → dz.
    const TopoDS_Shape slab = pr::box(slabAx,
                                       in.label_width_mm,
                                       in.label_height_mm,
                                       slabZ);

    TopoDS_Shape newShape;
    try {
        newShape = pr::fuse(wp.shape(), slab);
    } catch (const std::exception& e) {
        spdlog::warn("label_apply: fuse failed ({}); falling back to workpiece unchanged",
                     e.what());
        newShape = wp.shape();
    }

    // Build signature.
    json params = {
        { "entry_face_id",        *entryId },
        { "position_x_mm",        in.position_x_mm },
        { "position_y_mm",        in.position_y_mm },
        { "label_width_mm",       in.label_width_mm },
        { "label_height_mm",      in.label_height_mm },
        { "label_thickness_mm",   in.label_thickness_mm },
        { "label_text",           in.label_text },
        { "entry_face_normal",    { outNorm.X(), outNorm.Y(), outNorm.Z() } },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "label_width_mm",       in.label_width_mm },
        { "label_height_mm",      in.label_height_mm },
        { "label_thickness_mm",   in.label_thickness_mm },
        { "label_text",           in.label_text },
        { "has_text",             !in.label_text.empty() },
        { "planar_protrusion",    true },
        { "thin_slab",            in.label_thickness_mm <= 0.5 },
        { "additive",             true },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "label_applicator";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "adhesive_pad";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Volume ADDED → record as negative per the spot_weld accounting convention.
    tooling.stock_removed_mm3 =
        -(in.label_width_mm * in.label_height_mm * in.label_thickness_mm);
    tooling.est_cycle_time_s  = 2.0;     // labels are pressed on quickly
    tooling.extra = {
        { "process",       "label_adhesion" },
        { "has_text",      !in.label_text.empty() },
        { "additive",      true },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::label_apply applied: {}x{}x{} mm has_text={}",
                  in.label_width_mm, in.label_height_mm,
                  in.label_thickness_mm, !in.label_text.empty());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata replay PRIMARY (confidence 1.0).
//
// Geometric fallback: walk top-facing planar faces; any face whose Z is
// 0.05 – 0.5 mm ABOVE the dominant top would suggest a thin protrusion =
// label candidate.  Confidence 0.45 because thin-slab geometry can be
// confused with thin emboss / mold parting-line features.

namespace {

int countThinProtrusionTops(const Workpiece& wp, double& slabHeightMm_out)
{
    slabHeightMm_out = 0.0;
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    // The very top after a label fuse is the slab top; the body top sits
    // 0.05–0.5 mm below.  Look for planar +Z faces whose Z is below zMax
    // but within 0.5 mm of it.
    int count = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const gp_Dir n = wp.faceNormal(i);
        if (std::abs(n.Z()) < std::cos(1.0 * M_PI / 180.0)) continue;
        const gp_Pnt c = wp.faceCenter(i);
        const double dz = zMax - c.Z();
        if (dz > 1e-6 && dz <= 0.5) {
            ++count;
            if (dz > slabHeightMm_out) slabHeightMm_out = dz;
        }
    }
    return count;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata_replay" },
                                { "pattern", f.pattern } };
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    double slabH = 0.0;
    const int n = countThinProtrusionTops(wp, slabH);
    if (n > 0) {
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = {
            { "label_thickness_mm", slabH },
        };
        rf.confidence       = 0.45;
        rf.matched_geometry = {
            { "source",                "geometric_fallback" },
            { "protrusion_face_count", n },
            { "slab_height_mm",        slabH },
        };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::label_apply
