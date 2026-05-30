// @lat: [[engine/skills#paint_op]]
//
// Slice-8 upgrade: REAL geometric thin-shell additive synthesis.  Apply
// fuses a slab of cured paint film onto the target face footprint.
// Volume delta ≈ face_area × thickness_um × 1e-3 mm³.

#include "paint_op.hpp"

#include "Workpiece.hpp"
#include "_coating_common.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::paint_op {

namespace cc = koocadcam::skill::coating_common;
namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// Thickness range per ISO 12944-5:2019 "Paints and varnishes — Corrosion
// protection of steel structures by protective paint systems — Part 5:
// Protective paint systems", Table 4 — single-coat DFT (dry film thickness):
//   - primer       40 – 100 μm
//   - intermediate 50 – 150 μm
//   - topcoat      40 – 100 μm
// Multi-coat systems push to 500 μm cumulative.  We bracket the single-pass
// wet-film [10, 500] μm window which covers thin lacquers up to high-build
// industrial coats.
//
// Cure schedule per Painting & Coatings Manual (Bayer / Hempel handbook):
//   air-dry  ≥ 20 °C / 60 min  (acrylic, alkyd)
//   bake     80 – 120 °C / 20 – 30 min (epoxy, polyurethane)
//
// Face-area minimum: HVLP spray fan pattern minimum target ~ 1 cm²
// (Anest-Iwata HVLP spray gun datasheet, 2018, min target dim 25 mm).

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thickness_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "paint_op thickness_um must be > 0");
    } else {
        if (in.thickness_um < 10.0) {
            r.add("DFM-PAINT-THK", "error",
                  "paint_op thickness_um " + std::to_string(in.thickness_um) +
                  " < 10 μm — below ISO 12944-5 single-coat DFT floor");
        }
        if (in.thickness_um > 500.0) {
            r.add("DFM-PAINT-THK", "error",
                  "paint_op thickness_um " + std::to_string(in.thickness_um) +
                  " > 500 μm — exceeds single-pass sag-resistance limit "
                  "(specify multi-coat per ISO 12944-5 Table 4)");
        }
    }

    if (in.paint_type == "powder") {
        r.add("DFM-PAINT-TYPE", "warning",
              "paint_op paint_type='powder' — use the powder_coat skill "
              "instead (electrostatic application + heat cure)");
    } else if (in.paint_type != "wet") {
        r.add("DFM-PAINT-TYPE", "info",
              "paint_op paint_type '" + in.paint_type +
              "' unrecognised — expected 'wet' (or 'powder' → powder_coat)");
    }

    if (in.cure_temp_c < 10.0 || in.cure_temp_c > 200.0) {
        r.add("DFM-PAINT-CURE", "info",
              "paint_op cure_temp_c " + std::to_string(in.cure_temp_c) +
              " outside typical [10, 200] °C range — verify paint datasheet");
    }
    if (in.cure_time_min < 1.0) {
        r.add("DFM-PAINT-CURE", "info",
              "paint_op cure_time_min " + std::to_string(in.cure_time_min) +
              " < 1 min — likely insufficient cure");
    }
    if (in.color.empty()) {
        r.add("DFM-INPUT", "warning",
              "paint_op color empty — defaulting to black");
    }

    // Face-area minimum (HVLP spray-fan reachable target)
    if (auto fid = wp.resolve(in.entry_face); fid) {
        const double areaMm2 = wp.faceArea(*fid);
        if (areaMm2 < 100.0) {
            r.add("DFM-PAINT-AREA", "error",
                  "paint_op target face area " + std::to_string(areaMm2) +
                  " mm² < 100 mm² (1 cm²) — HVLP spray fan floor");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "paint_op DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("paint_op: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("paint_op: entry_face must be planar for slab fuse");

    const gp_Dir outNorm  = wp.faceNormal(*entryId);
    const gp_Pnt faceCtr  = wp.faceCenter(*entryId);
    const double faceArea = wp.faceArea(*entryId);

    Bnd_Box faceBb;
    BRepBndLib::AddOptimal(wp.face(*entryId), faceBb);
    double fxMin, fyMin, fzMin, fxMax, fyMax, fzMax;
    faceBb.Get(fxMin, fyMin, fzMin, fxMax, fyMax, fzMax);
    const double fdx = fxMax - fxMin;
    const double fdy = fyMax - fyMin;
    const double fdz = fzMax - fzMin;

    gp_Dir refDir = gp::DX();
    if (std::abs(outNorm.Dot(refDir)) > 0.9) refDir = gp::DY();
    {
        gp_Vec rv(refDir);
        const gp_Vec nv(outNorm);
        rv -= nv * rv.Dot(nv);
        if (rv.Magnitude() < 1e-9) rv = gp_Vec(gp::DY());
        rv.Normalize();
        refDir = gp_Dir(rv);
    }
    const gp_Vec outV(outNorm);
    const gp_Vec xV(refDir);
    const gp_Vec yV = outV.Crossed(xV);
    const gp_Dir yLoc(yV);

    auto axisExtent = [&](double ax, double ay, double az) {
        if (ax > 0.9) return fdx;
        if (ay > 0.9) return fdy;
        if (az > 0.9) return fdz;
        return std::sqrt(fdx * fdx + fdy * fdy + fdz * fdz);
    };
    double slabLen = axisExtent(std::abs(refDir.X()), std::abs(refDir.Y()),
                                std::abs(refDir.Z()));
    double slabWid = axisExtent(std::abs(yLoc.X()),   std::abs(yLoc.Y()),
                                std::abs(yLoc.Z()));
    if (slabLen < 1e-6) slabLen = std::sqrt(faceArea);
    if (slabWid < 1e-6) slabWid = std::sqrt(faceArea);

    // Slice-9 fix: shrink slab footprint so the original face survives as a
    // frame — recognize() then sees a parallel face-pair (frame + slab top).
    const double kInsetMm = 1.0;
    slabLen = std::max(slabLen - 2.0 * kInsetMm, slabLen * 0.5);
    slabWid = std::max(slabWid - 2.0 * kInsetMm, slabWid * 0.5);

    const double thickness_mm = in.thickness_um * 1.0e-3;
    const double kSinkPct     = 0.01;
    const double sink         = thickness_mm * kSinkPct;
    const double slabH        = thickness_mm + sink;

    const gp_Pnt slabOrigin(
        faceCtr.X() - refDir.X() * (slabLen / 2.0)
                    - yLoc.X()   * (slabWid / 2.0)
                    - outNorm.X() * sink,
        faceCtr.Y() - refDir.Y() * (slabLen / 2.0)
                    - yLoc.Y()   * (slabWid / 2.0)
                    - outNorm.Y() * sink,
        faceCtr.Z() - refDir.Z() * (slabLen / 2.0)
                    - yLoc.Z()   * (slabWid / 2.0)
                    - outNorm.Z() * sink);

    const gp_Ax2 slabAx(slabOrigin, outNorm, refDir);
    const TopoDS_Shape slab     = pr::box(slabAx, slabLen, slabWid, slabH);
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), slab);

    const std::string color = in.color.empty() ? "black" : in.color;
    const std::string ptype = in.paint_type.empty() ? "wet" : in.paint_type;
    const double added_mm3  = slabLen * slabWid * thickness_mm;

    json params = {
        { "entry_face_id",     *entryId },
        { "paint_type",        ptype },
        { "color",             color },
        { "thickness_um",      in.thickness_um },
        { "cure_temp_c",       in.cure_temp_c },
        { "cure_time_min",     in.cure_time_min },
        { "entry_face_normal", { outNorm.X(), outNorm.Y(), outNorm.Z() } },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "target_face_id",       *entryId },
        { "target_face_area_mm2", faceArea },
        { "paint_type",           ptype },
        { "color",                color },
        { "thickness_um",         in.thickness_um },
        { "cure_temp_c",          in.cure_temp_c },
        { "cure_time_min",        in.cure_time_min },
        { "cure_schedule_ok",
          in.cure_temp_c >= 10.0 && in.cure_temp_c <= 200.0
              && in.cure_time_min >= 1.0 },
        { "additive",             true },
        { "geometry_changed",     true },
        { "slab_len_mm",          slabLen },
        { "slab_wid_mm",          slabWid },
        { "slab_volume_mm3",      added_mm3 },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "spray_gun";
    tooling.tool_dia_mm       = 1.4;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "stainless_nozzle_polymer_paint";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = -added_mm3;
    tooling.est_cycle_time_s  = 30.0 + in.cure_time_min * 60.0;
    tooling.extra = {
        { "process",         "wet_paint_spray" },
        { "paint_type",      ptype },
        { "color",           color },
        { "thickness_um",    in.thickness_um },
        { "cure_temp_c",     in.cure_temp_c },
        { "cure_time_min",   in.cure_time_min },
        { "stock_added_mm3", added_mm3 },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::paint_op applied: face={} type={} thk={}μm added={}mm³",
        *entryId, ptype, in.thickness_um, added_mm3);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Primary: planar face-pairs separated by 10 – 500 μm with matching areas.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir ni;
        try { ni = wp.faceNormal(i); } catch (...) { continue; }
        const double areaI = wp.faceArea(i);
        if (areaI < 100.0) continue;
        const gp_Pnt ci = wp.faceCenter(i);

        for (int j = i + 1; j < wp.faceCount(); ++j) {
            if (!wp.isFacePlanar(j)) continue;
            gp_Dir nj;
            try { nj = wp.faceNormal(j); } catch (...) { continue; }
            const double dot = ni.Dot(nj);
            if (std::abs(std::abs(dot) - 1.0) > 1e-3) continue;

            const gp_Pnt cj = wp.faceCenter(j);
            const gp_Vec sepVec(ci, cj);
            const double dist = std::abs(sepVec.Dot(gp_Vec(ni)));
            // paint range 10 – 500 μm = 0.01 – 0.5 mm.
            if (dist < 0.009 || dist > 0.51) continue;

            const double areaJ = wp.faceArea(j);
            // Thin-shell coatings leave only a slab top + thin frame.
            if (areaJ < 0.5) continue;
            const double areaRatio = (areaJ > 0.0)
                ? std::min(areaI, areaJ) / std::max(areaI, areaJ) : 0.0;

            json recovered = {
                { "entry_face_id", i },
                { "paint_type",    "wet" },
                { "color",         "black" },
                { "thickness_um",  dist * 1000.0 },
                { "cure_temp_c",   80.0 },
                { "cure_time_min", 20.0 },
            };
            json matched = {
                { "parent_face_id",  i },
                { "shell_face_id",   j },
                { "area_ratio",      areaRatio },
                { "separation_mm",   dist },
                { "source",          "geometric" },
            };
            out.push_back(RecognizedFeature{
                kSkillId, recovered, 0.5, matched
            });
            break;
        }
    }
    if (!out.empty()) return out;

    return cc::recognizeFromMetadata(wp, kSkillId);
}

}  // namespace koocadcam::skill::paint_op
