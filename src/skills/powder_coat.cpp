// @lat: [[engine/skills#powder_coat]]
//
// Slice-8 upgrade: REAL geometric thin-shell additive synthesis.  Apply
// fuses a slab of cured powder-coat film onto the target face footprint
// (typical 50 – 300 μm).  Volume delta ≈ face_area × thickness_um × 1e-3.

#include "powder_coat.hpp"

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

namespace koocadcam::skill::powder_coat {

namespace cc = koocadcam::skill::coating_common;
namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// Thickness range per Powder Coating Institute (PCI) "Powder Coating: The
// Complete Finisher's Handbook" 3rd ed., 2002, §6.2:
//   - typical decorative DFT 50 – 100 μm
//   - heavy / texture DFT 100 – 300 μm
//   - > 300 μm risks orange-peel + sag during bake
// Cure schedule per same handbook §8: 180 – 220 °C, 10 – 20 min substrate
// metal temperature (PMT).
//
// Face-area minimum: electrostatic gun spray cone needs ~ 1 cm² to hold
// charge transfer per Wagner C4-HiCoat gun manual §3.1.

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thickness_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "powder_coat thickness_um must be > 0");
    } else {
        if (in.thickness_um < 50.0) {
            r.add("DFM-PWD-THK", "error",
                  "powder_coat thickness_um " + std::to_string(in.thickness_um) +
                  " < 50 μm — below PCI Handbook §6.2 electrostatic coverage floor");
        }
        if (in.thickness_um > 300.0) {
            r.add("DFM-PWD-THK", "error",
                  "powder_coat thickness_um " + std::to_string(in.thickness_um) +
                  " > 300 μm — above PCI Handbook §6.2 single-pass build "
                  "(orange-peel / sag risk during bake)");
        }
    }

    if (in.cure_temp_c < 180.0 || in.cure_temp_c > 220.0) {
        r.add("DFM-PWD-CURE", "info",
              "powder_coat cure_temp_c " + std::to_string(in.cure_temp_c) +
              " outside typical [180, 220] °C range for polyester/epoxy "
              "resins (PCI Handbook §8) — verify powder datasheet");
    }
    if (in.cure_time_min < 5.0) {
        r.add("DFM-PWD-CURE", "info",
              "powder_coat cure_time_min " + std::to_string(in.cure_time_min) +
              " min < 5 min — likely insufficient cross-link");
    }

    if (!cc::isKnownPowderTexture(in.texture)) {
        r.add("DFM-PWD-TEXTURE", "info",
              "powder_coat texture '" + in.texture +
              "' unrecognised — expected smooth | matte | wrinkle | metallic");
    }

    if (in.color.empty()) {
        r.add("DFM-INPUT", "warning",
              "powder_coat color empty — defaulting to black");
    }

    // Face-area minimum
    if (auto fid = wp.resolve(in.entry_face); fid) {
        const double areaMm2 = wp.faceArea(*fid);
        if (areaMm2 < 100.0) {
            r.add("DFM-PWD-AREA", "error",
                  "powder_coat target face area " + std::to_string(areaMm2) +
                  " mm² < 100 mm² (1 cm²) — electrostatic gun charge-transfer floor");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "powder_coat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("powder_coat: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("powder_coat: entry_face must be planar for slab fuse");

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
    const std::string tex   = in.texture.empty() ? "smooth" : in.texture;
    const double added_mm3  = slabLen * slabWid * thickness_mm;

    json params = {
        { "entry_face_id",     *entryId },
        { "color",             color },
        { "thickness_um",      in.thickness_um },
        { "cure_temp_c",       in.cure_temp_c },
        { "cure_time_min",     in.cure_time_min },
        { "texture",           tex },
        { "entry_face_normal", { outNorm.X(), outNorm.Y(), outNorm.Z() } },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "target_face_id",       *entryId },
        { "target_face_area_mm2", faceArea },
        { "color",                color },
        { "thickness_um",         in.thickness_um },
        { "cure_temp_c",          in.cure_temp_c },
        { "cure_time_min",        in.cure_time_min },
        { "texture",              tex },
        { "cure_schedule_ok",
          in.cure_temp_c >= 180.0 && in.cure_temp_c <= 220.0
              && in.cure_time_min >= 5.0 },
        { "additive",             true },
        { "geometry_changed",     true },
        { "slab_len_mm",          slabLen },
        { "slab_wid_mm",          slabWid },
        { "slab_volume_mm3",      added_mm3 },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "electrostatic_powder_gun";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "polymer_powder_resin";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = -added_mm3;
    tooling.est_cycle_time_s  = 20.0 + in.cure_time_min * 60.0;
    tooling.extra = {
        { "process",         "electrostatic_powder_coat" },
        { "color",           color },
        { "thickness_um",    in.thickness_um },
        { "cure_temp_c",     in.cure_temp_c },
        { "cure_time_min",   in.cure_time_min },
        { "texture",         tex },
        { "stock_added_mm3", added_mm3 },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::powder_coat applied: face={} thk={}μm added={}mm³",
        *entryId, in.thickness_um, added_mm3);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Primary: planar face-pairs separated by 50 – 300 μm with matching areas.

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
            // powder range 50 – 300 μm = 0.05 – 0.3 mm.
            if (dist < 0.049 || dist > 0.31) continue;

            const double areaJ = wp.faceArea(j);
            // Thin-shell coatings leave only a slab top + thin frame.
            if (areaJ < 0.5) continue;
            const double areaRatio = (areaJ > 0.0)
                ? std::min(areaI, areaJ) / std::max(areaI, areaJ) : 0.0;

            json recovered = {
                { "entry_face_id", i },
                { "color",         "black" },
                { "thickness_um",  dist * 1000.0 },
                { "cure_temp_c",   200.0 },
                { "cure_time_min", 15.0 },
                { "texture",       "smooth" },
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

}  // namespace koocadcam::skill::powder_coat
