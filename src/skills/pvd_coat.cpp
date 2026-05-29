// @lat: [[engine/skills#pvd_coat]]
//
// Slice-8 upgrade: REAL geometric thin-shell additive synthesis.  PVD
// films are very thin (0.5 – 10 μm); we fuse a slab matching the target
// face footprint to deposit the requested ceramic layer.  Because the
// thickness is sub-micron in the worst case, we use a larger overlap sink
// (max(thickness × 0.5, 1 μm)) to keep the Boolean fuse robust at OCCT
// confusion scale (~1e-7 mm).  The sink portion is absorbed by the parent
// volume, so the reported volume delta still equals
// (slab_footprint × thickness_mm).

#include "pvd_coat.hpp"

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

namespace koocadcam::skill::pvd_coat {

namespace cc = koocadcam::skill::coating_common;
namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────
//
// Thickness range per Bunshah "Handbook of Deposition Technologies for
// Films and Coatings" 2nd ed. (Noyes 1994), §13.3 (PVD process maps):
//   - decorative TiN          0.5  – 2 μm
//   - cutting-tool TiAlN/CrN  2    – 6 μm
//   - wear/erosion DLC        2    – 10 μm
// > 10 μm risks columnar grain delamination + residual-stress spallation
// (Bunshah Table 13.5).  We bracket [0.5, 10] μm.
//
// Compound hardness ranges per Holmberg & Matthews "Coatings Tribology"
// 2nd ed. (Elsevier 2009), Table 7.2.
//
// Face-area minimum: PVD chamber bias holder needs ~ 1 cm² for stable
// substrate-bias current (Bunshah §13.6 fixturing).

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.thickness_um <= 0.0) {
        r.add("DFM-INPUT", "error",
              "pvd_coat thickness_um must be > 0");
    } else {
        if (in.thickness_um < 0.5) {
            r.add("DFM-PVD-THK", "error",
                  "pvd_coat thickness_um " + std::to_string(in.thickness_um) +
                  " < 0.5 μm — below Bunshah Handbook §13.3 PVD nucleation thickness");
        }
        if (in.thickness_um > 10.0) {
            r.add("DFM-PVD-THK", "error",
                  "pvd_coat thickness_um " + std::to_string(in.thickness_um) +
                  " > 10 μm — exceeds Bunshah Table 13.5 PVD ceiling "
                  "(columnar delamination risk)");
        }
    }

    const cc::PvdCompoundEntry* compound = cc::findPvdCompound(in.coating_compound);
    if (!compound) {
        r.add("DFM-PVD-COMPOUND", "info",
              "pvd_coat coating_compound '" + in.coating_compound +
              "' not in known table {TiN, TiAlN, CrN, DLC} — process plan "
              "will treat as custom chemistry");
    } else if (in.hardness_hv > 0.0) {
        if (in.hardness_hv < compound->hardness_min_hv
            || in.hardness_hv > compound->hardness_max_hv) {
            r.add("DFM-PVD-HARDNESS", "info",
                  "pvd_coat hardness_hv " + std::to_string(in.hardness_hv) +
                  " HV outside expected " + in.coating_compound + " range [" +
                  std::to_string(compound->hardness_min_hv) + ", " +
                  std::to_string(compound->hardness_max_hv) + "] HV — verify"
                  " deposition recipe (Holmberg & Matthews Table 7.2)");
        }
    }

    // Face-area minimum
    if (auto fid = wp.resolve(in.entry_face); fid) {
        const double areaMm2 = wp.faceArea(*fid);
        if (areaMm2 < 100.0) {
            r.add("DFM-PVD-AREA", "error",
                  "pvd_coat target face area " + std::to_string(areaMm2) +
                  " mm² < 100 mm² (1 cm²) — Bunshah §13.6 fixturing floor");
        }
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "pvd_coat DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("pvd_coat: entry_face datum unresolved");
    if (!wp.isFacePlanar(*entryId))
        throw SkillError("pvd_coat: entry_face must be planar for slab fuse");

    const cc::PvdCompoundEntry* compound = cc::findPvdCompound(in.coating_compound);
    const cc::RgbHint rgb = compound ? compound->color_hint
                                     : cc::RgbHint{ 180, 180, 180 };
    double hardness = in.hardness_hv;
    if (hardness <= 0.0 && compound) {
        hardness = 0.5 * (compound->hardness_min_hv + compound->hardness_max_hv);
    }

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

    const double thickness_mm = in.thickness_um * 1.0e-3;
    // PVD-specific sink: use max(thk × 0.5, 1 μm) so even at 0.5 μm films
    // the fuse has enough overlap to survive OCCT confusion (~1e-7 mm).
    const double sink         = std::max(thickness_mm * 0.5, 1.0e-3);
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

    const double added_mm3 = slabLen * slabWid * thickness_mm;

    json params = {
        { "entry_face_id",     *entryId },
        { "coating_compound",  in.coating_compound },
        { "thickness_um",      in.thickness_um },
        { "hardness_hv",       hardness },
        { "entry_face_normal", { outNorm.X(), outNorm.Y(), outNorm.Z() } },
    };
    json pattern = {
        { "kind",                 kSkillId },
        { "target_face_id",       *entryId },
        { "target_face_area_mm2", faceArea },
        { "coating_compound",     in.coating_compound },
        { "thickness_um",         in.thickness_um },
        { "hardness_hv",          hardness },
        { "color_rgb_hint",       { rgb.r, rgb.g, rgb.b } },
        { "hardness_in_compound_range",
          compound
            ? (hardness >= compound->hardness_min_hv
               && hardness <= compound->hardness_max_hv)
            : false },
        { "additive",             true },
        { "geometry_changed",     true },
        { "slab_len_mm",          slabLen },
        { "slab_wid_mm",          slabWid },
        { "slab_volume_mm3",      added_mm3 },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "pvd_chamber";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = in.coating_compound + "_target";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = -added_mm3;
    // Typical arc-PVD deposition rate ~ 0.5 μm/h + 20-min pump-down floor.
    tooling.est_cycle_time_s  = std::max(20.0 * 60.0,
                                         in.thickness_um / 0.5 * 3600.0);
    tooling.extra = {
        { "process",          "physical_vapor_deposition" },
        { "coating_compound", in.coating_compound },
        { "thickness_um",     in.thickness_um },
        { "hardness_hv",      hardness },
        { "stock_added_mm3",  added_mm3 },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug(
        "skill::pvd_coat applied: face={} compound={} thk={}μm added={}mm³",
        *entryId, in.coating_compound, in.thickness_um, added_mm3);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Primary: planar face-pairs separated by 0.5 – 10 μm.  PVD films are at
// the edge of OCCT detection; we widen the band slightly to handle fuse
// merge artefacts.

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
            // PVD range 0.5 – 10 μm = 0.0005 – 0.01 mm.
            if (dist < 0.0004 || dist > 0.011) continue;

            const double areaJ = wp.faceArea(j);
            const double areaRatio = (areaJ > 0.0)
                ? std::min(areaI, areaJ) / std::max(areaI, areaJ) : 0.0;
            if (areaRatio < 0.7) continue;

            json recovered = {
                { "entry_face_id",    i },
                { "coating_compound", "TiN" },
                { "thickness_um",     dist * 1000.0 },
                { "hardness_hv",      2250.0 },     // TiN midpoint
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

}  // namespace koocadcam::skill::pvd_coat
