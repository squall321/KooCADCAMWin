// @lat: [[engine/skills#parting_surface]]

#include "parting_surface.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::parting_surface {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// Thin flange thickness — chosen so the parting surface is visually
// distinguishable but light on memory.
static constexpr double kFlangeThicknessMm = 0.5;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    if (in.silhouette_plane_z_mm < zMin - 1e-6 ||
        in.silhouette_plane_z_mm > zMax + 1e-6) {
        r.add("DFM-PART-Z", "error",
              "parting_surface silhouette_plane_z " +
              std::to_string(in.silhouette_plane_z_mm) +
              " outside workpiece Z range [" +
              std::to_string(zMin) + ", " +
              std::to_string(zMax) + "]");
    }

    if (in.extension_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "parting_surface extension_mm must be > 0");
    } else if (in.extension_mm < 2.0) {
        r.add("DFM-PART-EXT", "warning",
              "parting_surface extension " + std::to_string(in.extension_mm) +
              " mm < 2 mm — flange too thin to handle in the press");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "parting_surface DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto bb = pr::optimalBbox(wp.shape());

    // Outer flange XY bbox = workpiece bbox + extension on all sides.
    const double oXmin = bb.xMin - in.extension_mm;
    const double oYmin = bb.yMin - in.extension_mm;
    const double oXmax = bb.xMax + in.extension_mm;
    const double oYmax = bb.yMax + in.extension_mm;
    const double odx = oXmax - oXmin;
    const double ody = oYmax - oYmin;

    // The flange is centered (in Z) on the parting plane.
    const double zBot = in.silhouette_plane_z_mm - kFlangeThicknessMm / 2.0;

    // Outer flat box.
    const gp_Pnt outerOrigin(oXmin, oYmin, zBot);
    const gp_Ax2 outerAx(outerOrigin, gp::DZ());
    const TopoDS_Shape outerBox = pr::box(outerAx, odx, ody, kFlangeThicknessMm);

    // Inner cutter = workpiece XY bbox at the parting plane, extruded
    // through the flange.  This gives an "annular" flange that surrounds the
    // workpiece silhouette.  We use the workpiece bbox as a slice-1 proxy
    // for the silhouette.
    //
    // The inner cut box is INSET by kEmbed from the actual workpiece outline
    // so the annulus overlaps the workpiece by a tiny amount.  This makes
    // the fuse clean (avoids coplanar-face Boolean issues at the inner edge).
    constexpr double kEmbed = 0.02;
    const double iXmin = bb.xMin + kEmbed;
    const double iYmin = bb.yMin + kEmbed;
    const double idx = bb.dx() - 2.0 * kEmbed;
    const double idy = bb.dy() - 2.0 * kEmbed;
    const double kCutOverhang = 0.1;
    const gp_Pnt innerOrigin(iXmin, iYmin, zBot - kCutOverhang);
    const gp_Ax2 innerAx(innerOrigin, gp::DZ());
    const TopoDS_Shape innerCut = pr::box(innerAx, idx, idy,
                                           kFlangeThicknessMm + 2.0 * kCutOverhang);

    const TopoDS_Shape flange = pr::cut(outerBox, innerCut);
    if (flange.IsNull()) {
        throw SkillError("parting_surface: flange Boolean produced null shape");
    }

    // Fuse flange onto workpiece.
    const TopoDS_Shape newShape = pr::fuse(wp.shape(), flange);

    json params = {
        { "silhouette_plane_z_mm", in.silhouette_plane_z_mm },
        { "extension_mm",          in.extension_mm },
    };
    json pattern = {
        { "kind",                  kSkillId },
        { "silhouette_plane_z_mm", in.silhouette_plane_z_mm },
        { "extension_mm",          in.extension_mm },
        { "flange_thickness_mm",   kFlangeThicknessMm },
        { "outer_bbox", {
            { "xMin", oXmin }, { "xMax", oXmax },
            { "yMin", oYmin }, { "yMax", oYmax },
        } },
    };

    ToolingMeta tooling;
    tooling.tool_type = "mold_feature";
    const double flangeArea = (odx * ody) - (idx * idy);
    tooling.stock_removed_mm3 = -(flangeArea * kFlangeThicknessMm);
    tooling.est_cycle_time_s  = 0.0;

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::parting_surface applied: z={} ext={} faces {}→{}",
                  in.silhouette_plane_z_mm, in.extension_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Pattern: a large thin planar flange whose XY bbox is significantly larger
// than the rest of the workpiece's XY bbox at the same Z, with a small
// thickness (kFlangeThicknessMm ± tol).

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // History replay first.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "silhouette_plane_z_mm", f.params.value("silhouette_plane_z_mm", 0.0) },
            { "extension_mm",          f.params.value("extension_mm",          0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, 0.95, json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Topology heuristic: collect horizontal planar faces, sort by XY bbox
    // size, and look for one whose XY extent equals the workpiece's overall
    // bbox AND is strictly larger than every other horizontal planar face.
    // That is the parting flange: it spans the full XY footprint at a Z
    // strictly between the workpiece's Zmin and Zmax.
    const auto wpBb = pr::optimalBbox(wp.shape());
    if (wpBb.dx() < 1e-6 || wpBb.dy() < 1e-6) return out;

    struct Cand { int id; double dx; double dy; double z; double area; };
    std::vector<Cand> horizontals;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        try {
            const gp_Dir n = wp.faceNormal(i);
            if (std::abs(n.Z()) < 0.95) continue;
        } catch (...) { continue; }
        const auto fb = pr::optimalBbox(wp.face(i));
        horizontals.push_back({ i, fb.dx(), fb.dy(),
                                 wp.faceCenter(i).Z(),
                                 wp.faceArea(i) });
    }
    if (horizontals.empty()) return out;

    // Find candidate(s) whose XY bbox matches the workpiece bbox extents.
    int    flangeIdx = -1;
    double flangeArea = 0.0;
    double flangeZ = 0.0;
    double maxNonFlangeDx = 0.0;
    double maxNonFlangeDy = 0.0;
    for (const auto& c : horizontals) {
        const bool dxMatches = std::abs(c.dx - wpBb.dx()) < 0.5;
        const bool dyMatches = std::abs(c.dy - wpBb.dy()) < 0.5;
        if (dxMatches && dyMatches) {
            // Only treat as flange candidate if Z is strictly INSIDE the
            // workpiece Z range (not the top/bottom face).
            if (c.z > wpBb.zMin + 0.5 && c.z < wpBb.zMax - 0.5) {
                if (c.area > flangeArea) {
                    flangeArea = c.area;
                    flangeIdx = c.id;
                    flangeZ = c.z;
                }
            }
        } else {
            maxNonFlangeDx = std::max(maxNonFlangeDx, c.dx);
            maxNonFlangeDy = std::max(maxNonFlangeDy, c.dy);
        }
    }
    if (flangeIdx < 0) return out;

    // Extension = (workpiece bbox - inner workpiece footprint) / 2.
    double ext = 0.0;
    if (maxNonFlangeDx > 0.0 && maxNonFlangeDy > 0.0) {
        ext = std::min((wpBb.dx() - maxNonFlangeDx) / 2.0,
                       (wpBb.dy() - maxNonFlangeDy) / 2.0);
    }
    ext = std::max(0.0, ext);

    json rec = {
        { "silhouette_plane_z_mm", flangeZ },
        { "extension_mm",          ext },
    };
    json matched = {
        { "flange_face_id", flangeIdx },
        { "flange_area",    flangeArea },
    };
    out.push_back(RecognizedFeature{ kSkillId, rec, 0.65, matched });
    return out;
}

}  // namespace koocadcam::skill::parting_surface
