// @lat: [[engine/skills#auto_rib_between_two_walls]]

#include "auto_rib_between_two_walls.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::auto_rib_between_two_walls {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Lookup table: BASF plastic rib design ratios ─────────────────────────
namespace {

struct RibStyle {
    const char* key;
    double      ratio;     // rib_thick / wall_thick
};

constexpr std::array<RibStyle, 3> kRibTable {{
    { "thin",     0.50 },
    { "standard", 0.60 },
    { "thick",    0.75 },
}};

const RibStyle* lookupStyle(const std::string& key)
{
    for (const auto& s : kRibTable)
        if (key == s.key) return &s;
    return nullptr;
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.wall_A_face_id < 0 || in.wall_A_face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "wall_A_face_id out of range (got " +
              std::to_string(in.wall_A_face_id) +
              ", workpiece has " + std::to_string(wp.faceCount()) + " faces)");
    }
    if (in.wall_B_face_id < 0 || in.wall_B_face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "wall_B_face_id out of range (got " +
              std::to_string(in.wall_B_face_id) + ")");
    }
    if (in.wall_A_face_id == in.wall_B_face_id) {
        r.add("DFM-INPUT", "error", "wall_A_face_id == wall_B_face_id");
    }
    if (in.rib_thick_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "rib_thick_mm must be > 0");
    }
    if (in.wall_thick_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "wall_thick_mm must be > 0");
    }

    const RibStyle* style = lookupStyle(in.rib_style);
    if (!style) {
        r.add("DFM-RIB-SPEC", "error",
              "auto_rib_between_two_walls: unknown rib_style '" +
              in.rib_style + "' (supported: thin/standard/thick)");
        return r;
    }
    if (!r.passed) return r;

    // BASF rule: rib_thick / wall_thick should be ≤ 0.75 (else sink marks).
    const double ratio = in.rib_thick_mm / in.wall_thick_mm;
    if (ratio > 0.75) {
        r.add("DFM-RIB-RATIO", "error",
              "rib_thick/wall_thick " + std::to_string(ratio) +
              " > 0.75 (BASF sink-mark rule)");
    }

    // Aspect-ratio warning: when length/thickness > 40 the rib will warp.
    if (in.wall_A_face_id >= 0 && in.wall_A_face_id < wp.faceCount() &&
        in.wall_B_face_id >= 0 && in.wall_B_face_id < wp.faceCount()) {
        const gp_Pnt cA = wp.faceCenter(in.wall_A_face_id);
        const gp_Pnt cB = wp.faceCenter(in.wall_B_face_id);
        const double length = cA.Distance(cB);
        if (in.rib_thick_mm > 0.0 && length / in.rib_thick_mm > 40.0) {
            r.add("DFM-RIB-AR", "warning",
                  "rib aspect length/thick " +
                  std::to_string(length / in.rib_thick_mm) +
                  " > 40 (warpage risk)");
        }
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "auto_rib_between_two_walls DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    // ── Context: read wall face centres + workpiece bbox ─────────────────
    const gp_Pnt cA = wp.faceCenter(in.wall_A_face_id);
    const gp_Pnt cB = wp.faceCenter(in.wall_B_face_id);

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // The rib runs from cA to cB along the line connecting their centres.
    const gp_Vec ribVec(cA, cB);
    const double ribLength = ribVec.Magnitude();
    if (ribLength < 1e-3) {
        throw SkillError("auto_rib_between_two_walls: walls too close");
    }
    gp_Dir ribDir(ribVec);
    const gp_Pnt midPoint(
        (cA.X() + cB.X()) / 2.0,
        (cA.Y() + cB.Y()) / 2.0,
        (cA.Z() + cB.Z()) / 2.0);

    // Rib height — auto-detect if not specified.
    const double ribHeight = (in.rib_height_mm > 0.0)
        ? in.rib_height_mm
        : std::max(2.0, 0.6 * (zMax - zMin));

    const double t = in.rib_thick_mm;

    // ── Sub-feature 1: fuse rectangular rib block ────────────────────────
    //
    // Box origin = midPoint shifted by:
    //   - ribLength/2 backward along ribDir
    //   - t/2 to one side along the perpendicular
    //   - ribHeight downward (so it sits between cA and cB midZ)
    //
    // Build with gp_Ax2(origin, mainDir=Z_world, xDir=ribDir):
    //   DX = ribLength, DY = t, DZ = ribHeight.
    gp_Dir mainZ = gp::DZ();
    // Defensive: if ribDir is colinear with Z, swap mainDir to Y.
    if (std::abs(ribDir.Dot(mainZ)) > 0.99) {
        mainZ = gp::DY();
    }
    // Place rib BOTTOM at midPoint.Z() - ribHeight/2.
    const gp_Pnt boxOrigin(
        midPoint.X() - ribDir.X() * ribLength / 2.0,
        midPoint.Y() - ribDir.Y() * ribLength / 2.0,
        midPoint.Z() - ribHeight / 2.0);
    const gp_Ax2 ribAx(boxOrigin, mainZ, ribDir);
    // ax: DZ = mainZ = world Z (height), DX = ribDir (length), DY = mainZ×ribDir
    const TopoDS_Shape ribBox = pr::box(ribAx, ribLength, t, ribHeight);

    TopoDS_Shape afterFuse = pr::fuse(wp.shape(), ribBox);

    // ── Sub-feature 2 + 3: chamfer cuts at each wall junction ───────────
    // Approximate the chamfer as a small cut box at each end face of the rib
    // along its main axis — chamSize = min(t, 0.5 mm).
    const double cham = std::min(t * 0.5, 0.5);
    const gp_Pnt chamA(
        cA.X() + ribDir.X() * cham * 0.5,
        cA.Y() + ribDir.Y() * cham * 0.5,
        cA.Z() - cham * 0.5);
    const gp_Pnt chamB(
        cB.X() - ribDir.X() * cham * 0.5,
        cB.Y() - ribDir.Y() * cham * 0.5,
        cB.Z() - cham * 0.5);
    const gp_Ax2 chamAxA(chamA, mainZ, ribDir);
    const gp_Ax2 chamAxB(chamB, mainZ, ribDir);
    const TopoDS_Shape chamToolA = pr::box(chamAxA, cham, cham, cham);
    const TopoDS_Shape chamToolB = pr::box(chamAxB, cham, cham, cham);

    std::vector<TopoDS_Shape> chamTools{ chamToolA, chamToolB };
    const TopoDS_Shape newShape = pr::cutMany(afterFuse, chamTools);

    // ── Bookkeeping ─────────────────────────────────────────────────────
    const double vRib  = ribLength * t * ribHeight;
    const double vCham = 2.0 * cham * cham * cham;

    json params = {
        { "wall_A_face_id", in.wall_A_face_id },
        { "wall_B_face_id", in.wall_B_face_id },
        { "rib_thick_mm",   in.rib_thick_mm },
        { "rib_height_mm",  in.rib_height_mm },
        { "rib_style",      in.rib_style },
        { "wall_thick_mm",  in.wall_thick_mm },
    };
    json pattern = {
        { "kind",               kSkillId },
        { "is_compound",        true },
        { "subfeature_count",   3 },
        { "rib_style",          in.rib_style },
        { "rib_length_mm",      ribLength },
        { "rib_height_mm",      ribHeight },
        { "rib_thick_mm",       t },
        { "auto_detected_length", true },
        { "standard",           "BASF moulding guide" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "mould;end_mill";
    tooling.tool_dia_mm       = t;
    tooling.tool_material     = "P20-steel";
    tooling.stock_removed_mm3 = vCham;
    tooling.est_cycle_time_s  = 25.0;
    tooling.extra = {
        { "rib_volume_mm3",     vRib },
        { "chamfer_volume_mm3", vCham },
        { "added_volume_mm3",   vRib },
        { "removed_volume_mm3", vCham },
        { "net_delta_mm3",      vRib - vCham },
        { "rib_length_detected_mm", ribLength },
        { "standard",           "BASF moulding guide" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::auto_rib_between_two_walls: L={} t={} style={}",
                  ribLength, t, in.rib_style);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic: a freshly-fused rib leaves 4 thin parallel planar
// faces (length×height) plus 2 endcap faces.  Slice-1 approximation:
// look for a clustered pair of parallel planar faces narrower than 5 mm
// apart, with the same outward normal vectors flipped.  Fallback to
// metadata replay.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    auto emitMetadataReplay = [&]() {
        for (const auto& f : wp.features()) {
            if (f.skill_id != kSkillId) continue;
            RecognizedFeature rf;
            rf.skill_id = kSkillId;
            rf.recovered_params = f.params;
            rf.confidence = 1.0;
            rf.matched_geometry = { { "via", "metadata_replay" } };
            out.push_back(rf);
            return;
        }
    };

    // Collect planar faces with their normals + centres.
    struct PlaneInfo { int idx; gp_Pnt centre; gp_Dir normal; double area; };
    std::vector<PlaneInfo> planes;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        try {
            PlaneInfo pi;
            pi.idx    = i;
            pi.centre = wp.faceCenter(i);
            pi.normal = wp.faceNormal(i);
            pi.area   = wp.faceArea(i);
            planes.push_back(pi);
        } catch (...) {}
    }

    // Find pairs of parallel, anti-aligned planes separated by ≤ 5 mm — those
    // are the two long faces of a thin rib.
    for (size_t i = 0; i < planes.size(); ++i) {
        for (size_t j = i + 1; j < planes.size(); ++j) {
            const double dot = planes[i].normal.X() * planes[j].normal.X() +
                               planes[i].normal.Y() * planes[j].normal.Y() +
                               planes[i].normal.Z() * planes[j].normal.Z();
            if (dot > -0.99) continue;  // need anti-parallel normals
            const double dist = planes[i].centre.Distance(planes[j].centre);
            if (dist > 5.0 || dist < 0.5) continue;

            const gp_Pnt mid(
                (planes[i].centre.X() + planes[j].centre.X()) / 2.0,
                (planes[i].centre.Y() + planes[j].centre.Y()) / 2.0,
                (planes[i].centre.Z() + planes[j].centre.Z()) / 2.0);
            (void)mid;

            json recovered = {
                { "wall_A_face_id", planes[i].idx },
                { "wall_B_face_id", planes[j].idx },
                { "rib_thick_mm",   dist },
                { "rib_style",      "standard" },
                { "wall_thick_mm",  std::max(dist / 0.6, 2.0) },
                { "rib_height_mm",  0.0 },
            };
            json matched = {
                { "face_A", planes[i].idx },
                { "face_B", planes[j].idx },
                { "via",    "geometric_primary" },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.7, matched });
        }
    }

    if (out.empty()) emitMetadataReplay();
    return out;
}

}  // namespace koocadcam::skill::auto_rib_between_two_walls
