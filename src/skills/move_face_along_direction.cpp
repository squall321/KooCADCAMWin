// @lat: [[engine/skills#move_face_along_direction]]
//
// Approximation strategy (see header for rationale):
//   Compute the face's planar AABB on its plane → build a box of that
//   footprint × |distance| mm tall along `direction`.  If distance > 0
//   fuse (extrude OUTWARD), if distance < 0 cut (recess INWARD).
//
// Robust face translation via BRepFeat_DPrism would require sketch-based
// definition of the sliding plane and a topological history map; it is
// known to be fragile across OCCT versions for arbitrary face shapes.

#include "move_face_along_direction.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::move_face_along_direction {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    const double dnorm = std::sqrt(in.direction_xyz[0] * in.direction_xyz[0] +
                                   in.direction_xyz[1] * in.direction_xyz[1] +
                                   in.direction_xyz[2] * in.direction_xyz[2]);
    if (dnorm < 1e-9) {
        r.add("DFM-INPUT", "error",
              "move_face_along_direction: direction vector is zero-length");
    }
    if (std::abs(in.distance_mm) < 1e-6) {
        r.add("DFM-INPUT", "error",
              "move_face_along_direction: distance_mm is ~0 (identity move)");
    }
    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "move_face_along_direction: face_id " + std::to_string(in.face_id) +
              " out of range [0," + std::to_string(wp.faceCount()) + ")");
        return r;
    }
    if (!wp.isFacePlanar(in.face_id)) {
        r.add("DFM-INPUT", "error",
              "move_face_along_direction: face " + std::to_string(in.face_id) +
              " is not planar (approximation only supports planar faces)");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "move_face_along_direction DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Vec dirVec(in.direction_xyz[0], in.direction_xyz[1], in.direction_xyz[2]);
    const gp_Dir dir = gp_Dir(dirVec);

    // Get the face bbox to define footprint of the prism.
    const TopoDS_Face& face = wp.face(in.face_id);
    Bnd_Box fb;
    BRepBndLib::AddOptimal(face, fb);
    double fxMin, fyMin, fzMin, fxMax, fyMax, fzMax;
    fb.Get(fxMin, fyMin, fzMin, fxMax, fyMax, fzMax);

    // Build an axis-aligned box of footprint sx × sy and height |distance|
    // positioned so that one of its faces lies on the source face's plane,
    // extruding along `dir` (snapped to nearest axis if near-axial; otherwise
    // we use the full XYZ extent of the face bbox).
    const double sx = std::max(1e-3, fxMax - fxMin);
    const double sy = std::max(1e-3, fyMax - fyMin);
    const double sz = std::max(1e-3, fzMax - fzMin);
    const double absDist = std::abs(in.distance_mm);

    // Pick the dominant axis of the direction vector.
    //
    // Place the prism so it ALWAYS straddles or sits on the face plane:
    //   - distance > 0 → prism extends OUTWARD along +dir (to be FUSED).
    //   - distance < 0 → prism extends INWARD along -dir into the body,
    //                    starting at the face plane (to be CUT).
    // In either case the slab's "near face" coincides with the moved face.
    const double ax = std::abs(dir.X());
    const double ay = std::abs(dir.Y());
    const double az = std::abs(dir.Z());
    const bool outward = (in.distance_mm > 0.0);

    TopoDS_Shape prism;
    if (ax >= ay && ax >= az) {
        // Extrude along X.  Face plane is at fxMax (if +X) or fxMin (if -X).
        double startX;
        if (dir.X() >= 0.0) {
            startX = outward ? fxMax : fxMax - absDist;
        } else {
            startX = outward ? fxMin - absDist : fxMin;
        }
        const gp_Pnt origin(startX, fyMin, fzMin);
        prism = pr::box(gp_Ax2(origin, gp::DZ()), absDist, sy, sz);
    } else if (ay >= ax && ay >= az) {
        double startY;
        if (dir.Y() >= 0.0) {
            startY = outward ? fyMax : fyMax - absDist;
        } else {
            startY = outward ? fyMin - absDist : fyMin;
        }
        const gp_Pnt origin(fxMin, startY, fzMin);
        prism = pr::box(gp_Ax2(origin, gp::DZ()), sx, absDist, sz);
    } else {
        double startZ;
        if (dir.Z() >= 0.0) {
            startZ = outward ? fzMax : fzMax - absDist;
        } else {
            startZ = outward ? fzMin - absDist : fzMin;
        }
        const gp_Pnt origin(fxMin, fyMin, startZ);
        prism = pr::box(gp_Ax2(origin, gp::DZ()), sx, sy, absDist);
    }

    // distance > 0 → fuse (add material outward)
    // distance < 0 → cut  (remove material inward)
    TopoDS_Shape newShape;
    if (in.distance_mm > 0.0) {
        newShape = pr::fuse(wp.shape(), prism);
    } else {
        newShape = pr::cut(wp.shape(), prism);
    }

    json params = {
        { "face_id",       in.face_id },
        { "direction_xyz", in.direction_xyz },
        { "distance_mm",   in.distance_mm },
    };
    json pattern = {
        { "kind",           kSkillId },
        { "is_compound",    false },
        { "direction_xyz",  { dir.X(), dir.Y(), dir.Z() } },
        { "distance_mm",    in.distance_mm },
        { "outward",        in.distance_mm > 0.0 },
        { "footprint_sx",   sx },
        { "footprint_sy",   sy },
        { "footprint_sz",   sz },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "end_mill";
    tooling.tool_dia_mm       = 6.0;
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 4;
    tooling.cutting_speed_sfm = 350.0;
    tooling.feed_per_tooth_mm = 0.05;
    tooling.stock_removed_mm3 = sx * sy * absDist;
    tooling.est_cycle_time_s  = std::max(1.0, absDist / 5.0);

    FeatureSignature sig { kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);  // keep full chain history
    wpNew->addFeature(sig);

    spdlog::debug("skill::move_face_along_direction: face={} dist={} dir=({},{},{})",
                  in.face_id, in.distance_mm, dir.X(), dir.Y(), dir.Z());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Geometric heuristic: any single solid that has at least one planar face
// pair separated by an axial offset — too generic to deduplicate.  We
// instead replay from the FeatureSignature history: if any prior signature
// has skill_id == kSkillId, surface it.

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
    return out;
}

}  // namespace koocadcam::skill::move_face_along_direction
