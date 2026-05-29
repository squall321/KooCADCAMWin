// @lat: [[engine/skills#top_face_recess_with_walls]]

#include "top_face_recess_with_walls.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Fillets.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

namespace koocadcam::skill::top_face_recess_with_walls {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec library ─────────────────────────────────────────────────────────

namespace {

struct RecessSpec {
    const char* name;
    double      wall_thickness_mm;
    double      wall_height_mm;
    double      fillet_r_mm;
};

constexpr std::array<RecessSpec, 4> kRecessTable {{
    { "small_pcb",   1.0, 2.0, 0.5 },
    { "large_pcb",   1.5, 3.0, 1.0 },
    { "gasket_seat", 2.0, 1.5, 0.5 },
    { "lid_seat",    1.5, 4.0, 1.0 },
}};

const RecessSpec* findSpec(const std::string& name)
{
    for (const auto& s : kRecessTable) {
        if (name == s.name) return &s;
    }
    return nullptr;
}

}  // namespace

double specWallThicknessFor(const std::string& s) { const auto* x = findSpec(s); return x ? x->wall_thickness_mm : 0.0; }
double specWallHeightFor   (const std::string& s) { const auto* x = findSpec(s); return x ? x->wall_height_mm    : 0.0; }
double specFilletRFor      (const std::string& s) { const auto* x = findSpec(s); return x ? x->fillet_r_mm       : 0.0; }

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const auto* spec = findSpec(in.spec_class);
    if (!spec) {
        r.add("DFM-RECESS-UNKNOWN", "error",
              "top_face_recess_with_walls: unknown spec_class '" + in.spec_class +
              "' (supported: small_pcb/large_pcb/gasket_seat/lid_seat)");
        return r;
    }

    if (spec->wall_thickness_mm < 0.4) {
        r.add("DFM-WALL-MIN", "error",
              "top_face_recess_with_walls: spec wall_thickness " +
              std::to_string(spec->wall_thickness_mm) +
              " mm < 0.4 mm mould-rule minimum");
    }

    if (in.inner_length_mm <= 0.0 || in.inner_width_mm <= 0.0) {
        r.add("DFM-RECESS-INNER", "error",
              "top_face_recess_with_walls: inner_length and inner_width must be > 0");
    }
    if (in.recess_depth_mm < 0.0) {
        r.add("DFM-RECESS-DEPTH", "error",
              "top_face_recess_with_walls: recess_depth must be ≥ 0");
    }

    if (spec->fillet_r_mm > spec->wall_thickness_mm + 1e-9) {
        r.add("DFM-RECESS-FILLET", "error",
              "top_face_recess_with_walls: fillet R " +
              std::to_string(spec->fillet_r_mm) +
              " mm > wall_thickness " +
              std::to_string(spec->wall_thickness_mm) + " mm");
    }

    // Auto-context: verify the recess footprint fits inside the workpiece
    // entry-face bbox.  Workpiece-context awareness.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double outerL = in.inner_length_mm + 2.0 * spec->wall_thickness_mm;
    const double outerW = in.inner_width_mm  + 2.0 * spec->wall_thickness_mm;
    if (in.center_x_mm - outerL / 2.0 < xMin - 1e-6 ||
        in.center_x_mm + outerL / 2.0 > xMax + 1e-6 ||
        in.center_y_mm - outerW / 2.0 < yMin - 1e-6 ||
        in.center_y_mm + outerW / 2.0 > yMax + 1e-6) {
        r.add("DFM-RECESS-FIT", "error",
              "top_face_recess_with_walls: outer wall footprint " +
              std::to_string(outerL) + "x" + std::to_string(outerW) +
              " mm exceeds workpiece bbox");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "top_face_recess_with_walls DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = findSpec(in.spec_class);
    const double T  = spec->wall_thickness_mm;
    const double H  = spec->wall_height_mm;
    const double R  = spec->fillet_r_mm;

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("top_face_recess_with_walls: entry_face datum unresolved");

    BRepAdaptor_Surface entrySurf(wp.face(*entryId));
    if (entrySurf.GetType() != GeomAbs_Plane) {
        throw SkillError("top_face_recess_with_walls: entry_face must be planar");
    }
    const gp_Pln entryPlane = entrySurf.Plane();
    gp_Dir outward = entryPlane.Axis().Direction();
    if (wp.face(*entryId).Orientation() == TopAbs_REVERSED) outward.Reverse();

    // For slice 1 we support only the world-Z-up case (entry face normal ≈ +Z).
    if (outward.Z() < 0.95) {
        throw SkillError("top_face_recess_with_walls: only entry faces with +Z normal supported in slice 1");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;

    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;
    const double IL = in.inner_length_mm;
    const double IW = in.inner_width_mm;
    constexpr double kEmbed = 0.05;

    // ── Step 1: cut the recess (pocket). ────────────────────────────────
    TopoDS_Shape current = wp.shape();
    if (in.recess_depth_mm > 1e-6) {
        const gp_Pnt cutOrigin(cx - IL / 2.0,
                               cy - IW / 2.0,
                               topZ - in.recess_depth_mm);
        const TopoDS_Shape pocketTool =
            pr::box(gp_Ax2(cutOrigin, gp::DZ()), IL, IW, in.recess_depth_mm + kEmbed);
        try {
            current = pr::cut(current, pocketTool);
        } catch (const Standard_Failure& e) {
            throw SkillError(std::string("top_face_recess_with_walls: pocket cut failed: ") +
                             e.what());
        }
    }

    // ── Steps 2–4: fuse the 4 walls (outer ring). ───────────────────────
    //   +X wall  spans X ∈ [cx+IL/2, cx+IL/2+T], Y ∈ [cy-IW/2-T, cy+IW/2+T]
    //   −X wall  spans X ∈ [cx-IL/2-T, cx-IL/2],  same Y span
    //   +Y wall  spans Y ∈ [cy+IW/2, cy+IW/2+T], X ∈ [cx-IL/2, cx+IL/2]
    //   −Y wall  spans Y ∈ [cy-IW/2-T, cy-IW/2],  same X span
    //
    // Walls go from Z = topZ - kEmbed (slightly embedded into the workpiece)
    // up to Z = topZ + H.
    const double zBot = topZ - kEmbed;
    const double zTotalH = H + kEmbed;

    auto makeWall = [&](double x0, double y0, double dx, double dy) {
        const gp_Pnt origin(x0, y0, zBot);
        return pr::box(gp_Ax2(origin, gp::DZ()), dx, dy, zTotalH);
    };

    const TopoDS_Shape wXpos = makeWall(cx + IL / 2.0,         cy - IW / 2.0 - T,
                                        T,                     IW + 2.0 * T);
    const TopoDS_Shape wXneg = makeWall(cx - IL / 2.0 - T,     cy - IW / 2.0 - T,
                                        T,                     IW + 2.0 * T);
    const TopoDS_Shape wYpos = makeWall(cx - IL / 2.0,         cy + IW / 2.0,
                                        IL,                    T);
    const TopoDS_Shape wYneg = makeWall(cx - IL / 2.0,         cy - IW / 2.0 - T,
                                        IL,                    T);

    try {
        current = pr::fuseMany(current, { wXpos, wXneg, wYpos, wYneg });
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("top_face_recess_with_walls: wall fuse failed: ") +
                         e.what());
    }

    // ── Step 5: fillet the 4 outer-top corner edges (top-rim ring) ──────
    TopoDS_Shape finalShape = current;
    if (R > 0.0) {
        const double topWallZ  = topZ + H;
        try {
            finalShape = pr::filletEdges(current, R, pr::edgesAtZ(topWallZ, 1e-2));
        } catch (const Standard_Failure&) {
            // Some OCCT builds can't fillet the full top rim; keep the
            // un-filleted shape rather than aborting.
            finalShape = current;
        }
    }

    // ── Signature ────────────────────────────────────────────────────────
    json params = {
        { "entry_face_id",    *entryId },
        { "spec_class",       in.spec_class },
        { "center_x_mm",      cx },
        { "center_y_mm",      cy },
        { "inner_length_mm",  IL },
        { "inner_width_mm",   IW },
        { "recess_depth_mm",  in.recess_depth_mm },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    5 },
        { "spec_class",          in.spec_class },
        { "wall_thickness_mm",   T },
        { "wall_height_mm",      H },
        { "fillet_r_mm",         R },
        { "inner_length_mm",     IL },
        { "inner_width_mm",      IW },
        { "outer_length_mm",     IL + 2.0 * T },
        { "outer_width_mm",      IW + 2.0 * T },
        { "recess_depth_mm",     in.recess_depth_mm },
        { "wall_top_z",          topZ + H },
        { "recess_floor_z",      topZ - in.recess_depth_mm },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "compound_recess_wall";
    tooling.tool_dia_mm      = 0.0;
    tooling.tool_length_mm   = 0.0;
    tooling.tool_material    = "n/a";
    tooling.flute_count      = 0;
    const double pocketVol = IL * IW * in.recess_depth_mm;
    const double wallVol   = 2.0 * (IW + 2.0 * T) * T * H + 2.0 * IL * T * H;
    tooling.stock_removed_mm3 = pocketVol - wallVol;   // can be ±
    tooling.extra = {
        { "compound_chain",
          "cut_pocket → fuse_wall_+X → fuse_wall_-X → fuse_wall_±Y(many) → fillet_corners" },
        { "spec_class",        in.spec_class },
        { "wall_thickness_mm", T },
        { "wall_height_mm",    H },
        { "fillet_r_mm",       R },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(finalShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::top_face_recess_with_walls applied: spec='{}' inner={}x{} depth={} faces {}→{}",
                  in.spec_class, IL, IW, in.recess_depth_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Primary: replay from feature history.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "spec_class",      f.params.value("spec_class",      std::string()) },
            { "center_x_mm",     f.params.value("center_x_mm",     0.0) },
            { "center_y_mm",     f.params.value("center_y_mm",     0.0) },
            { "inner_length_mm", f.params.value("inner_length_mm", 0.0) },
            { "inner_width_mm",  f.params.value("inner_width_mm",  0.0) },
            { "recess_depth_mm", f.params.value("recess_depth_mm", 0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, /*confidence*/ 0.92,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback: look for a downward-facing planar floor (pocket)
    // and at least one upward-facing planar face above the workpiece's
    // original top — that indicates a wall ring.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    int floorId = -1;
    double floorZ = zMax;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() > -0.95) continue;             // looking for normal ≈ -Z
        const gp_Pnt c = wp.faceCenter(i);
        if (c.Z() >= zMax - 1e-3) continue;       // must be below top
        if (c.Z() < floorZ) { floorZ = c.Z(); floorId = i; }
    }

    // Find wall-top faces above zMax of the "main" body — heuristic only.
    std::vector<int> wallTops;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.95) continue;
        const gp_Pnt c = wp.faceCenter(i);
        if (c.Z() <= zMax - 1e-3) continue;
        wallTops.push_back(i);
    }

    if (floorId < 0 && wallTops.empty()) return out;

    const auto floorBb = (floorId >= 0) ? pr::optimalBbox(wp.face(floorId))
                                        : pr::Bbox3d{};
    json rec = {
        { "spec_class",      "large_pcb" },
        { "center_x_mm",     (floorBb.xMin + floorBb.xMax) / 2.0 },
        { "center_y_mm",     (floorBb.yMin + floorBb.yMax) / 2.0 },
        { "inner_length_mm", std::max(0.0, floorBb.dx()) },
        { "inner_width_mm",  std::max(0.0, floorBb.dy()) },
        { "recess_depth_mm", zMax - floorZ },
    };
    json matched = {
        { "floor_face_id",  floorId },
        { "wall_top_count", static_cast<int>(wallTops.size()) },
    };
    out.push_back(RecognizedFeature{ kSkillId, rec, /*confidence*/ 0.7, matched });
    return out;
}

}  // namespace koocadcam::skill::top_face_recess_with_walls
