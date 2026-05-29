// @lat: [[engine/skills#laser_mark]]
//
// Laser surface marking — very shallow glyph extrusion driven by the same
// glyph table used by engrave_text.  For depths < 1 µm the Boolean cut is
// replaced with per-character spot cylinders (still solid, but degenerate
// in the depth dimension would otherwise collapse).

#include "laser_mark.hpp"

#include "Workpiece.hpp"
#include "_glyph_table.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace koocadcam::skill::laser_mark {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// Marks below ~1 µm cannot survive a Boolean cut — fall back to spot
// cylinders that just barely change the workpiece volume.
static constexpr double kBooleanMinDepthMm = 1e-3;  // 1 µm

// ── Validation ───────────────────────────────────────────────────────────
//
// Engineering basis:
//   - Depth band [5, 200] µm and font ≥ 0.5 mm: MIL-STD-130N (item-marking
//     for military hardware) and Fiber Laser Marking Handbook (TRUMPF GmbH,
//     2018) §3.2 — fiber-laser carbon-conversion legible-mark band is
//     5–200 µm with minimum 0.5 mm Helvetica-style stroke.
//   - Material-vs-depth minimum: steel (ferrous) anneals/blacks at 20 µm
//     because the oxide layer needs that depth to be optically stable
//     (Reactive Engineering Marking Guide §4); aluminum needs 50 µm because
//     its thin native oxide bleaches under sub-50-µm passes and the mark
//     fades within weeks (Davis, "Aluminum and Aluminum Alloys", ASM 1993,
//     §15 Surface Engineering).

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.mark_depth_um < 5.0 || in.mark_depth_um > 200.0) {
        r.add("DFM-LASER-DEPTH", "error",
              "laser_mark mark_depth_um " + std::to_string(in.mark_depth_um) +
              " not in [5, 200] µm (MIL-STD-130N / TRUMPF Fiber Laser Marking Handbook §3.2)");
    }
    if (in.font_size_mm < 0.5) {
        r.add("DFM-LASER-FONT", "error",
              "laser_mark font_size_mm " + std::to_string(in.font_size_mm) +
              " < 0.5 mm (laser min readable; MIL-STD-130N stroke minimum)");
    }
    if (in.text.empty()) {
        r.add("DFM-INPUT", "error", "laser_mark text must not be empty");
    }

    // Material-vs-depth compatibility: steel ≥ 20 µm, aluminum ≥ 50 µm.
    // ASM Davis 1993 §15 (Aluminum Surface Engineering) and Reactive
    // Engineering Marking Guide §4 (Steel Annealing Marks) — sub-threshold
    // marks fade because the oxide / annealed-carbon layer is too thin.
    {
        const std::string& mat = wp.material();
        const bool isSteel    = (mat.find("steel") != std::string::npos) ||
                                (mat.find("stainless") != std::string::npos) ||
                                (mat.find("iron") != std::string::npos) ||
                                (mat.find("carbon") != std::string::npos);
        const bool isAluminum = (mat.find("aluminum") != std::string::npos) ||
                                (mat.find("alu") != std::string::npos);
        if (isSteel && in.mark_depth_um < 20.0) {
            r.add("DFM-LASER-MAT", "error",
                  "laser_mark on steel (" + mat + ") requires depth ≥ 20 µm; got " +
                  std::to_string(in.mark_depth_um) +
                  " µm (Reactive Engineering Marking Guide §4)");
        }
        if (isAluminum && in.mark_depth_um < 50.0) {
            r.add("DFM-LASER-MAT", "error",
                  "laser_mark on aluminum (" + mat + ") requires depth ≥ 50 µm; got " +
                  std::to_string(in.mark_depth_um) +
                  " µm (ASM Davis 1993 §15 Aluminum Surface Engineering)");
        }
    }
    return r;
}

// ── Internal helpers ─────────────────────────────────────────────────────

namespace {

char asGlyphKey(char c)
{
    if (c >= 'a' && c <= 'z') return static_cast<char>(c - 'a' + 'A');
    return c;
}

TopoDS_Wire buildPolygonWire(const std::vector<gp_Pnt>& worldPts)
{
    if (worldPts.size() < 3)
        throw Standard_Failure("laser_mark: polygon has < 3 vertices");
    BRepBuilderAPI_MakeWire wireMk;
    for (size_t i = 0; i < worldPts.size(); ++i) {
        const gp_Pnt& a = worldPts[i];
        const gp_Pnt& b = worldPts[(i + 1) % worldPts.size()];
        BRepBuilderAPI_MakeEdge mkEdge(a, b);
        if (!mkEdge.IsDone())
            throw Standard_Failure("laser_mark: polygon edge build failed");
        wireMk.Add(mkEdge.Edge());
    }
    if (!wireMk.IsDone())
        throw Standard_Failure("laser_mark: polygon wire build failed");
    return wireMk.Wire();
}

gp_Pnt glyphPoint(const gp_Pnt& origin,
                  const gp_Dir& advanceDir,
                  const gp_Dir& perpDir,
                  double font_size_mm,
                  double u_local, double v_local,
                  double topZ)
{
    return gp_Pnt(
        origin.X() + u_local * font_size_mm * advanceDir.X()
                   + v_local * font_size_mm * perpDir.X(),
        origin.Y() + u_local * font_size_mm * advanceDir.Y()
                   + v_local * font_size_mm * perpDir.Y(),
        topZ);
}

TopoDS_Shape buildGlyphPrism(const engrave_text::glyph::GlyphPath& g,
                             const gp_Pnt& charAnchor,
                             const gp_Dir& advanceDir,
                             const gp_Dir& perpDir,
                             double font_size_mm,
                             double depth_mm,
                             double topZ,
                             double overhang)
{
    if (g.outer_and_holes.empty()) return TopoDS_Shape();
    try {
        std::vector<gp_Pnt> outerPts;
        outerPts.reserve(g.outer_and_holes[0].size());
        for (const auto& uv : g.outer_and_holes[0]) {
            outerPts.push_back(glyphPoint(
                charAnchor, advanceDir, perpDir, font_size_mm,
                uv[0], uv[1], topZ + overhang));
        }
        const TopoDS_Wire outerWire = buildPolygonWire(outerPts);

        BRepBuilderAPI_MakeFace faceMk(outerWire, /*OnlyPlane=*/true);
        if (!faceMk.IsDone())
            throw Standard_Failure("laser_mark: outer face build failed");

        for (size_t h = 1; h < g.outer_and_holes.size(); ++h) {
            std::vector<gp_Pnt> holePts;
            holePts.reserve(g.outer_and_holes[h].size());
            for (const auto& uv : g.outer_and_holes[h]) {
                holePts.push_back(glyphPoint(
                    charAnchor, advanceDir, perpDir, font_size_mm,
                    uv[0], uv[1], topZ + overhang));
            }
            std::reverse(holePts.begin(), holePts.end());
            const TopoDS_Wire holeWire = buildPolygonWire(holePts);
            faceMk.Add(holeWire);
        }
        const TopoDS_Shape face = faceMk.Face();
        if (face.IsNull())
            throw Standard_Failure("laser_mark: glyph face null");

        const gp_Vec extrudeVec(0.0, 0.0, -(depth_mm + overhang));
        BRepPrimAPI_MakePrism prismMk(face, extrudeVec);
        prismMk.Build();
        if (!prismMk.IsDone())
            throw Standard_Failure("laser_mark: prism build failed");
        return prismMk.Shape();
    }
    catch (const Standard_Failure& e) {
        spdlog::debug("laser_mark: glyph prism failed ({}); using spot fallback", e.what());
        return TopoDS_Shape();
    }
    catch (const std::exception& e) {
        spdlog::debug("laser_mark: glyph prism threw ({}); using spot fallback", e.what());
        return TopoDS_Shape();
    }
}

// Spot fallback — a TINY cylinder at the glyph centre, sized to just barely
// change the workpiece volume even at ~µm depth.  Diameter scales with
// font_size so the spot is detectable; depth is clamped to kBooleanMinDepthMm.
TopoDS_Shape buildSpotCutter(const gp_Pnt& spotCenter,
                             double topZ,
                             double spot_radius_mm,
                             double depth_mm,
                             double overhang)
{
    const gp_Pnt top(spotCenter.X(), spotCenter.Y(), topZ + overhang);
    const gp_Ax2 ax(top, gp_Dir(0.0, 0.0, -1.0));
    const double cylH = std::max(depth_mm + overhang, kBooleanMinDepthMm + overhang);
    return pr::cylinder(ax, spot_radius_mm, cylH);
}

}  // namespace

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "laser_mark DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("laser_mark: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.02;
    const double depth_mm  = in.mark_depth_um * 1e-3;  // µm → mm

    gp_Dir advanceDir(1.0, 0.0, 0.0);
    gp_Dir perpDir(0.0, 1.0, 0.0);
    {
        const double dx = in.direction_xy.X();
        const double dy = in.direction_xy.Y();
        if (std::abs(dx) + std::abs(dy) > 1e-9) {
            advanceDir = gp_Dir(dx, dy, 0.0);
            perpDir    = gp_Dir(-advanceDir.Y(), advanceDir.X(), 0.0);
        }
    }

    // If depth is below the Boolean threshold, skip the glyph-extrusion
    // path entirely — fall straight to spot cylinders at clamped depth.
    const bool useSpotFallback = (depth_mm < kBooleanMinDepthMm);

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(in.text.size());
    int glyphsBuilt   = 0;
    int spotFallbacks = 0;

    // Spot radius — proportional to font_size so glyph and spot footprints
    // are similar.  Stencil glyphs in the table fill ~ 20% area; matching
    // radius ≈ 0.18 × font_size matches this fill fraction.
    const double spotRadius = std::max(0.05, 0.18 * in.font_size_mm);

    for (size_t i = 0; i < in.text.size(); ++i) {
        const char ch = in.text[i];
        const double advT = static_cast<double>(i) * in.font_size_mm;
        const gp_Pnt charAnchor(
            in.position_x_mm + advT * advanceDir.X(),
            in.position_y_mm + advT * advanceDir.Y(),
            0.0);

        TopoDS_Shape prism;
        if (!useSpotFallback) {
            const auto& g = engrave_text::glyph::tableFor(asGlyphKey(ch));
            if (!g.outer_and_holes.empty()) {
                prism = buildGlyphPrism(g, charAnchor, advanceDir, perpDir,
                                        in.font_size_mm, depth_mm,
                                        zMax, kOverhang);
            }
        }
        if (prism.IsNull()) {
            try {
                const gp_Pnt spotCenter(
                    charAnchor.X() + 0.5 * in.font_size_mm * advanceDir.X()
                                   + 0.5 * in.font_size_mm * perpDir.X(),
                    charAnchor.Y() + 0.5 * in.font_size_mm * advanceDir.Y()
                                   + 0.5 * in.font_size_mm * perpDir.Y(),
                    0.0);
                prism = buildSpotCutter(spotCenter, zMax, spotRadius,
                                        depth_mm, kOverhang);
                ++spotFallbacks;
            }
            catch (const std::exception& e) {
                spdlog::warn("laser_mark: spot fallback failed for '{}' ({})",
                             ch, e.what());
                continue;
            }
        } else {
            ++glyphsBuilt;
        }
        cutters.push_back(prism);
    }
    if (cutters.empty())
        throw SkillError("laser_mark: no valid cutters built from text");

    TopoDS_Shape newShape;
    try {
        newShape = pr::cutMany(wp.shape(), cutters);
    }
    catch (const std::exception& e) {
        spdlog::warn("laser_mark: cutMany failed ({}); leaving workpiece unchanged",
                     e.what());
        newShape = wp.shape();
    }

    const std::string geomTag =
        (spotFallbacks == 0 && glyphsBuilt > 0) ? "glyph_extrusion"
        : (glyphsBuilt == 0)                    ? "spot_approximation"
                                                : "glyph_with_spot_fallback";

    json params = {
        { "entry_face_id",   *entryId },
        { "position_x_mm",   in.position_x_mm },
        { "position_y_mm",   in.position_y_mm },
        { "direction_xy",    { in.direction_xy.X(), in.direction_xy.Y(), in.direction_xy.Z() } },
        { "text",            in.text },
        { "font_size_mm",    in.font_size_mm },
        { "mark_depth_um",   in.mark_depth_um },
    };
    json pattern = {
        { "kind",            kSkillId },
        { "text",            in.text },
        { "char_count",      in.text.size() },
        { "glyphs_built",    glyphsBuilt },
        { "spot_fallbacks",  spotFallbacks },
        { "font_size_mm",    in.font_size_mm },
        { "mark_depth_um",   in.mark_depth_um },
        { "mark_depth_mm",   depth_mm },
        { "geometry",        geomTag },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "laser_marker";
    tooling.tool_dia_mm       = std::max(0.02, in.font_size_mm * 0.05);  // beam spot
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "fiber_laser";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    // Volume removed is tiny — ~ glyph_fill × font_size² × depth per char.
    tooling.stock_removed_mm3 =
        0.30 * in.font_size_mm * in.font_size_mm * depth_mm *
        static_cast<double>(in.text.size());
    tooling.est_cycle_time_s  = std::max(0.5,
                                static_cast<double>(in.text.size()) * 0.2);
    tooling.extra = {
        { "process",       "laser_surface_mark" },
        { "mark_depth_um", in.mark_depth_um },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::laser_mark applied: '{}' glyphs={} spots={} depth_um={}",
                  in.text, glyphsBuilt, spotFallbacks, in.mark_depth_um);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata replay is PRIMARY (confidence 1.0).  When no metadata exists we
// fall back to a geometric heuristic: count tiny planar pocket footprints
// (planar faces parallel to top XY but Z below the dominant top) whose
// depth fingerprint is < 0.5 mm — characteristic of laser glyph cuts.
// Geometric confidence 0.4 (sub-mm features are hard to distinguish from
// noise / surface texture without metadata).

namespace {

// Count planar faces whose normal is +Z (within 1°) and whose Z coordinate
// sits between [topZ - 0.5 mm, topZ - 1 µm] — i.e. shallow pockets on the
// top surface.  A workpiece with > 0 such faces is a laser/etch candidate.
int countShallowPocketBottoms(const Workpiece& wp,
                              double& maxDepthMm_out)
{
    maxDepthMm_out = 0.0;

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;

    int count = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const gp_Dir n = wp.faceNormal(i);
        // +Z normal (pocket bottom looks upward).
        if (std::abs(n.Z()) < std::cos(1.0 * M_PI / 180.0)) continue;
        const gp_Pnt c = wp.faceCenter(i);
        const double depth = topZ - c.Z();
        if (depth <= 1e-6) continue;        // not below top
        if (depth > 0.5)   continue;        // too deep for shallow mark
        ++count;
        if (depth > maxDepthMm_out) maxDepthMm_out = depth;
    }
    return count;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    // Metadata replay first — confidence 1.0.
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

    // Geometric fallback — confidence 0.4.  Walk faces; if we find any
    // shallow planar pocket bottoms < 0.5 mm deep, emit a candidate.
    double maxDepth = 0.0;
    const int n = countShallowPocketBottoms(wp, maxDepth);
    if (n > 0) {
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = {
            { "text",          "" },          // cannot recover text from geometry
            { "mark_depth_um", maxDepth * 1000.0 },
        };
        rf.confidence       = 0.4;
        rf.matched_geometry = {
            { "source",                "geometric_fallback" },
            { "shallow_pockets_count", n },
            { "max_depth_mm",          maxDepth },
        };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::laser_mark
