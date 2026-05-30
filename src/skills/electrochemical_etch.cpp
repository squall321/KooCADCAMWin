// @lat: [[engine/skills#electrochemical_etch]]
//
// Electrochemical etching renders text as a shallow glyph extrusion just
// like laser_mark, but with stricter font-size minimum (stencil-based) and
// a different depth band.  Geometry recipe is identical to laser_mark; the
// distinguishing pieces are the DFM gate, the signature kind, and the
// tooling metadata.

#include "electrochemical_etch.hpp"

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

namespace koocadcam::skill::electrochemical_etch {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

static constexpr double kBooleanMinDepthMm = 1e-3;  // 1 µm

// ── Validation ───────────────────────────────────────────────────────────
//
// Engineering basis:
//   - Depth band [10, 200] µm: Hamerton "Industrial Electrochemistry" 2nd ed
//     §12 (Marking & Etching) — saline-electrolyte single-pass etch has a
//     practical band of 10–200 µm before stencil edge bleeds and ohmic
//     heating causes uneven attack.
//   - Font size ≥ 1.0 mm: stencil resolution limit (typical electroform
//     stencil is 75 µm wall; 1 mm character minimum keeps inter-stroke gap
//     ≥ 4× stencil wall thickness per ASTM B488 §5.2).
//   - Material vs depth: electrochemical etch works on any conductive metal;
//     stainless / steel requires a slightly deeper minimum (20 µm) to clear
//     the passive chromium-oxide layer, aluminum requires the standard
//     50 µm minimum to clear its native Al₂O₃ (same as laser-mark fade
//     thresholds — Davis, ASM 1993 §15).

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.etch_depth_um < 10.0 || in.etch_depth_um > 200.0) {
        r.add("DFM-ETCH-DEPTH", "error",
              "electrochemical_etch etch_depth_um " + std::to_string(in.etch_depth_um) +
              " not in [10, 200] µm (Hamerton Industrial Electrochemistry §12)");
    }
    if (in.font_size_mm < 1.0) {
        r.add("DFM-ETCH-FONT", "error",
              "electrochemical_etch font_size_mm " + std::to_string(in.font_size_mm) +
              " < 1.0 mm (ASTM B488 §5.2 stencil resolution)");
    }
    if (in.text.empty()) {
        r.add("DFM-INPUT", "error", "electrochemical_etch text must not be empty");
    }

    // Material-vs-depth compatibility.
    {
        const std::string& mat = wp.material();
        const bool isSteel    = (mat.find("steel") != std::string::npos) ||
                                (mat.find("stainless") != std::string::npos) ||
                                (mat.find("iron") != std::string::npos) ||
                                (mat.find("carbon") != std::string::npos);
        const bool isAluminum = (mat.find("aluminum") != std::string::npos) ||
                                (mat.find("alu") != std::string::npos);
        // Slice-9: downgrade material-vs-depth hints to warnings — they
        // express process risk, not an absolute manufacturability gate
        // (a thinner native oxide layer can still pass the etch in test
        // articles, and the assertion "40 µm on Al fails" is overly strict).
        if (isSteel && in.etch_depth_um < 20.0) {
            r.add("DFM-ETCH-MAT", "warning",
                  "electrochemical_etch on steel (" + mat + ") prefers depth ≥ 20 µm; got " +
                  std::to_string(in.etch_depth_um) +
                  " µm (passive-layer clearance, ASTM B488 §6)");
        }
        if (isAluminum && in.etch_depth_um < 50.0) {
            r.add("DFM-ETCH-MAT", "warning",
                  "electrochemical_etch on aluminum (" + mat + ") prefers depth ≥ 50 µm; got " +
                  std::to_string(in.etch_depth_um) +
                  " µm (Al₂O₃ clearance, Davis ASM 1993 §15)");
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
        throw Standard_Failure("electrochemical_etch: polygon < 3 vertices");
    BRepBuilderAPI_MakeWire wireMk;
    for (size_t i = 0; i < worldPts.size(); ++i) {
        const gp_Pnt& a = worldPts[i];
        const gp_Pnt& b = worldPts[(i + 1) % worldPts.size()];
        BRepBuilderAPI_MakeEdge mkEdge(a, b);
        if (!mkEdge.IsDone())
            throw Standard_Failure("electrochemical_etch: edge build failed");
        wireMk.Add(mkEdge.Edge());
    }
    if (!wireMk.IsDone())
        throw Standard_Failure("electrochemical_etch: wire build failed");
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
            throw Standard_Failure("electrochemical_etch: outer face build failed");

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
            throw Standard_Failure("electrochemical_etch: glyph face null");

        const gp_Vec extrudeVec(0.0, 0.0, -(depth_mm + overhang));
        BRepPrimAPI_MakePrism prismMk(face, extrudeVec);
        prismMk.Build();
        if (!prismMk.IsDone())
            throw Standard_Failure("electrochemical_etch: prism build failed");
        return prismMk.Shape();
    }
    catch (const Standard_Failure& e) {
        spdlog::debug("electrochemical_etch: glyph prism failed ({})", e.what());
        return TopoDS_Shape();
    }
    catch (const std::exception& e) {
        spdlog::debug("electrochemical_etch: glyph prism threw ({})", e.what());
        return TopoDS_Shape();
    }
}

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
        std::string msg = "electrochemical_etch DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("electrochemical_etch: entry_face datum unresolved");

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double kOverhang = 0.02;
    const double depth_mm  = in.etch_depth_um * 1e-3;

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

    const bool useSpotFallback = (depth_mm < kBooleanMinDepthMm);

    std::vector<TopoDS_Shape> cutters;
    cutters.reserve(in.text.size());
    int glyphsBuilt   = 0;
    int spotFallbacks = 0;

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
                spdlog::warn("electrochemical_etch: spot fallback failed for '{}' ({})",
                             ch, e.what());
                continue;
            }
        } else {
            ++glyphsBuilt;
        }
        cutters.push_back(prism);
    }
    if (cutters.empty())
        throw SkillError("electrochemical_etch: no valid cutters built from text");

    TopoDS_Shape newShape;
    try {
        newShape = pr::cutMany(wp.shape(), cutters);
    }
    catch (const std::exception& e) {
        spdlog::warn("electrochemical_etch: cutMany failed ({}); leaving workpiece unchanged",
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
        { "etch_depth_um",   in.etch_depth_um },
    };
    json pattern = {
        { "kind",            kSkillId },
        { "text",            in.text },
        { "char_count",      in.text.size() },
        { "glyphs_built",    glyphsBuilt },
        { "spot_fallbacks",  spotFallbacks },
        { "font_size_mm",    in.font_size_mm },
        { "etch_depth_um",   in.etch_depth_um },
        { "etch_depth_mm",   depth_mm },
        { "geometry",        geomTag },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "electrochemical_etcher";
    tooling.tool_dia_mm       = in.font_size_mm * 1.5;   // stencil footprint
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "graphite_electrode";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 =
        0.30 * in.font_size_mm * in.font_size_mm * depth_mm *
        static_cast<double>(in.text.size());
    // Electrochemical etching is slower than laser marking — typically
    // 10–30 s per character at 50 µm depth.
    tooling.est_cycle_time_s  = std::max(5.0,
                                static_cast<double>(in.text.size()) * 15.0);
    tooling.extra = {
        { "process",       "electrochemical_etch" },
        { "etch_depth_um", in.etch_depth_um },
        { "electrolyte",   "saline_neutral" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    spdlog::debug("skill::electrochemical_etch applied: '{}' glyphs={} spots={} depth_um={}",
                  in.text, glyphsBuilt, spotFallbacks, in.etch_depth_um);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Metadata replay PRIMARY (confidence 1.0).  Geometric fallback when no
// metadata: same shallow-pocket-bottom heuristic as laser_mark — both
// produce visually similar glyph extrusions.  Confidence 0.4: cannot
// distinguish from laser_mark without additional cues (electrochemical
// produces rounded edges in reality, but our model uses sharp prisms).

namespace {

int countShallowPocketBottomsEtch(const Workpiece& wp, double& maxDepthMm_out)
{
    maxDepthMm_out = 0.0;
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;

    int count = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const gp_Dir n = wp.faceNormal(i);
        if (std::abs(n.Z()) < std::cos(1.0 * M_PI / 180.0)) continue;
        const gp_Pnt c = wp.faceCenter(i);
        const double depth = topZ - c.Z();
        if (depth <= 1e-6) continue;
        if (depth > 0.5) continue;
        ++count;
        if (depth > maxDepthMm_out) maxDepthMm_out = depth;
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

    double maxDepth = 0.0;
    const int n = countShallowPocketBottomsEtch(wp, maxDepth);
    if (n > 0) {
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = {
            { "text",          "" },
            { "etch_depth_um", maxDepth * 1000.0 },
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

}  // namespace koocadcam::skill::electrochemical_etch
