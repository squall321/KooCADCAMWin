// @lat: [[engine/skills#curved_lip_around_face]]

#include "curved_lip_around_face.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
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

namespace koocadcam::skill::curved_lip_around_face {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec library ─────────────────────────────────────────────────────────

namespace {

struct LipSpec {
    const char* name;
    double      inset_mm;
    double      thickness_mm;
    double      height_mm;
};

constexpr std::array<LipSpec, 4> kLipTable {{
    { "bezel_lip",       1.0, 0.8, 0.5 },
    { "cover_lip",       2.0, 1.5, 2.0 },
    { "seal_groove_lip", 3.0, 1.0, 1.5 },
    { "flange_lip",      5.0, 2.5, 3.0 },
}};

const LipSpec* findSpec(const std::string& name)
{
    for (const auto& s : kLipTable) {
        if (name == s.name) return &s;
    }
    return nullptr;
}

// Context: query the workpiece for the entry face's planar bbox.
// Returns false if the face isn't planar with +Z normal or is too small.
struct FaceContext {
    double xMin, yMin, xMax, yMax, z;
    double L() const { return xMax - xMin; }
    double W() const { return yMax - yMin; }
};

bool extractFaceContext(const Workpiece& wp, int faceId, FaceContext& out)
{
    if (faceId < 0 || faceId >= wp.faceCount()) return false;
    if (!wp.isFacePlanar(faceId)) return false;
    gp_Dir n;
    try { n = wp.faceNormal(faceId); } catch (...) { return false; }
    if (n.Z() < 0.95) return false;
    const auto fb = pr::optimalBbox(wp.face(faceId));
    if (fb.dx() < 0.5 || fb.dy() < 0.5) return false;
    out.xMin = fb.xMin; out.xMax = fb.xMax;
    out.yMin = fb.yMin; out.yMax = fb.yMax;
    out.z    = wp.faceCenter(faceId).Z();
    return true;
}

}  // namespace

double specInsetFor    (const std::string& s) { const auto* x = findSpec(s); return x ? x->inset_mm     : 0.0; }
double specThicknessFor(const std::string& s) { const auto* x = findSpec(s); return x ? x->thickness_mm : 0.0; }
double specHeightFor   (const std::string& s) { const auto* x = findSpec(s); return x ? x->height_mm    : 0.0; }

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const auto* spec = findSpec(in.spec_class);
    if (!spec) {
        r.add("DFM-LIP-UNKNOWN", "error",
              "curved_lip_around_face: unknown spec_class '" + in.spec_class +
              "' (supported: bezel_lip/cover_lip/seal_groove_lip/flange_lip)");
        return r;
    }

    if (spec->thickness_mm < 0.4) {
        r.add("DFM-WALL-MIN", "error",
              "curved_lip_around_face: spec thickness " +
              std::to_string(spec->thickness_mm) +
              " mm < 0.4 mm mould-rule minimum");
    }

    const double ratio = spec->height_mm / std::max(1e-6, spec->thickness_mm);
    if (ratio > 4.0) {
        r.add("DFM-LIP-ASPECT", "error",
              "curved_lip_around_face: H/T ratio " + std::to_string(ratio) +
              " > 4 — lip is unstable");
    }

    // Auto-context detection: resolve the entry face and verify its bbox
    // is large enough for the spec geometry.
    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) {
        r.add("DFM-LIP-CONTEXT", "error",
              "curved_lip_around_face: entry_face datum could not be resolved on this workpiece "
              "(auto-detect failure — workpiece is missing the required base face)");
        return r;
    }
    FaceContext ctx;
    if (!extractFaceContext(wp, *entryId, ctx)) {
        r.add("DFM-LIP-CONTEXT", "error",
              "curved_lip_around_face: entry face " + std::to_string(*entryId) +
              " is not planar with +Z normal or is too small "
              "(auto-detect failure)");
        return r;
    }
    const double minDim = std::min(ctx.L(), ctx.W());
    const double consumed = 2.0 * (spec->inset_mm + spec->thickness_mm);
    if (consumed >= minDim) {
        r.add("DFM-LIP-INSET", "error",
              "curved_lip_around_face: 2 × (inset + thickness) = " +
              std::to_string(consumed) + " mm ≥ min(face_L, face_W) = " +
              std::to_string(minDim) + " mm — lip would invert");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "curved_lip_around_face DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = findSpec(in.spec_class);
    const double inset = spec->inset_mm;
    const double T     = spec->thickness_mm;
    const double H     = spec->height_mm;

    auto entryId = wp.resolve(in.entry_face);
    FaceContext ctx;
    extractFaceContext(wp, *entryId, ctx);

    constexpr double kEmbed = 0.05;

    // Outer ring footprint = face bbox shrunk by inset.
    const double oxMin = ctx.xMin + inset;
    const double oyMin = ctx.yMin + inset;
    const double oxMax = ctx.xMax - inset;
    const double oyMax = ctx.yMax - inset;
    // Inner ring footprint = outer shrunk by thickness.
    const double ixMin = oxMin + T;
    const double iyMin = oyMin + T;
    const double ixMax = oxMax - T;
    const double iyMax = oyMax - T;

    const double zBot = ctx.z - kEmbed;
    const double zH   = H + kEmbed;

    // Outer prism.
    const gp_Pnt outerOrigin(oxMin, oyMin, zBot);
    const TopoDS_Shape outerPrism =
        pr::box(gp_Ax2(outerOrigin, gp::DZ()),
                oxMax - oxMin, oyMax - oyMin, zH);
    // Inner prism.
    const gp_Pnt innerOrigin(ixMin, iyMin, zBot);
    const TopoDS_Shape innerPrism =
        pr::box(gp_Ax2(innerOrigin, gp::DZ()),
                ixMax - ixMin, iyMax - iyMin, zH);

    TopoDS_Shape lip;
    try {
        lip = pr::cut(outerPrism, innerPrism);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("curved_lip_around_face: ring build failed: ") +
                         e.what());
    }
    TopoDS_Shape finalShape;
    try {
        finalShape = pr::fuse(wp.shape(), lip);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("curved_lip_around_face: fuse failed: ") +
                         e.what());
    }

    json params = {
        { "entry_face_id", *entryId },
        { "spec_class",    in.spec_class },
    };
    json pattern = {
        { "kind",             kSkillId },
        { "is_compound",      true },
        { "subfeature_count", 6 },             // outer_box, inner_box, cut, fuse + 2 context derives
        { "spec_class",       in.spec_class },
        { "inset_mm",         inset },
        { "thickness_mm",     T },
        { "height_mm",        H },
        { "outer_l_mm",       oxMax - oxMin },
        { "outer_w_mm",       oyMax - oyMin },
        { "inner_l_mm",       ixMax - ixMin },
        { "inner_w_mm",       iyMax - iyMin },
        { "face_l_mm",        ctx.L() },
        { "face_w_mm",        ctx.W() },
        { "face_z",           ctx.z },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "compound_lip";
    tooling.tool_dia_mm      = 0.0;
    tooling.tool_length_mm   = 0.0;
    tooling.tool_material    = "n/a";
    tooling.flute_count      = 0;
    // Ring area × height (added).
    const double ringArea = (oxMax - oxMin) * (oyMax - oyMin) -
                            (ixMax - ixMin) * (iyMax - iyMin);
    tooling.stock_removed_mm3 = -ringArea * H;
    tooling.extra = {
        { "compound_chain",
          "context_face_bbox → outer_prism → inner_prism → cut → fuse" },
        { "spec_class", in.spec_class },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(finalShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::curved_lip_around_face applied: spec='{}' face={}x{} inset={} T={} H={} faces {}→{}",
                  in.spec_class, ctx.L(), ctx.W(), inset, T, H,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "spec_class", f.params.value("spec_class", std::string()) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, /*confidence*/ 0.92,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback: find an inward-facing planar face (the lip's inner
    // wall) at the same Z elevation as an outward-facing pair — the
    // ring topology gives 4 outer planar faces + 4 inner planar faces.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    int upTopCount = 0;
    double topZ = -1e30;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.95) continue;
        const gp_Pnt c = wp.faceCenter(i);
        if (c.Z() <= zMax - 1e-3) continue;
        ++upTopCount;
        if (c.Z() > topZ) topZ = c.Z();
    }
    if (upTopCount < 1) return out;

    // Estimate inset/thickness/height roughly from outermost vs innermost
    // upward-facing-face bboxes near topZ.  Too lossy to reconstruct the
    // exact spec — return a single low-confidence candidate.
    json rec = {
        { "spec_class", "cover_lip" },
    };
    json matched = { { "lip_top_face_count", upTopCount }, { "top_z", topZ } };
    out.push_back(RecognizedFeature{ kSkillId, rec, /*confidence*/ 0.6, matched });
    return out;
}

}  // namespace koocadcam::skill::curved_lip_around_face
