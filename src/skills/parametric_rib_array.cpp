// @lat: [[engine/skills#parametric_rib_array]]

#include "parametric_rib_array.hpp"

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
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::parametric_rib_array {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec library ─────────────────────────────────────────────────────────
//
// Industry-typical rib geometry presets.  These map an intent name to the
// recommended thickness, the maximum height/thickness ratio before stability
// degrades, and a default draft angle for mould release.

namespace {

struct RibSpec {
    const char* name;
    double      thickness_mm;
    double      max_ht_ratio;
    double      draft_deg;
};

constexpr std::array<RibSpec, 5> kRibTable {{
    { "mold_thin",     0.8, 3.0, 1.0 },
    { "mold_standard", 1.2, 3.0, 1.0 },
    { "mold_heavy",    2.0, 4.0, 1.0 },
    { "structural",    3.0, 5.0, 0.5 },
    { "air_flow_fin",  0.6, 8.0, 0.0 },
}};

const RibSpec* findSpec(const std::string& name)
{
    for (const auto& s : kRibTable) {
        if (name == s.name) return &s;
    }
    return nullptr;
}

}  // namespace

double specThicknessFor(const std::string& spec_class)
{
    const auto* s = findSpec(spec_class);
    return s ? s->thickness_mm : 0.0;
}

double specMaxHTRatioFor(const std::string& spec_class)
{
    const auto* s = findSpec(spec_class);
    return s ? s->max_ht_ratio : 0.0;
}

double specDraftDegFor(const std::string& spec_class)
{
    const auto* s = findSpec(spec_class);
    return s ? s->draft_deg : 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const auto* spec = findSpec(in.spec_class);
    if (!spec) {
        r.add("DFM-RIB-UNKNOWN", "error",
              "parametric_rib_array: unknown spec_class '" + in.spec_class +
              "' (supported: mold_thin/mold_standard/mold_heavy/structural/air_flow_fin)");
        return r;
    }

    // Resolve thickness: caller override OR spec default.
    const double thick = (in.thickness_mm > 0.0) ? in.thickness_mm : spec->thickness_mm;

    if (thick < 0.4) {
        r.add("DFM-WALL-MIN", "error",
              "parametric_rib_array: rib thickness " + std::to_string(thick) +
              " mm < 0.4 mm mould-rule minimum");
    }

    if (in.rib_count < 1) {
        r.add("DFM-RIB-COUNT", "error",
              "parametric_rib_array: rib_count must be ≥ 1");
    }

    if (in.pitch_mm <= 0.0 && in.rib_count > 1) {
        r.add("DFM-INPUT", "error",
              "parametric_rib_array: pitch_mm must be > 0 when rib_count > 1");
    }

    if (in.rib_count > 1 && in.pitch_mm < thick * 2.0) {
        r.add("DFM-RIB-SPACING", "error",
              "parametric_rib_array: pitch " + std::to_string(in.pitch_mm) +
              " mm < 2 × thickness (" + std::to_string(thick * 2.0) +
              " mm) — ribs would overlap or trap molten plastic");
    }

    if (in.length_mm <= 0.0) {
        r.add("DFM-RIB-LENGTH", "error",
              "parametric_rib_array: length_mm must be > 0");
    }

    if (in.height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "parametric_rib_array: height_mm must be > 0");
    } else if (thick > 0.0) {
        const double ratio = in.height_mm / thick;
        if (ratio > spec->max_ht_ratio) {
            r.add("DFM-RIB-ASPECT", "error",
                  "parametric_rib_array: H/T ratio " + std::to_string(ratio) +
                  " > spec max " + std::to_string(spec->max_ht_ratio) +
                  " for class '" + in.spec_class + "'");
        }
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

// Build a single rib box solid using gp_Ax2 framing.
//
// gp_Ax2(P, V, Vx) interprets V = LOCAL Z = extrude direction (rib grows
// "upward" along V), Vx = LOCAL X = along-rib direction (length).  Then
// LOCAL Y = V × Vx is the array-axis direction.  We pass
// (dx=length, dy=thickness, dz=height) to BRepPrimAPI_MakeBox.
//
// origin is placed at the rib's BOTTOM-NEAR-LEFT corner (slightly embedded
// into the workpiece for clean fuse).
TopoDS_Shape buildRibBox(const gp_Pnt& center, const gp_Dir& alongLength,
                         const gp_Dir& alongPitch, const gp_Dir& extrude,
                         double length, double thickness, double height,
                         double embed)
{
    // Local X = alongLength (long dim).
    // Local Z = extrude (height dim).
    // gp_Ax2(P, V, Vx): V = local Z, Vx = local X.  Then BRepPrimAPI_MakeBox
    // produces a box of (dx along Vx, dy along Vy, dz along V).
    //
    // origin = center - half_length·alongLength - half_thickness·alongPitch
    //                 - embed·extrude
    const gp_Pnt origin(
        center.X() - 0.5 * length    * alongLength.X()
                   - 0.5 * thickness * alongPitch.X()
                   - embed           * extrude.X(),
        center.Y() - 0.5 * length    * alongLength.Y()
                   - 0.5 * thickness * alongPitch.Y()
                   - embed           * extrude.Y(),
        center.Z() - 0.5 * length    * alongLength.Z()
                   - 0.5 * thickness * alongPitch.Z()
                   - embed           * extrude.Z());

    const gp_Ax2 ax(origin, extrude, alongLength);
    return pr::box(ax, length, thickness, height + embed);
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "parametric_rib_array DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = findSpec(in.spec_class);
    // spec must be non-null (validate would have errored otherwise).
    const double thick = (in.thickness_mm > 0.0) ? in.thickness_mm : spec->thickness_mm;

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("parametric_rib_array: entry_face datum unresolved");

    // Resolve extrude direction: prefer the entry face outward normal so the
    // ribs grow OUT of the base regardless of caller's axis hint.
    gp_Dir extrude = in.extrude_dir;
    try {
        BRepAdaptor_Surface surf(wp.face(*entryId));
        if (surf.GetType() == GeomAbs_Plane) {
            gp_Dir n = surf.Plane().Axis().Direction();
            if (wp.face(*entryId).Orientation() == TopAbs_REVERSED) n.Reverse();
            extrude = n;
        }
    } catch (...) { /* fall back to caller's extrude_dir */ }

    // Build a local in-plane orthonormal frame:
    //   alongPitch  = projection of in.array_axis onto entry plane
    //   alongLength = extrude × alongPitch
    gp_Vec axisV(in.array_axis.X(), in.array_axis.Y(), in.array_axis.Z());
    // Project axis onto plane perpendicular to extrude.
    gp_Vec extV(extrude.X(), extrude.Y(), extrude.Z());
    axisV = axisV - extV * (axisV.Dot(extV));
    if (axisV.Magnitude() < 1e-9) {
        // Caller's array_axis was parallel to extrude — pick world X
        // projected if possible.
        axisV = gp_Vec(1.0, 0.0, 0.0) - extV * (gp_Vec(1.0, 0.0, 0.0).Dot(extV));
        if (axisV.Magnitude() < 1e-9) {
            axisV = gp_Vec(0.0, 1.0, 0.0) - extV * (gp_Vec(0.0, 1.0, 0.0).Dot(extV));
        }
    }
    axisV.Normalize();
    const gp_Dir alongPitch(axisV.X(), axisV.Y(), axisV.Z());
    gp_Vec lenV = extV.Crossed(axisV);
    lenV.Normalize();
    const gp_Dir alongLength(lenV.X(), lenV.Y(), lenV.Z());

    constexpr double kEmbed = 0.02;  // small embed for clean fuse

    // Compute rib centres, equally spaced on a line through the anchor.
    // Centred array: total span = (N-1) × pitch; first rib at
    // anchor - (N-1)/2 × pitch × alongPitch.
    const int    N        = in.rib_count;
    const double startOff = -((N - 1) / 2.0) * in.pitch_mm;

    // Anchor point projected on entry plane (use anchor XY, project Z onto plane).
    BRepAdaptor_Surface surf2(wp.face(*entryId));
    const gp_Pln entryPlane = surf2.Plane();
    auto projectOntoPlane = [&](double x, double y) -> gp_Pnt {
        const gp_Pnt P0 = entryPlane.Location();
        const gp_Dir n  = entryPlane.Axis().Direction();
        if (std::abs(n.Z()) < 1e-9) {
            return gp_Pnt(x, y, P0.Z());
        }
        const double z = P0.Z() - (n.X() * (x - P0.X()) + n.Y() * (y - P0.Y())) / n.Z();
        return gp_Pnt(x, y, z);
    };
    const gp_Pnt anchorOnFace = projectOntoPlane(in.anchor_x_mm, in.anchor_y_mm);

    // Build ribs sequentially and fuse onto the workpiece.
    TopoDS_Shape current = wp.shape();
    json ribsJson = json::array();
    for (int k = 0; k < N; ++k) {
        const double off = startOff + k * in.pitch_mm;
        const gp_Pnt center(
            anchorOnFace.X() + off * alongPitch.X(),
            anchorOnFace.Y() + off * alongPitch.Y(),
            anchorOnFace.Z() + off * alongPitch.Z());

        TopoDS_Shape ribSolid;
        try {
            ribSolid = buildRibBox(center, alongLength, alongPitch, extrude,
                                   in.length_mm, thick, in.height_mm, kEmbed);
        } catch (const Standard_Failure& e) {
            throw SkillError(std::string("parametric_rib_array: box build failed: ") +
                             e.what());
        }
        try {
            current = pr::fuse(current, ribSolid);
        } catch (const Standard_Failure& e) {
            throw SkillError(std::string("parametric_rib_array: fuse failed: ") +
                             e.what());
        }
        ribsJson.push_back({
            { "id",         k },
            { "center_x",   center.X() },
            { "center_y",   center.Y() },
            { "center_z",   center.Z() },
        });
    }

    // Build the macro signature.
    json params = {
        { "entry_face_kind", "resolved_id" },
        { "entry_face_id",   *entryId },
        { "spec_class",      in.spec_class },
        { "rib_count",       in.rib_count },
        { "pitch_mm",        in.pitch_mm },
        { "length_mm",       in.length_mm },
        { "height_mm",       in.height_mm },
        { "thickness_mm",    thick },
        { "anchor_x_mm",     in.anchor_x_mm },
        { "anchor_y_mm",     in.anchor_y_mm },
        { "array_axis",      { alongPitch.X(), alongPitch.Y(), alongPitch.Z() } },
        { "extrude_dir",     { extrude.X(), extrude.Y(), extrude.Z() } },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    in.rib_count },
        { "spec_class",          in.spec_class },
        { "spec_thickness_mm",   spec->thickness_mm },
        { "spec_max_ht_ratio",   spec->max_ht_ratio },
        { "rib_count",           in.rib_count },
        { "pitch_mm",            in.pitch_mm },
        { "length_mm",           in.length_mm },
        { "thickness_mm",        thick },
        { "height_mm",           in.height_mm },
        { "ht_ratio",            in.height_mm / thick },
        { "ribs",                ribsJson },
        { "array_axis",          { alongPitch.X(), alongPitch.Y(), alongPitch.Z() } },
        { "extrude_dir",         { extrude.X(), extrude.Y(), extrude.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "mold_feature_array";
    tooling.tool_dia_mm      = 0.0;
    tooling.tool_length_mm   = 0.0;
    tooling.tool_material    = "n/a";
    tooling.flute_count      = 0;
    // Volume ADDED (negative stock_removed).  Each rib ≈ L×T×H.
    tooling.stock_removed_mm3 =
        -static_cast<double>(in.rib_count) * in.length_mm * thick * in.height_mm;
    tooling.est_cycle_time_s = 0.0;
    tooling.extra = {
        { "compound_chain",   "build_rib_box × N → fuseMany" },
        { "spec_class",       in.spec_class },
        { "spec_thickness_mm", spec->thickness_mm },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::parametric_rib_array applied: N={} pitch={} L={} T={} H={} (spec='{}') faces {}→{}",
                  in.rib_count, in.pitch_mm, in.length_mm, thick, in.height_mm,
                  in.spec_class, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────
//
// Primary: replay from feature history.  Geometric fallback: scan for
// candidate top-of-rib faces (planar, normal aligned with extrude_dir,
// elongated bbox) and cluster them along a single axis at uniform pitch.

namespace {

bool nearlyEqual(double a, double b, double tol) { return std::abs(a - b) < tol; }

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Primary strategy: replay stored signatures.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "spec_class",   f.params.value("spec_class",   std::string()) },
            { "rib_count",    f.params.value("rib_count",    0) },
            { "pitch_mm",     f.params.value("pitch_mm",     0.0) },
            { "length_mm",    f.params.value("length_mm",    0.0) },
            { "height_mm",    f.params.value("height_mm",    0.0) },
            { "thickness_mm", f.params.value("thickness_mm", 0.0) },
            { "anchor_x_mm",  f.params.value("anchor_x_mm",  0.0) },
            { "anchor_y_mm",  f.params.value("anchor_y_mm",  0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, /*confidence*/ 0.92,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback: scan elongated planar faces with normal close to
    // +Z and cluster them by aspect / Y-stride.
    struct RibCandidate { gp_Pnt center; double length; double thickness; double topZ; };
    std::vector<RibCandidate> cands;
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double wpSpan = std::max({xMax - xMin, yMax - yMin, zMax - zMin});
    if (wpSpan < 1e-6) return out;

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.95) continue;          // we want UPWARD-facing tops
        const auto faceBb = pr::optimalBbox(wp.face(i));
        const double a = std::max(faceBb.dx(), faceBb.dy());
        const double b = std::min(faceBb.dx(), faceBb.dy());
        if (b < 1e-3) continue;
        const double aspect = a / b;
        if (aspect < 3.0) continue;
        if (a > wpSpan * 0.9) continue;       // not the base
        if (b > wpSpan * 0.3) continue;       // not the base
        if (faceBb.zMax < zMax - 1e-3) continue;  // accept only tops near workpiece top? not necessarily.
        // We treat any sufficiently elongated up-facing planar face as a
        // candidate rib top.
        RibCandidate c;
        c.center    = gp_Pnt((faceBb.xMin + faceBb.xMax) / 2.0,
                             (faceBb.yMin + faceBb.yMax) / 2.0,
                             (faceBb.zMin + faceBb.zMax) / 2.0);
        c.length    = a;
        c.thickness = b;
        c.topZ      = faceBb.zMax;
        cands.push_back(c);
    }
    if (cands.size() < 2) return out;

    // Group candidates whose thickness matches within 10% — they could be one
    // array.  Then check stride uniformity.
    for (size_t i = 0; i < cands.size(); ++i) {
        std::vector<size_t> group{ i };
        for (size_t j = i + 1; j < cands.size(); ++j) {
            if (nearlyEqual(cands[i].thickness, cands[j].thickness, cands[i].thickness * 0.1) &&
                nearlyEqual(cands[i].length,    cands[j].length,    cands[i].length    * 0.1))
                group.push_back(j);
        }
        if (group.size() < 2) continue;
        // Sort by Y (or X — pick the axis with largest spread).
        const double dx = std::abs(cands[group.back()].center.X() - cands[group.front()].center.X());
        const double dy = std::abs(cands[group.back()].center.Y() - cands[group.front()].center.Y());
        const bool axisY = dy > dx;
        std::sort(group.begin(), group.end(), [&](size_t a, size_t b){
            return axisY ? cands[a].center.Y() < cands[b].center.Y()
                         : cands[a].center.X() < cands[b].center.X();
        });
        // Stride.
        const double stride = axisY
            ? (cands[group.back()].center.Y() - cands[group.front()].center.Y())
            : (cands[group.back()].center.X() - cands[group.front()].center.X());
        if (std::abs(stride) < 1e-6) continue;
        const double pitch = std::abs(stride) / (group.size() - 1);

        const gp_Pnt anchor(
            (cands[group.front()].center.X() + cands[group.back()].center.X()) / 2.0,
            (cands[group.front()].center.Y() + cands[group.back()].center.Y()) / 2.0,
            cands[group.front()].center.Z());

        json rec = {
            { "spec_class",   "mold_standard" },   // best guess
            { "rib_count",    static_cast<int>(group.size()) },
            { "pitch_mm",     pitch },
            { "length_mm",    cands[group.front()].length },
            { "thickness_mm", cands[group.front()].thickness },
            { "height_mm",    cands[group.front()].topZ - zMin },
            { "anchor_x_mm",  anchor.X() },
            { "anchor_y_mm",  anchor.Y() },
        };
        json matched = {
            { "rib_top_face_ids", json::array() },
            { "axis",             axisY ? "y" : "x" },
        };
        out.push_back(RecognizedFeature{ kSkillId, rec, /*confidence*/ 0.55, matched });
        break;     // one array per call (greedy)
    }
    return out;
}

}  // namespace koocadcam::skill::parametric_rib_array
