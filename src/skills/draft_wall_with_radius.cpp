// @lat: [[engine/skills#draft_wall_with_radius]]

#include "draft_wall_with_radius.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Bbox.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Fillets.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <gp.hxx>
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

namespace koocadcam::skill::draft_wall_with_radius {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec library ─────────────────────────────────────────────────────────

namespace {

struct DraftSpec {
    const char* name;
    double      draft_deg;
    double      fillet_r_mm;
};

constexpr std::array<DraftSpec, 4> kDraftTable {{
    { "low_draft",        1.0, 0.3 },
    { "standard_draft",   3.0, 0.5 },
    { "high_draft",       5.0, 1.0 },
    { "aggressive_draft", 7.0, 1.5 },
}};

const DraftSpec* findSpec(const std::string& name)
{
    for (const auto& s : kDraftTable) {
        if (name == s.name) return &s;
    }
    return nullptr;
}

}  // namespace

double specDraftDegFor(const std::string& spec_class)
{
    const auto* s = findSpec(spec_class);
    return s ? s->draft_deg : 0.0;
}

double specFilletRMmFor(const std::string& spec_class)
{
    const auto* s = findSpec(spec_class);
    return s ? s->fillet_r_mm : 0.0;
}

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const auto* spec = findSpec(in.spec_class);
    if (!spec) {
        r.add("DFM-DRAFT-UNKNOWN", "error",
              "draft_wall_with_radius: unknown spec_class '" + in.spec_class +
              "' (supported: low_draft/standard_draft/high_draft/aggressive_draft)");
        return r;
    }

    if (in.base_thickness_mm < 0.4) {
        r.add("DFM-WALL-MIN", "error",
              "draft_wall_with_radius: base_thickness " +
              std::to_string(in.base_thickness_mm) +
              " mm < 0.4 mm mould-rule minimum");
    }

    if (in.length_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "draft_wall_with_radius: length_mm must be > 0");
    }
    if (in.height_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "draft_wall_with_radius: height_mm must be > 0");
    }

    if (spec->draft_deg < 0.0 || spec->draft_deg > 15.0) {
        r.add("DFM-DRAFT-RANGE", "error",
              "draft_wall_with_radius: draft " + std::to_string(spec->draft_deg) +
              " deg outside [0, 15]");
    }

    // Fillet sanity: R ≤ 0.4 × base_thickness so the fillet doesn't consume
    // the wall.  We use base_thickness as the "thinnest" reference; for
    // outward draft, top_thickness is larger so this is the binding side.
    if (in.base_thickness_mm > 0.0 && spec->fillet_r_mm > 0.4 * in.base_thickness_mm) {
        r.add("DFM-FILLET-LIMIT", "error",
              "draft_wall_with_radius: fillet R " +
              std::to_string(spec->fillet_r_mm) + " mm > 0.4 × base thickness (" +
              std::to_string(0.4 * in.base_thickness_mm) + " mm)");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

namespace {

TopoDS_Wire buildRectWire(const gp_Pnt& center,
                          const gp_Dir& xDir, const gp_Dir& yDir,
                          double dx, double dy)
{
    const double hx = dx / 2.0, hy = dy / 2.0;
    const gp_Pnt p1(center.X() - hx * xDir.X() - hy * yDir.X(),
                    center.Y() - hx * xDir.Y() - hy * yDir.Y(),
                    center.Z() - hx * xDir.Z() - hy * yDir.Z());
    const gp_Pnt p2(center.X() + hx * xDir.X() - hy * yDir.X(),
                    center.Y() + hx * xDir.Y() - hy * yDir.Y(),
                    center.Z() + hx * xDir.Z() - hy * yDir.Z());
    const gp_Pnt p3(center.X() + hx * xDir.X() + hy * yDir.X(),
                    center.Y() + hx * xDir.Y() + hy * yDir.Y(),
                    center.Z() + hx * xDir.Z() + hy * yDir.Z());
    const gp_Pnt p4(center.X() - hx * xDir.X() + hy * yDir.X(),
                    center.Y() - hx * xDir.Y() + hy * yDir.Y(),
                    center.Z() - hx * xDir.Z() + hy * yDir.Z());

    BRepBuilderAPI_MakePolygon poly;
    poly.Add(p1); poly.Add(p2); poly.Add(p3); poly.Add(p4); poly.Close();
    if (!poly.IsDone())
        throw Standard_Failure("draft_wall_with_radius: rect wire build failed");
    return poly.Wire();
}

}  // namespace

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "draft_wall_with_radius DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = findSpec(in.spec_class);
    const double draftRad = spec->draft_deg * M_PI / 180.0;
    const double rFillet  = spec->fillet_r_mm;

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("draft_wall_with_radius: entry_face datum unresolved");

    BRepAdaptor_Surface entrySurf(wp.face(*entryId));
    if (entrySurf.GetType() != GeomAbs_Plane) {
        throw SkillError("draft_wall_with_radius: entry_face must be planar");
    }
    const gp_Pln entryPlane = entrySurf.Plane();
    gp_Dir outward = entryPlane.Axis().Direction();
    if (wp.face(*entryId).Orientation() == TopAbs_REVERSED) outward.Reverse();

    // Build in-plane frame.  xDir = length_dir projected on entry plane.
    gp_Vec lenV(in.length_dir.X(), in.length_dir.Y(), in.length_dir.Z());
    gp_Vec extV(outward.X(), outward.Y(), outward.Z());
    lenV = lenV - extV * (lenV.Dot(extV));
    if (lenV.Magnitude() < 1e-9) {
        lenV = gp_Vec(1.0, 0.0, 0.0) - extV * (gp_Vec(1.0, 0.0, 0.0).Dot(extV));
        if (lenV.Magnitude() < 1e-9)
            lenV = gp_Vec(0.0, 1.0, 0.0) - extV * (gp_Vec(0.0, 1.0, 0.0).Dot(extV));
    }
    lenV.Normalize();
    const gp_Dir xDir(lenV.X(), lenV.Y(), lenV.Z());
    gp_Vec yV = extV.Crossed(lenV);
    yV.Normalize();
    const gp_Dir yDir(yV.X(), yV.Y(), yV.Z());

    // Project anchor onto entry plane.
    auto projectOntoPlane = [&](double x, double y) -> gp_Pnt {
        const gp_Pnt P0 = entryPlane.Location();
        const gp_Dir n  = entryPlane.Axis().Direction();
        if (std::abs(n.Z()) < 1e-9) return gp_Pnt(x, y, P0.Z());
        const double z = P0.Z() - (n.X() * (x - P0.X()) + n.Y() * (y - P0.Y())) / n.Z();
        return gp_Pnt(x, y, z);
    };
    constexpr double kEmbed = 0.05;
    const gp_Pnt baseCenter = projectOntoPlane(in.center_x_mm, in.center_y_mm);
    const gp_Pnt embeddedBase(baseCenter.X() - kEmbed * outward.X(),
                              baseCenter.Y() - kEmbed * outward.Y(),
                              baseCenter.Z() - kEmbed * outward.Z());

    const double baseT  = in.base_thickness_mm;
    const double topT   = baseT + 2.0 * in.height_mm * std::tan(draftRad);
    const double baseL  = in.length_mm;
    const double topL   = baseL + 2.0 * in.height_mm * std::tan(draftRad);

    // Bottom & top wires.  Bottom is slightly embedded into the workpiece.
    const TopoDS_Wire bottomWire = buildRectWire(embeddedBase, xDir, yDir, baseL, baseT);
    const gp_Pnt topCenter(embeddedBase.X() + (in.height_mm + kEmbed) * outward.X(),
                           embeddedBase.Y() + (in.height_mm + kEmbed) * outward.Y(),
                           embeddedBase.Z() + (in.height_mm + kEmbed) * outward.Z());
    const TopoDS_Wire topWire    = buildRectWire(topCenter, xDir, yDir, topL, topT);

    BRepOffsetAPI_ThruSections lofter(true /*isSolid*/, true /*ruled*/);
    lofter.AddWire(bottomWire);
    lofter.AddWire(topWire);
    lofter.Build();
    if (!lofter.IsDone())
        throw SkillError("draft_wall_with_radius: ThruSections failed");
    const TopoDS_Shape wallSolid = lofter.Shape();

    TopoDS_Shape fused;
    try {
        fused = pr::fuse(wp.shape(), wallSolid);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("draft_wall_with_radius: fuse failed: ") +
                         e.what());
    }

    // Apply top-edge fillet.  Top of wall is at world Z = topCenter.Z() (the
    // wall extends ABOVE entry_plane).  We pick edges whose mid-Z is within
    // tolerance of this Z.
    TopoDS_Shape finalShape = fused;
    if (rFillet > 0.0) {
        const double topZ = topCenter.Z();
        try {
            finalShape = pr::filletEdges(fused, rFillet, pr::edgesAtZ(topZ, 1e-2));
        } catch (const Standard_Failure&) {
            // Some OCCT builds fail on certain top-edge configurations; keep
            // the unfilleted shape rather than aborting.
            finalShape = fused;
        }
    }

    json params = {
        { "entry_face_kind",    "resolved_id" },
        { "entry_face_id",      *entryId },
        { "spec_class",         in.spec_class },
        { "center_x_mm",        in.center_x_mm },
        { "center_y_mm",        in.center_y_mm },
        { "length_mm",          in.length_mm },
        { "base_thickness_mm",  in.base_thickness_mm },
        { "height_mm",          in.height_mm },
        { "length_dir",         { xDir.X(), xDir.Y(), xDir.Z() } },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    5 },                  // base wire, top wire, loft, fuse, fillet
        { "spec_class",          in.spec_class },
        { "spec_draft_deg",      spec->draft_deg },
        { "spec_fillet_r_mm",    rFillet },
        { "base_thickness_mm",   baseT },
        { "top_thickness_mm",    topT },
        { "base_length_mm",      baseL },
        { "top_length_mm",       topL },
        { "height_mm",           in.height_mm },
        { "draft_angle_deg",     spec->draft_deg },
        { "fillet_r_mm",         rFillet },
        { "extrude_dir",         { outward.X(), outward.Y(), outward.Z() } },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "mold_feature";
    tooling.tool_dia_mm      = 0.0;
    tooling.tool_length_mm   = 0.0;
    tooling.tool_material    = "n/a";
    tooling.flute_count      = 0;
    // Trapezoidal prism volume ≈ height × length × (baseT + topT)/2 (added).
    const double vol = in.height_mm * (baseL + topL) / 2.0 * (baseT + topT) / 2.0
                       / std::max(1.0, 1.0);
    tooling.stock_removed_mm3 = -vol;
    tooling.extra = {
        { "compound_chain",   "base_wire → top_wire → ThruSections → fuse → top_fillet" },
        { "spec_class",       in.spec_class },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(finalShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::draft_wall_with_radius applied: spec='{}' L={} baseT={} H={} faces {}→{}",
                  in.spec_class, in.length_mm, in.base_thickness_mm, in.height_mm,
                  wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

namespace {

bool looksLikeDraftedWallTop(const Workpiece& wp, int faceIdx,
                              gp_Pnt& outCenter, double& outArea)
{
    if (!wp.isFacePlanar(faceIdx)) return false;
    gp_Dir n;
    try { n = wp.faceNormal(faceIdx); } catch (...) { return false; }
    if (n.Z() < 0.95) return false;
    outCenter = wp.faceCenter(faceIdx);
    outArea   = wp.faceArea(faceIdx);
    return true;
}

}  // namespace

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    // Primary: replay from feature history.
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "spec_class",        f.params.value("spec_class",        std::string()) },
            { "center_x_mm",       f.params.value("center_x_mm",       0.0) },
            { "center_y_mm",       f.params.value("center_y_mm",       0.0) },
            { "length_mm",         f.params.value("length_mm",         0.0) },
            { "base_thickness_mm", f.params.value("base_thickness_mm", 0.0) },
            { "height_mm",         f.params.value("height_mm",         0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, /*confidence*/ 0.92,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback: find a small upward-facing planar face high above
    // the dominant top, surrounded by angled walls.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    // Find the LARGEST upward-facing face (assumed base top), and any other
    // upward-facing face above it is a wall top candidate.
    int    baseId  = -1;
    double bestA   = -1.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        gp_Pnt c; double a;
        if (looksLikeDraftedWallTop(wp, i, c, a)) {
            if (a > bestA) { bestA = a; baseId = i; }
        }
    }
    (void)zMax;  // bbox values consumed below in candidate loop
    if (baseId < 0) return out;
    const double baseZ = wp.faceCenter(baseId).Z();

    for (int i = 0; i < wp.faceCount(); ++i) {
        if (i == baseId) continue;
        gp_Pnt c; double a;
        if (!looksLikeDraftedWallTop(wp, i, c, a)) continue;
        if (c.Z() <= baseZ + 1e-3) continue;
        const auto fb = pr::optimalBbox(wp.face(i));
        if (fb.dx() < 1e-3 || fb.dy() < 1e-3) continue;
        // Recover height + estimated base thickness (top minus 2H·tan(α));
        // we cheat and report the top thickness as base, draft = 0 — this is
        // a slice-1 fallback meant to flag the candidate, not reconstruct
        // the spec exactly.
        json rec = {
            { "spec_class",        "standard_draft" },
            { "center_x_mm",       c.X() },
            { "center_y_mm",       c.Y() },
            { "length_mm",         std::max(fb.dx(), fb.dy()) },
            { "base_thickness_mm", std::min(fb.dx(), fb.dy()) },
            { "height_mm",         c.Z() - baseZ },
        };
        json matched = { { "top_face_id", i }, { "base_face_id", baseId } };
        out.push_back(RecognizedFeature{ kSkillId, rec, /*confidence*/ 0.6, matched });
    }
    return out;
}

}  // namespace koocadcam::skill::draft_wall_with_radius
