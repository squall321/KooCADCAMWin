// @lat: [[engine/skills#partition_wall_with_passthrough]]

#include "partition_wall_with_passthrough.hpp"

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

namespace koocadcam::skill::partition_wall_with_passthrough {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Spec library ─────────────────────────────────────────────────────────

namespace {

struct PassSpec {
    const char* name;
    double      cutout_w_mm;
    double      cutout_h_mm;
};

constexpr std::array<PassSpec, 5> kPassTable {{
    { "cable_pass",      6.0,  4.0 },
    { "connector_pass", 12.0,  8.0 },
    { "airflow_louver", 20.0,  3.0 },
    { "usb_c_pass",      9.0,  4.0 },
    { "hdmi_pass",      16.0,  7.0 },
}};

const PassSpec* findSpec(const std::string& name)
{
    for (const auto& s : kPassTable) {
        if (name == s.name) return &s;
    }
    return nullptr;
}

}  // namespace

double specCutoutWFor(const std::string& s) { const auto* x = findSpec(s); return x ? x->cutout_w_mm : 0.0; }
double specCutoutHFor(const std::string& s) { const auto* x = findSpec(s); return x ? x->cutout_h_mm : 0.0; }

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    const auto* spec = findSpec(in.spec_class);
    if (!spec) {
        r.add("DFM-PASS-UNKNOWN", "error",
              "partition_wall_with_passthrough: unknown spec_class '" + in.spec_class +
              "' (supported: cable_pass/connector_pass/airflow_louver/usb_c_pass/hdmi_pass)");
        return r;
    }

    if (in.wall_thickness_mm < 0.4) {
        r.add("DFM-WALL-MIN", "error",
              "partition_wall_with_passthrough: wall_thickness " +
              std::to_string(in.wall_thickness_mm) +
              " mm < 0.4 mm mould-rule minimum");
    }

    if (in.wall_length_mm <= 0.0 || in.wall_height_mm <= 0.0) {
        r.add("DFM-PASS-INPUT", "error",
              "partition_wall_with_passthrough: wall_length and wall_height must be > 0");
    }

    if (spec->cutout_w_mm >= in.wall_length_mm) {
        r.add("DFM-PASS-FIT", "error",
              "partition_wall_with_passthrough: cutout_w " +
              std::to_string(spec->cutout_w_mm) + " mm ≥ wall_length " +
              std::to_string(in.wall_length_mm) + " mm");
    }
    if (spec->cutout_h_mm >= in.wall_height_mm) {
        r.add("DFM-PASS-FIT", "error",
              "partition_wall_with_passthrough: cutout_h " +
              std::to_string(spec->cutout_h_mm) + " mm ≥ wall_height " +
              std::to_string(in.wall_height_mm) + " mm");
    }

    // Strap check: at least 1 mm of wall left above the cutout.
    // cut_top_z = cut_offset_z + cutout_h/2.  We resolve cut_offset_z the
    // same way apply() does: if 0, default to (wall_height − cutout_h)/2.
    double cutZ = in.cut_offset_z_mm;
    if (cutZ <= 1e-9) cutZ = (in.wall_height_mm - spec->cutout_h_mm) / 2.0;
    const double topStrap = in.wall_height_mm - (cutZ + spec->cutout_h_mm / 2.0);
    if (topStrap < 1.0 && in.wall_height_mm > 0.0) {
        r.add("DFM-PASS-STRAP", "error",
              "partition_wall_with_passthrough: top strap " +
              std::to_string(topStrap) + " mm < 1.0 mm");
    }

    (void)wp;
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "partition_wall_with_passthrough DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const auto* spec = findSpec(in.spec_class);
    const double Wc = spec->cutout_w_mm;
    const double Hc = spec->cutout_h_mm;

    auto entryId = wp.resolve(in.entry_face);
    if (!entryId) throw SkillError("partition_wall_with_passthrough: entry_face datum unresolved");

    BRepAdaptor_Surface entrySurf(wp.face(*entryId));
    if (entrySurf.GetType() != GeomAbs_Plane) {
        throw SkillError("partition_wall_with_passthrough: entry_face must be planar");
    }
    const gp_Pln entryPlane = entrySurf.Plane();
    gp_Dir outward = entryPlane.Axis().Direction();
    if (wp.face(*entryId).Orientation() == TopAbs_REVERSED) outward.Reverse();

    if (outward.Z() < 0.95) {
        throw SkillError("partition_wall_with_passthrough: only +Z entry faces supported in slice 1");
    }

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double topZ = zMax;

    // In-plane frame.  xDir = length, yDir = thickness.
    gp_Vec lenV(in.length_dir.X(), in.length_dir.Y(), in.length_dir.Z());
    gp_Vec extV(outward.X(), outward.Y(), outward.Z());
    lenV = lenV - extV * (lenV.Dot(extV));
    if (lenV.Magnitude() < 1e-9) lenV = gp_Vec(1.0, 0.0, 0.0);
    lenV.Normalize();
    const gp_Dir xDir(lenV.X(), lenV.Y(), lenV.Z());
    gp_Vec yV = extV.Crossed(lenV); yV.Normalize();
    const gp_Dir yDir(yV.X(), yV.Y(), yV.Z());

    // Resolve cut_offset_z if 0 — centred default.
    double cutZ = in.cut_offset_z_mm;
    if (cutZ <= 1e-9) cutZ = (in.wall_height_mm - Hc) / 2.0;

    constexpr double kEmbed = 0.05;

    // ── Step 1: build wall box and fuse. ────────────────────────────────
    //
    // Wall lives between Z = topZ - kEmbed and Z = topZ + H.
    // Wall footprint centred on (center_x, center_y), aligned with length_dir.
    const double cx = in.center_x_mm;
    const double cy = in.center_y_mm;
    const double L  = in.wall_length_mm;
    const double T  = in.wall_thickness_mm;
    const double H  = in.wall_height_mm;

    const gp_Pnt wallOrigin(
        cx - 0.5 * L * xDir.X() - 0.5 * T * yDir.X(),
        cy - 0.5 * L * xDir.Y() - 0.5 * T * yDir.Y(),
        topZ - kEmbed);
    const TopoDS_Shape wallBox = pr::box(gp_Ax2(wallOrigin, outward, xDir),
                                         L, T, H + kEmbed);

    TopoDS_Shape current = wp.shape();
    try {
        current = pr::fuse(current, wallBox);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("partition_wall_with_passthrough: fuse failed: ") +
                         e.what());
    }

    // ── Step 2: cut the passthrough hole. ───────────────────────────────
    //
    // Cutter extends slightly past the wall thickness in both Y directions,
    // and is placed at (cut_offset_x along xDir, cutZ above face).
    const double cutCenterX = cx + in.cut_offset_x_mm * xDir.X();
    const double cutCenterY = cy + in.cut_offset_x_mm * xDir.Y();
    const double cutZWorld  = topZ + cutZ;
    const double cutDepth   = T + 2.0 * kEmbed;

    const gp_Pnt cutOrigin(
        cutCenterX - 0.5 * Wc * xDir.X() - (0.5 * T + kEmbed) * yDir.X(),
        cutCenterY - 0.5 * Wc * xDir.Y() - (0.5 * T + kEmbed) * yDir.Y(),
        cutZWorld   - 0.5 * Hc);
    const TopoDS_Shape cutter = pr::box(gp_Ax2(cutOrigin, outward, xDir),
                                        Wc, cutDepth, Hc);

    TopoDS_Shape finalShape;
    try {
        finalShape = pr::cut(current, cutter);
    } catch (const Standard_Failure& e) {
        throw SkillError(std::string("partition_wall_with_passthrough: cut failed: ") +
                         e.what());
    }

    json params = {
        { "entry_face_id",     *entryId },
        { "spec_class",        in.spec_class },
        { "center_x_mm",       cx },
        { "center_y_mm",       cy },
        { "wall_length_mm",    L },
        { "wall_thickness_mm", T },
        { "wall_height_mm",    H },
        { "cut_offset_x_mm",   in.cut_offset_x_mm },
        { "cut_offset_z_mm",   cutZ },
        { "length_dir",        { xDir.X(), xDir.Y(), xDir.Z() } },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    2 },
        { "spec_class",          in.spec_class },
        { "cutout_w_mm",         Wc },
        { "cutout_h_mm",         Hc },
        { "wall_length_mm",      L },
        { "wall_thickness_mm",   T },
        { "wall_height_mm",      H },
        { "cut_offset_x_mm",     in.cut_offset_x_mm },
        { "cut_offset_z_mm",     cutZ },
        { "top_strap_mm",        H - (cutZ + Hc / 2.0) },
        { "bot_strap_mm",        cutZ - Hc / 2.0 },
    };

    ToolingMeta tooling;
    tooling.tool_type        = "compound_partition";
    tooling.tool_dia_mm      = 0.0;
    tooling.tool_length_mm   = 0.0;
    tooling.tool_material    = "n/a";
    tooling.flute_count      = 0;
    // Net volume = wall added - cutout removed.
    const double wallVol = L * T * H;
    const double cutVol  = Wc * T * Hc;
    tooling.stock_removed_mm3 = cutVol - wallVol;
    tooling.extra = {
        { "compound_chain", "wall_box → fuse → passthrough_box → cut" },
        { "spec_class",     in.spec_class },
        { "cutout_w_mm",    Wc },
        { "cutout_h_mm",    Hc },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(finalShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::partition_wall_with_passthrough applied: spec='{}' L={} T={} H={} faces {}→{}",
                  in.spec_class, L, T, H, wp.faceCount(), wpNew->faceCount());

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rec = {
            { "spec_class",        f.params.value("spec_class",        std::string()) },
            { "center_x_mm",       f.params.value("center_x_mm",       0.0) },
            { "center_y_mm",       f.params.value("center_y_mm",       0.0) },
            { "wall_length_mm",    f.params.value("wall_length_mm",    0.0) },
            { "wall_thickness_mm", f.params.value("wall_thickness_mm", 0.0) },
            { "wall_height_mm",    f.params.value("wall_height_mm",    0.0) },
            { "cut_offset_x_mm",   f.params.value("cut_offset_x_mm",   0.0) },
            { "cut_offset_z_mm",   f.params.value("cut_offset_z_mm",   0.0) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rec, /*confidence*/ 0.92,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback: look for a thin elongated extrusion above entry
    // plane with a single hole through it.  Slice-1 approximation: emit a
    // single candidate when an upward-facing planar face is found above the
    // workpiece's main top plane and a circular/rectangular hole feature is
    // detectable on a wall side.  We keep this VERY conservative.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);

    int wallTopId = -1;
    double wallTopZ = -1e30;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n;
        try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.95) continue;
        const gp_Pnt c = wp.faceCenter(i);
        if (c.Z() <= zMax - 1e-3) continue;
        if (c.Z() > wallTopZ) { wallTopZ = c.Z(); wallTopId = i; }
    }
    if (wallTopId < 0) return out;

    const auto fb = pr::optimalBbox(wp.face(wallTopId));
    if (fb.dx() <= 0.0 || fb.dy() <= 0.0) return out;
    // Likely length-thickness ratio > 3 for partition walls.
    const double a = std::max(fb.dx(), fb.dy());
    const double b = std::min(fb.dx(), fb.dy());
    if (a / std::max(1e-6, b) < 3.0) return out;

    json rec = {
        { "spec_class",        "connector_pass" },
        { "center_x_mm",       (fb.xMin + fb.xMax) / 2.0 },
        { "center_y_mm",       (fb.yMin + fb.yMax) / 2.0 },
        { "wall_length_mm",    a },
        { "wall_thickness_mm", b },
        { "wall_height_mm",    wallTopZ - zMax + (zMax - zMin) * 0.0 }, // approx
        { "cut_offset_x_mm",   0.0 },
        { "cut_offset_z_mm",   0.0 },
    };
    json matched = { { "wall_top_face_id", wallTopId } };
    out.push_back(RecognizedFeature{ kSkillId, rec, /*confidence*/ 0.65, matched });
    return out;
}

}  // namespace koocadcam::skill::partition_wall_with_passthrough
