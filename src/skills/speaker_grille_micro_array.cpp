// @lat: [[engine/skills#speaker_grille_micro_array]]

#include "speaker_grille_micro_array.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::speaker_grille_micro_array {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.face_id < 0 || in.face_id >= wp.faceCount()) {
        r.add("DFM-INPUT", "error",
              "speaker_grille_micro_array: face_id out of range");
    }
    if (!(in.grille_length_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "speaker_grille_micro_array: grille_length_mm must be > 0");
    }
    if (!(in.grille_width_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "speaker_grille_micro_array: grille_width_mm must be > 0");
    }
    if (!(in.hole_dia_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "speaker_grille_micro_array: hole_dia_mm must be > 0");
    }
    if (!(in.hole_pitch_mm > 0.0)) {
        r.add("DFM-INPUT", "error",
              "speaker_grille_micro_array: hole_pitch_mm must be > 0");
    }
    if (in.pattern != "hex" && in.pattern != "grid") {
        r.add("DFM-INPUT", "error",
              "speaker_grille_micro_array: pattern must be 'hex' or 'grid' (got '" +
              in.pattern + "')");
    }
    // DFM-MIN-HOLE: holes ≥ 0.3 mm
    if (in.hole_dia_mm > 0.0 && in.hole_dia_mm < 0.3) {
        r.add("DFM-MIN-HOLE", "error",
              "speaker_grille_micro_array: hole_dia_mm " +
              std::to_string(in.hole_dia_mm) + " < 0.3 mm minimum");
    }
    // DFM-MIN-WALL: rib (pitch − dia) ≥ 0.4 mm (epsilon for float math).
    if (in.hole_pitch_mm > 0.0 && in.hole_dia_mm > 0.0 &&
        (in.hole_pitch_mm - in.hole_dia_mm) < 0.4 - 1e-6) {
        r.add("DFM-MIN-WALL", "error",
              "speaker_grille_micro_array: rib width " +
              std::to_string(in.hole_pitch_mm - in.hole_dia_mm) +
              " < 0.4 mm minimum");
    }
    // DFM-PHONE-RNG — typical phone speaker hole window
    if (in.hole_dia_mm > 0.0 &&
        (in.hole_dia_mm < 0.5 || in.hole_dia_mm > 1.0)) {
        r.add("DFM-PHONE-RNG", "warning",
              "hole_dia " + std::to_string(in.hole_dia_mm) +
              " outside typical phone speaker range [0.5, 1.0] mm");
    }
    if (in.hole_pitch_mm > 0.0 &&
        (in.hole_pitch_mm < 1.0 || in.hole_pitch_mm > 1.5)) {
        r.add("DFM-PHONE-RNG", "warning",
              "hole_pitch " + std::to_string(in.hole_pitch_mm) +
              " outside typical phone speaker range [1.0, 1.5] mm");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "speaker_grille_micro_array DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error") msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const gp_Pnt faceC = wp.faceCenter(in.face_id);
    const double baseZ = faceC.Z();

    const double L      = in.grille_length_mm;
    const double W      = in.grille_width_mm;
    const double dia    = in.hole_dia_mm;
    const double pitch  = in.hole_pitch_mm;
    const double cx     = in.center_x_mm;
    const double cy     = in.center_y_mm;
    constexpr double kEmbed = 0.05;

    // pocket depth = max(dia * 1.5, 0.3 mm), capped at 0.6 mm — shallow grille frame.
    const double pocketDepth = std::min(0.6, std::max(0.3, dia * 1.5));

    const double volBefore = volumeOf(wp.shape());
    std::vector<TopoDS_Shape> cutters;

    // ── Op 1: shallow rectangular pocket frame ──────────────────────────
    {
        const gp_Pnt origin(cx - L / 2.0, cy - W / 2.0, baseZ - pocketDepth);
        const gp_Ax2 ax(origin, gp::DZ());
        cutters.push_back(pr::box(ax, L, W, pocketDepth + kEmbed));
    }

    // ── Op 2: drill micro-hole array through the pocket bottom ──────────
    //
    // Holes go all the way through the face from above (kEmbed overhang) to
    // a depth of (pocketDepth + 2 × baseZ) — practically we cut from baseZ + kEmbed
    // down through the workpiece. We approximate as a generous through-hole.
    const double holeR    = dia * 0.5;
    const double holeDepth = std::max(2.0, pocketDepth + 2.0);  // through bottom

    // Active hole area inside pocket — leave ≥ holeR margin from pocket wall.
    const double margin = holeR + 0.05;
    const double xLo = cx - L / 2.0 + margin;
    const double xHi = cx + L / 2.0 - margin;
    const double yLo = cy - W / 2.0 + margin;
    const double yHi = cy + W / 2.0 - margin;
    if (xHi <= xLo || yHi <= yLo) {
        throw SkillError(
            "speaker_grille_micro_array: grille too small for hole array");
    }

    std::vector<std::pair<double, double>> centers;
    if (in.pattern == "grid") {
        for (double y = yLo; y <= yHi + 1e-9; y += pitch) {
            for (double x = xLo; x <= xHi + 1e-9; x += pitch) {
                centers.emplace_back(x, y);
            }
        }
    } else {  // hex
        const double rowDy = pitch * std::sqrt(3.0) * 0.5;
        int rowIdx = 0;
        for (double y = yLo; y <= yHi + 1e-9; y += rowDy, ++rowIdx) {
            const double xShift = (rowIdx % 2 == 0) ? 0.0 : pitch * 0.5;
            for (double x = xLo + xShift; x <= xHi + 1e-9; x += pitch) {
                centers.emplace_back(x, y);
            }
        }
    }
    if (centers.empty()) {
        throw SkillError(
            "speaker_grille_micro_array: no hole centers fit inside grille area");
    }

    for (const auto& c : centers) {
        const gp_Pnt p(c.first, c.second, baseZ + kEmbed);
        const gp_Ax2 ax(p, gp_Dir(0.0, 0.0, -1.0));
        cutters.push_back(pr::cylinder(ax, holeR, holeDepth));
    }

    TopoDS_Shape newShape = pr::cutMany(wp.shape(), cutters);
    if (newShape.IsNull()) {
        throw SkillError("speaker_grille_micro_array: cutMany returned null");
    }
    const double volAfter = volumeOf(newShape);
    const double volRemoved = volBefore - volAfter;

    const int holeCount = static_cast<int>(centers.size());

    json params = {
        { "face_id",          in.face_id },
        { "center_x_mm",      in.center_x_mm },
        { "center_y_mm",      in.center_y_mm },
        { "grille_length_mm", in.grille_length_mm },
        { "grille_width_mm",  in.grille_width_mm },
        { "hole_dia_mm",      in.hole_dia_mm },
        { "hole_pitch_mm",    in.hole_pitch_mm },
        { "pattern",          in.pattern },
    };
    json pattern = {
        { "kind",                       kSkillId },
        { "is_compound",                true },
        { "is_phone_feature",           true },
        { "phone_feature_type",         "speaker_grille" },
        { "subfeature_count",           static_cast<int>(cutters.size()) },
        { "hole_count",                 holeCount },
        { "pattern_type",               in.pattern },
        { "hole_dia_mm",                in.hole_dia_mm },
        { "hole_pitch_mm",              in.hole_pitch_mm },
        { "pocket_depth_mm",            pocketDepth },
        { "grille_length_mm",           in.grille_length_mm },
        { "grille_width_mm",            in.grille_width_mm },
        { "derived_volume_removed_mm3", volRemoved },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "micro_drill;end_mill";
    tooling.tool_dia_mm       = std::max(0.3, in.hole_dia_mm);
    tooling.tool_material     = "carbide";
    tooling.flute_count       = 2;
    tooling.stock_removed_mm3 = volRemoved;
    tooling.est_cycle_time_s  = 10.0 + static_cast<double>(holeCount) * 0.8;
    tooling.extra = {
        { "hole_count",      holeCount },
        { "pattern_type",    in.pattern },
        { "pocket_depth_mm", pocketDepth },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::speaker_grille_micro_array: holes={} pattern={} vol-removed={}",
                  holeCount, in.pattern, volRemoved);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    auto emitMetadataReplay = [&]() {
        for (const auto& f : wp.features()) {
            if (f.skill_id != kSkillId) continue;
            RecognizedFeature rf;
            rf.skill_id         = kSkillId;
            rf.recovered_params = f.params;
            rf.confidence       = 0.95;
            rf.matched_geometry = f.pattern;
            out.push_back(rf);
        }
    };

    // Geometric: large cluster of small equal-radius cylinders (the micro-hole
    // array) is the signature of this skill.
    std::vector<double> radii;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFaceCylinder(i)) continue;
        try {
            BRepAdaptor_Surface s(wp.face(i));
            const gp_Cylinder gc = s.Cylinder();
            // micro-grille holes are ≤ 1.0 mm dia => radius ≤ 0.5 mm
            if (gc.Radius() <= 0.55) {
                radii.push_back(gc.Radius());
            }
        } catch (...) {}
    }

    if (radii.size() >= 6) {
        std::sort(radii.begin(), radii.end());
        int bestCount = 1, run = 1;
        double bestR = radii.front();
        for (size_t i = 1; i < radii.size(); ++i) {
            if (std::abs(radii[i] - radii[i - 1]) < 1e-2) {
                ++run;
                if (run > bestCount) { bestCount = run; bestR = radii[i]; }
            } else {
                run = 1;
            }
        }
        if (bestCount >= 6) {
            json recovered = {
                { "face_id",          -1 },
                { "center_x_mm",      0.0 },
                { "center_y_mm",      0.0 },
                { "grille_length_mm", 8.0 },
                { "grille_width_mm",  3.0 },
                { "hole_dia_mm",      2.0 * bestR },
                { "hole_pitch_mm",    0.0 },
                { "pattern",          "hex" },
            };
            json matched = {
                { "cyl_count",  bestCount },
                { "dom_radius", bestR },
                { "via",        "geometric_primary" },
            };
            out.push_back(RecognizedFeature{ kSkillId, recovered, 0.7, matched });
        }
    }

    emitMetadataReplay();
    return out;
}

}  // namespace koocadcam::skill::speaker_grille_micro_array
