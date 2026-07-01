// @lat: [[process/test-strategy#re-roundtrip]] [[engine/feature-watch]]
//
// FULL-PART round-trip: build the COMPLETE watch (WatchFrontModel::buildAll, all
// ~10 steps — base, fillets, bezel, display pocket + sapphire dome, crown
// cavity/knob, side buttons, speaker grille, rear sensors, lugs, secondary
// fillets) → write to STEP (strips ALL feature history, leaving only geometry) →
// re-import as foreign geometry → re::analyze → inferProcessPlan → Executor →
// compare the re-synthesised solid to the original.
//
// Unlike watch_re_roundtrip (a hand-built drill+chamfer cuboid) this drives the
// REAL builder, so it MEASURES the true end-to-end recognition fidelity across
// every feature at once.  Two tests:
//   1. MEASUREMENT HARNESS (full default watch): reports which feature kinds the
//      geometric recognizer recovers vs misses, and the overall volume drift.
//      A loose gate (<40% drift) — its job is to SURFACE remaining gaps as
//      concrete numbers, not to be tight.  Tighten as gaps close.
//   2. STRICT fidelity (reduced spec, only currently-recognisable features):
//      a watch with just a bezel + display pocket + a couple of holes must
//      round-trip to <12% volume drift.

#include <gtest/gtest.h>

#include "engine/WatchFrontModel.hpp"
#include "io/StepIO.hpp"
#include "process/Executor.hpp"
#include "process/ProcessPlan.hpp"
#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Cylinder.hxx>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace koocadcam;
using nlohmann::json;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

// Build a watch from `spec`, write+read it through STEP (drops all metadata),
// and return the foreign-geometry Workpiece + the original shape.
struct Reimported {
    TopoDS_Shape original;
    std::shared_ptr<skill::Workpiece> foreign;
};
Reimported buildAndReimport(const json& spec, const std::string& tag)
{
    std::vector<engine::BuildWarning> warnings;
    const TopoDS_Shape shape = engine::WatchFrontModel::buildAll(spec, warnings);
    EXPECT_FALSE(shape.IsNull()) << "buildAll produced a null shape (" << tag << ")";

    const fs::path stepPath = fs::temp_directory_path() / ("watch_full_" + tag + ".step");
    std::string err;
    EXPECT_TRUE(io::StepIO::write(shape, stepPath, err)) << err;
    auto reim = io::StepIO::read(stepPath, err);
    EXPECT_TRUE(reim.has_value()) << err;

    Reimported r;
    r.original = shape;
    r.foreign  = reim.has_value() ? std::make_shared<skill::Workpiece>(*reim) : nullptr;
    return r;
}
}  // namespace

// ── 1. MEASUREMENT HARNESS — full default watch ────────────────────────────
//
// Surfaces, as concrete data, which feature kinds the geometric recognizer
// recovers on a complete watch and what the overall volume drift is.  Prints a
// per-skill recognition table.  Loose gate: it must recover SOMETHING and the
// re-synth volume must be in a sane ballpark — the value is the printed report,
// which dictates the next recognizer to build.
TEST(WatchFullRoundtrip, MeasuresRecognitionOfCompleteWatch)
{
    const json spec = engine::WatchFrontModel::defaultSpec();
    auto re_ = buildAndReimport(spec, "default");
    ASSERT_NE(re_.foreign, nullptr);
    const double vOrig = volumeOf(re_.original);

    // What does the geometric recognizer find on the foreign full watch?
    const auto cands = re::analyzeFiltered(*re_.foreign, 0.7);
    std::map<std::string, int> byKind;
    for (const auto& c : cands) byKind[c.skill_id]++;

    std::printf("\n[WATCH FULL] original volume = %.1f mm^3, faces = %d\n",
                vOrig, re_.foreign->faceCount());
    std::printf("[WATCH FULL] recognized feature kinds (conf>=0.7):\n");
    for (const auto& [k, n] : byKind)
        std::printf("    %-28s x%d\n", k.c_str(), n);
    if (byKind.empty())
        std::printf("    (none)\n");

    // The complete watch has many recognisable machining features (the bezel
    // ring, the display pocket, holes); analyze must surface at least a few.
    EXPECT_FALSE(cands.empty())
        << "a complete watch must yield SOME recognised features";

    // Re-synthesise from the inferred plan and report the volume drift.
    const process::ProcessPlan inferred = re::inferProcessPlan(*re_.foreign, 0.7);
    std::printf("[WATCH FULL] inferred plan steps = %d\n",
                static_cast<int>(inferred.size()));

    process::ProcessPlan replayable;
    for (const auto& step : inferred.steps())
        replayable.append(re::liftRecoveredStep(step));

    // Start from the same cylindrical stock the builder uses (Ø44 x 10).
    auto stock = skill::createCylindricalStock(44.0, 10.0);
    auto result = process::Executor::execute(replayable, stock, process::ExecuteOptions{true});
    ASSERT_NE(result.workpiece, nullptr) << "full-watch re-synth produced no workpiece";
    ASSERT_FALSE(result.workpiece->shape().IsNull());
    const double vResynth = volumeOf(result.workpiece->shape());
    const double drift = std::abs(vResynth - vOrig) / std::max(1.0, vOrig);
    std::printf("[WATCH FULL] re-synth volume = %.1f mm^3, drift = %.1f%%\n",
                vResynth, drift * 100.0);

    // FULL-WATCH FIDELITY GATE.  With the bezel (annular_groove) + lugs
    // (box_boss) now round-tripping, the complete default watch reproduces to a
    // few percent volume drift (measured ~3.4% — down from ~89% when the bezel
    // was mis-claimed as a stepped bore and the lugs were unrecovered).  The
    // residual is the still-unrecognised crown cavity / side buttons / speaker
    // grille, so we gate at <15% with headroom; tighten as those close.  A
    // regression that drops the bezel or a lug (or re-introduces an over-cut)
    // pushes the drift back up and trips this gate.
    EXPECT_LT(drift, 0.15)
        << "full-watch round-trip drift > 15% — a major feature stopped round-"
           "tripping (bezel/lugs) or a cut is over-claimed (see the printed table)";
}

// ── 2. STRICT fidelity — reduced spec, only recognisable features ──────────
//
// A watch built with ONLY features the geometric recognizer covers today
// (bezel ring + display pocket + a few holes) must round-trip tightly.  This is
// the regression gate: as long as those recognizers work, the end-to-end loop
// reproduces the part within a small tolerance.
TEST(WatchFullRoundtrip, StrictFidelityOnRecognisableSubset)
{
    // Start from the default spec but strip the features whose geometric
    // recognition is still a gap (crown knob/knurl, lugs, sensor domes, speaker
    // grille on the curved side, secondary micro-fillets), keeping the bezel,
    // display pocket and any straightforward holes.
    json spec = engine::WatchFrontModel::defaultSpec();
    spec.erase("crown_cavity");
    spec.erase("side_buttons");
    spec.erase("speaker_grille");
    spec.erase("rear_sensors");
    spec.erase("lugs");
    spec.erase("secondary_fillets");

    auto re_ = buildAndReimport(spec, "reduced");
    ASSERT_NE(re_.foreign, nullptr);
    const double vOrig = volumeOf(re_.original);

    const process::ProcessPlan inferred = re::inferProcessPlan(*re_.foreign, 0.7);
    // Report the post-dedupe plan (the bezel must survive as annular_groove,
    // not get displaced by a bore_with_shelf mis-claim on the shared walls).
    {
        std::printf("[WATCH REDUCED] inferred plan (post-dedupe):");
        bool sawAnnular = false;
        for (const auto& s : inferred.steps()) {
            std::printf(" %s", s.skill_id.c_str());
            if (s.skill_id == "annular_groove") sawAnnular = true;
        }
        std::printf("\n");
        EXPECT_TRUE(sawAnnular)
            << "the bezel must round-trip as annular_groove (not be displaced by "
               "a bore_with_shelf mis-claim that over-cuts the part)";
    }
    ASSERT_FALSE(inferred.empty())
        << "the reduced watch must yield a non-empty inferred plan";

    process::ProcessPlan replayable;
    for (const auto& step : inferred.steps())
        replayable.append(re::liftRecoveredStep(step));

    auto stock = skill::createCylindricalStock(44.0, 10.0);
    auto result = process::Executor::execute(replayable, stock, process::ExecuteOptions{true});
    ASSERT_NE(result.workpiece, nullptr);
    ASSERT_FALSE(result.workpiece->shape().IsNull());

    const double vResynth = volumeOf(result.workpiece->shape());
    const double drift = std::abs(vResynth - vOrig) / std::max(1.0, vOrig);
    std::printf("\n[WATCH REDUCED] orig=%.1f resynth=%.1f drift=%.1f%%\n",
                vOrig, vResynth, drift * 100.0);
    EXPECT_LT(drift, 0.12)
        << "reduced-watch (recognisable features only) must round-trip to <12% "
           "volume drift";
}

// ── 3. Crown KNOB (radial cone frustum) round-trip ─────────────────────────
//
// A realistic watch has a PROTRUDING crown knob — a sideways (radial-axis)
// tapered cone fused onto the case.  The default spec keeps the crown purely
// subtractive (a cavity) so other tests' volume/bbox assertions are stable, so
// here we OPT IN to the protruding knob and verify revolve_boss recovers it via
// its radial-axis cone meridian path: the knob must surface as a recognised
// revolve_boss feature, and a knob-only reduced watch must round-trip tightly.
TEST(WatchFullRoundtrip, CrownKnobRoundTripsAsRadialRevolve)
{
    // Reduced spec (only the bezel + display + holes + the protruding knob) so
    // the measured drift isolates the knob's contribution, not the still-gap
    // features.
    json spec = engine::WatchFrontModel::defaultSpec();
    spec.erase("side_buttons");
    spec.erase("speaker_grille");
    spec.erase("rear_sensors");
    spec.erase("lugs");
    spec.erase("secondary_fillets");
    // OPT IN: a protruding tapered crown knob pointing radially outward.
    spec["crown_cavity"]["body_protrusion_mm"] = 2.0;
    spec["crown_cavity"]["body_dia_mm"]        = 3.0;

    auto re_ = buildAndReimport(spec, "crown_knob");
    ASSERT_NE(re_.foreign, nullptr);
    const double vOrig = volumeOf(re_.original);

    const auto cands = re::analyzeFiltered(*re_.foreign, 0.7);
    std::map<std::string, int> byKind;
    for (const auto& c : cands) byKind[c.skill_id]++;
    std::printf("\n[WATCH KNOB] recognized kinds:");
    for (const auto& [k, n] : byKind) std::printf(" %s x%d", k.c_str(), n);
    std::printf("\n");

    // The protruding cone knob must be recovered as a revolve_boss with a
    // regeneratable (non-empty) profile and a ±X (radial) axis — NOT left as a
    // sub-threshold empty-meridian detection.
    const skill::RecognizedFeature* knob = nullptr;
    for (const auto& c : cands)
        if (c.skill_id == "revolve_boss" &&
            c.recovered_params.value("profile_polyline", json::array()).size() >= 3u) {
            knob = &c; break;
        }
    ASSERT_NE(knob, nullptr)
        << "the protruding crown knob must round-trip as a revolve_boss with a "
           "recovered cone meridian (radial-axis path)";
    const auto& ad = knob->recovered_params["axis_dir"];
    EXPECT_LT(std::abs(ad[2].get<double>()), 0.2)
        << "the crown knob axis must be roughly radial (in the XY plane), not ±Z";

    // End-to-end: the knob-bearing reduced watch must still round-trip tightly —
    // the additive knob is now reproduced, so it does not become drift.
    const process::ProcessPlan inferred = re::inferProcessPlan(*re_.foreign, 0.7);
    process::ProcessPlan replayable;
    for (const auto& step : inferred.steps())
        replayable.append(re::liftRecoveredStep(step));
    auto stock = skill::createCylindricalStock(44.0, 10.0);
    auto result = process::Executor::execute(replayable, stock, process::ExecuteOptions{true});
    ASSERT_NE(result.workpiece, nullptr);
    ASSERT_FALSE(result.workpiece->shape().IsNull());
    const double vResynth = volumeOf(result.workpiece->shape());
    const double drift = std::abs(vResynth - vOrig) / std::max(1.0, vOrig);
    std::printf("[WATCH KNOB] orig=%.1f resynth=%.1f drift=%.1f%%\n",
                vOrig, vResynth, drift * 100.0);
    EXPECT_LT(drift, 0.15)
        << "the crown-knob watch must round-trip to <15% (the additive knob is "
           "now recovered by the radial cone-meridian path)";
}

// ── 4. Speaker GRILLE (radial hole grid) diagnostic ────────────────────────
//
// The side speaker grille is a radial-axis hole grid.  linear_pattern now
// recovers such grids (single-feature test passes), but it does NOT yet surface
// on the full watch.  This test isolates WHY: build a grille-only watch, STEP
// round-trip it, and report how many small cylinders survive and whether
// linear_pattern recovers them — the printed table drives the next fix.
TEST(WatchFullRoundtrip, SpeakerGrilleRecognitionDiagnostic)
{
    // Strip only the OPTIONAL features (same safe set the StrictFidelity test
    // uses) so buildAll stays valid; keep the speaker grille.  The bezel /
    // display pocket remain, but they are large distinct radii, easy to tell
    // apart from the sub-mm grille holes in the printed table.
    json spec = engine::WatchFrontModel::defaultSpec();
    for (const char* k : { "crown_cavity", "side_buttons", "rear_sensors", "lugs",
                           "secondary_fillets" })
        spec.erase(k);

    auto re_ = buildAndReimport(spec, "grille_only");
    ASSERT_NE(re_.foreign, nullptr);

    std::map<int, int> byRad;   // radius*100 -> count
    for (int i = 0; i < re_.foreign->faceCount(); ++i)
        if (re_.foreign->isFaceCylinder(i)) {
            BRepAdaptor_Surface s(re_.foreign->face(i));
            const double r = s.Cylinder().Radius();
            byRad[static_cast<int>(r * 100)]++;
        }
    std::printf("\n[GRILLE] faces=%d cylinder faces by radius:\n",
                re_.foreign->faceCount());
    for (const auto& [r, n] : byRad)
        std::printf("    r=%.2fmm x%d\n", r / 100.0, n);

    const auto cands = re::analyzeFiltered(*re_.foreign, 0.7);
    std::map<std::string, int> byKind;
    for (const auto& c : cands) byKind[c.skill_id]++;
    std::printf("[GRILLE] recognised kinds:");
    for (const auto& [k, n] : byKind) std::printf(" %s x%d", k.c_str(), n);
    std::printf("\n");

    // The 2x6 side speaker grille must now surface as ONE linear_pattern (a 2-D
    // grid about the radial axis) — proving the arbitrary-axis + 2-D grid path
    // works end-to-end on a real product, not just the single-feature unit test.
    EXPECT_GE(byKind["linear_pattern"], 1)
        << "the side speaker grille must be recovered as a linear_pattern grid";
}
