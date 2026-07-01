// @lat: [[process/test-strategy#re-roundtrip]] [[engine/feature-phone]]
//
// FULL-PART round-trip for the PHONE: build the complete phone
// (PhoneFrontModel::buildAll: base + corner/edge fillets, rounded-rect display
// pocket, camera holes + island bump, side buttons, USB-C obround port, camera
// deco rings) -> STEP (strips all feature history) -> re-import as foreign
// geometry -> re::analyze -> inferProcessPlan -> Executor -> compare to the
// original.  Mirrors watch_full_roundtrip; measures end-to-end phone fidelity.

#include <gtest/gtest.h>

#include "engine/PhoneFrontModel.hpp"
#include "io/StepIO.hpp"
#include "process/Executor.hpp"
#include "process/ProcessPlan.hpp"
#include "re/Recognizer.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

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

struct Reimported {
    TopoDS_Shape original;
    std::shared_ptr<skill::Workpiece> foreign;
};
Reimported buildAndReimport(const json& spec, const std::string& tag)
{
    std::vector<engine::BuildWarning> warnings;
    const TopoDS_Shape shape = engine::PhoneFrontModel::buildAll(spec, warnings);
    EXPECT_FALSE(shape.IsNull()) << "buildAll produced a null shape (" << tag << ")";

    const fs::path stepPath = fs::temp_directory_path() / ("phone_full_" + tag + ".step");
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

// ── 1. MEASUREMENT HARNESS — full default phone ────────────────────────────
TEST(PhoneFullRoundtrip, MeasuresRecognitionOfCompletePhone)
{
    const json spec = engine::PhoneFrontModel::defaultSpec();
    auto re_ = buildAndReimport(spec, "default");
    ASSERT_NE(re_.foreign, nullptr);
    const double vOrig = volumeOf(re_.original);

    const auto cands = re::analyzeFiltered(*re_.foreign, 0.7);
    std::map<std::string, int> byKind;
    for (const auto& c : cands) byKind[c.skill_id]++;

    std::printf("\n[PHONE FULL] original volume = %.1f mm^3, faces = %d\n",
                vOrig, re_.foreign->faceCount());
    std::printf("[PHONE FULL] recognized feature kinds (conf>=0.7):\n");
    for (const auto& [k, n] : byKind) std::printf("    %-28s x%d\n", k.c_str(), n);
    if (byKind.empty()) std::printf("    (none)\n");

    EXPECT_FALSE(cands.empty()) << "a complete phone must yield SOME recognised features";

    const process::ProcessPlan inferred = re::inferProcessPlan(*re_.foreign, 0.7);
    std::printf("[PHONE FULL] inferred plan steps = %d\n", static_cast<int>(inferred.size()));

    process::ProcessPlan replayable;
    for (const auto& step : inferred.steps()) replayable.append(re::liftRecoveredStep(step));

    auto stock = skill::createCuboidStock(76.0, 160.0, 8.0);
    auto result = process::Executor::execute(replayable, stock, process::ExecuteOptions{true});
    ASSERT_NE(result.workpiece, nullptr) << "phone re-synth produced no workpiece";
    ASSERT_FALSE(result.workpiece->shape().IsNull());
    const double vResynth = volumeOf(result.workpiece->shape());
    const double drift = std::abs(vResynth - vOrig) / std::max(1.0, vOrig);
    std::printf("[PHONE FULL] re-synth volume = %.1f mm^3, drift = %.1f%%\n",
                vResynth, drift * 100.0);

    // Measurement gate — sanity bounds (non-degenerate solid).  Tighten as the
    // remaining phone gaps close; the printed table is the backlog signal.
    EXPECT_GT(vResynth, 0.5 * vOrig)
        << "phone re-synth collapsed — recognition/replay is broken (see table)";
    EXPECT_LT(vResynth, 1.20 * vOrig)
        << "phone re-synth ADDED net volume — an inverted cut or double boss";
}

// ── 2. STRICT fidelity — reduced spec, only recognisable features ──────────
TEST(PhoneFullRoundtrip, StrictFidelityOnRecognisableSubset)
{
    // Keep the features the geometric recognizer covers today (display pocket,
    // camera holes, USB-C obround, deco rings); strip the still-gap features.
    json spec = engine::PhoneFrontModel::defaultSpec();
    spec.erase("side_buttons");
    spec.erase("camera_island");   // additive island handled by box_boss but may
                                   // interact with the camera holes; keep strict

    auto re_ = buildAndReimport(spec, "reduced");
    ASSERT_NE(re_.foreign, nullptr);
    const double vOrig = volumeOf(re_.original);

    const process::ProcessPlan inferred = re::inferProcessPlan(*re_.foreign, 0.7);
    ASSERT_FALSE(inferred.empty()) << "the reduced phone must yield a plan";
    {
        std::map<std::string, int> bk;
        for (const auto& s : inferred.steps()) bk[s.skill_id]++;
        std::printf("[PHONE REDUCED] inferred plan (post-dedupe):");
        for (const auto& [k, n] : bk) std::printf(" %s x%d", k.c_str(), n);
        std::printf("\n");
        // REGRESSION: every recovered coordinate must be SANE (within the part's
        // bounding box, ~ +/-200mm).  The side USB-C obround was recovered with
        // end_y ~ -5e7 (an ill-conditioned entry-plane projection where the slot
        // axis grazed the chosen entry face), placing the cut outside the stock.
        for (const auto& s : inferred.steps()) {
            for (auto it = s.params.begin(); it != s.params.end(); ++it) {
                if (!it.value().is_number()) continue;
                const double v = it.value().get<double>();
                EXPECT_LT(std::abs(v), 1000.0)
                    << "recovered param " << it.key() << "=" << v << " on "
                    << s.skill_id << " is out of range (coordinate blow-up)";
            }
        }
        // REGRESSION: box_boss must NOT claim the display-pocket FRAME (the flat
        // top face bounded by inward pocket walls) as a large additive boss —
        // that over-shoots the cut-only phone (was a 158x74x0.6 false boss).
        const auto raw = re::analyzeFiltered(*re_.foreign, 0.7);
        for (const auto& c : raw) {
            if (c.skill_id != "box_boss") continue;
            const double L = c.recovered_params.value("length_mm", 0.0);
            const double W = c.recovered_params.value("width_mm", 0.0);
            EXPECT_LT(std::max(L, W), 100.0)
                << "box_boss falsely claimed a part-sized boss (a pocket frame), "
                   "L=" << L << " W=" << W << " — convexity gate regressed";
        }
    }

    process::ProcessPlan replayable;
    for (const auto& step : inferred.steps()) replayable.append(re::liftRecoveredStep(step));

    auto stock = skill::createCuboidStock(76.0, 160.0, 8.0);
    auto result = process::Executor::execute(replayable, stock, process::ExecuteOptions{true});
    ASSERT_NE(result.workpiece, nullptr);
    ASSERT_FALSE(result.workpiece->shape().IsNull());

    const double vResynth = volumeOf(result.workpiece->shape());
    const double drift = std::abs(vResynth - vOrig) / std::max(1.0, vOrig);
    std::printf("\n[PHONE REDUCED] orig=%.1f resynth=%.1f drift=%.1f%%\n",
                vOrig, vResynth, drift * 100.0);
    EXPECT_LT(drift, 0.12)
        << "reduced-phone (recognisable features only) must round-trip to <12% drift";
}
