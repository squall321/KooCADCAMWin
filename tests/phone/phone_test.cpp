// @lat: [[process/test-strategy#phone]]
// PhoneFrontModel M2-phase-1 integration tests.
// Validates that the parametric primitives layer (extracted in
// refactor 797903b) composes correctly for rectangular geometry.

#include <gtest/gtest.h>

#include "engine/PhoneFrontModel.hpp"
#include "engine/primitives/Bbox.hpp"
#include "io/JsonSpec.hpp"

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

namespace {

fs::path resolveFixture(const char* leaf)
{
    fs::path p = fs::current_path();
    for (int i = 0; i < 6; ++i) {
        fs::path candidate = p / "tests" / "data" / leaf;
        if (fs::exists(candidate)) return candidate;
        p = p.parent_path();
    }
    if (const char* env = std::getenv("KOO_TESTS_DATA_DIR"))
        return fs::path(env) / leaf;
    return {};
}

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// 1. Sample fixture builds a valid B-rep with ~76×160×8 envelope
TEST(PhoneFrontModel, BuildAllSampleProducesValidShape)
{
    using namespace koocadcam;
    const fs::path specPath = resolveFixture("sample_phone.json");
    ASSERT_FALSE(specPath.empty()) << "sample_phone.json not found";

    std::string err;
    auto specOpt = io::JsonSpec::read(specPath, err);
    ASSERT_TRUE(specOpt.has_value()) << err;

    std::vector<engine::BuildWarning> warnings;
    TopoDS_Shape shape = engine::PhoneFrontModel::buildAll(*specOpt, warnings);
    ASSERT_FALSE(shape.IsNull()) << "buildAll returned null shape";

    BRepCheck_Analyzer analyzer(shape);
    EXPECT_TRUE(analyzer.IsValid()) << "shape failed BRepCheck_Analyzer";

    const auto bb = engine::prim::optimalBbox(shape);
    EXPECT_NEAR(bb.dx(),  76.0, 0.5) << "phone width out of tolerance";
    EXPECT_NEAR(bb.dy(), 160.0, 0.5) << "phone height out of tolerance";
    EXPECT_NEAR(bb.dz(),   8.0, 0.5) << "phone thickness out of tolerance";
}

// 2. buildAll is deterministic
TEST(PhoneFrontModel, BuildAllIsDeterministic)
{
    using namespace koocadcam;
    nlohmann::json spec = engine::PhoneFrontModel::defaultSpec();

    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape s1 = engine::PhoneFrontModel::buildAll(spec, w1);
    TopoDS_Shape s2 = engine::PhoneFrontModel::buildAll(spec, w2);
    ASSERT_FALSE(s1.IsNull());
    ASSERT_FALSE(s2.IsNull());

    EXPECT_GT(volumeOf(s1), 0.0);
    EXPECT_NEAR(volumeOf(s1), volumeOf(s2), 1e-6);
}

// 3. Display pocket removes material from the slab
TEST(PhoneFrontModel, DisplayPocketReducesVolume)
{
    using namespace koocadcam;
    nlohmann::json full = engine::PhoneFrontModel::defaultSpec();
    nlohmann::json noPkt = full;
    noPkt.erase("display_pocket");

    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape sFull  = engine::PhoneFrontModel::buildAll(full,  w1);
    TopoDS_Shape sNoPkt = engine::PhoneFrontModel::buildAll(noPkt, w2);
    ASSERT_FALSE(sFull.IsNull());
    ASSERT_FALSE(sNoPkt.IsNull());

    EXPECT_LT(volumeOf(sFull), volumeOf(sNoPkt))
        << "display pocket should remove material";

    // Pocket volume ~ width * height * depth (sharp-rectangle approximation,
    // 25 % tolerance because rounded corners reduce the actual cut).
    const auto& dp = full["display_pocket"];
    const double approx = dp["width_mm"].get<double>()
                        * dp["height_mm"].get<double>()
                        * dp["depth_mm"].get<double>();
    EXPECT_NEAR(volumeOf(sNoPkt) - volumeOf(sFull), approx, approx * 0.25);
}

// 4. Camera holes count affects volume monotonically
TEST(PhoneFrontModel, MoreCamerasRemoveMoreMaterial)
{
    using namespace koocadcam;
    nlohmann::json oneCam = engine::PhoneFrontModel::defaultSpec();
    oneCam["cameras"] = nlohmann::json::array({ oneCam["cameras"][0] });

    nlohmann::json allCams = engine::PhoneFrontModel::defaultSpec();

    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape sOne = engine::PhoneFrontModel::buildAll(oneCam,  w1);
    TopoDS_Shape sAll = engine::PhoneFrontModel::buildAll(allCams, w2);
    ASSERT_FALSE(sOne.IsNull());
    ASSERT_FALSE(sAll.IsNull());

    EXPECT_LT(volumeOf(sAll), volumeOf(sOne))
        << "3 cameras should remove more material than 1";
}

// 4b. Raised camera island (opt-in) — the signature modern-phone bump.  ADDS
// material (fused platform > the lens holes through it) and protrudes -Z past
// the rear face.  Default (no camera_island) is unchanged.
TEST(PhoneFrontModel, CameraIslandAddsMaterialAndProtrudes)
{
    using namespace koocadcam;
    nlohmann::json plain = engine::PhoneFrontModel::defaultSpec();
    ASSERT_TRUE(plain.contains("cameras"));
    ASSERT_FALSE(plain["cameras"].empty());

    // Place the island around the first camera, sized to cover the cluster.
    nlohmann::json island = plain;
    const double cx = plain["cameras"][0]["offset_x_mm"].get<double>();
    const double cy = plain["cameras"][0]["offset_y_mm"].get<double>();
    island["camera_island"] = {
        { "center_x_mm", cx }, { "center_y_mm", cy },
        { "width_mm", 18.0 }, { "height_mm", 18.0 }, { "bump_mm", 1.2 } };

    std::vector<engine::BuildWarning> wp, wi;
    TopoDS_Shape sPlain  = engine::PhoneFrontModel::buildAll(plain,  wp);
    TopoDS_Shape sIsland = engine::PhoneFrontModel::buildAll(island, wi);
    ASSERT_FALSE(sPlain.IsNull());
    ASSERT_FALSE(sIsland.IsNull());
    EXPECT_TRUE(BRepCheck_Analyzer(sIsland).IsValid())
        << "camera island must yield a valid solid";

    EXPECT_GT(volumeOf(sIsland), volumeOf(sPlain))
        << "camera island should ADD material (platform > lens holes)";

    const double dzPlain  = engine::prim::optimalBbox(sPlain).dz();
    const double dzIsland = engine::prim::optimalBbox(sIsland).dz();
    EXPECT_GT(dzIsland, dzPlain + 0.5)
        << "camera island should protrude past the rear face (thicker -Z)";
}

// 5. Missing optional sections pass through (engine should not crash)
TEST(PhoneFrontModel, OptionalSectionsArePassThrough)
{
    using namespace koocadcam;
    nlohmann::json minimal{
        { "schema_version", "0.1.0" },
        { "product_name",   "Bare Slab" },
        { "base", {
            { "width_mm", 70.0 }, { "height_mm", 140.0 },
            { "thickness_mm", 8.0 }, { "initial_corner_r_mm", 5.0 }
        }},
        { "corner_radius", {
            { "r_top_mm", 0.5 }, { "r_side_mm", 0.5 }
        }}
    };

    std::vector<engine::BuildWarning> warnings;
    TopoDS_Shape shape = engine::PhoneFrontModel::buildAll(minimal, warnings);
    ASSERT_FALSE(shape.IsNull()) << "minimal spec must still build";

    const auto bb = engine::prim::optimalBbox(shape);
    EXPECT_NEAR(bb.dx(),  70.0, 0.5);
    EXPECT_NEAR(bb.dy(), 140.0, 0.5);
    EXPECT_NEAR(bb.dz(),   8.0, 0.5);
}

// ── DFM: the default phone must clear the shared rule catalog ─────────────
// (breakthrough plan B5.5 — phone profile of dfm::runProductDFM; same gate
// philosophy as watch_features' RunDFMPassesOnSampleSpec.)
TEST(PhoneFrontModel, RunDFMPassesOnDefaultSpec)
{
    using namespace koocadcam;
    nlohmann::json spec = engine::PhoneFrontModel::defaultSpec();

    std::vector<engine::BuildWarning> warnings;
    TopoDS_Shape shape = engine::PhoneFrontModel::buildAll(spec, warnings);
    ASSERT_FALSE(shape.IsNull());

    auto report = engine::PhoneFrontModel::runDFM(shape, spec);

    std::string errs;
    for (const auto& f : report.findings)
        if (f.severity == "error")
            errs += "\n  [" + f.code + "] " + f.message;

    EXPECT_TRUE(report.passed)
        << "default phone spec must pass DFM — error findings:" << errs;
    // The expansion summary rules must run for phones too.
    auto hasCode = [&](const char* code) {
        for (const auto& f : report.findings)
            if (f.code == code) return true;
        return false;
    };
    EXPECT_TRUE(hasCode("DFM-012")) << "face-count rule must run";
    EXPECT_TRUE(hasCode("DFM-016")) << "fill-ratio rule must run";
}

// ── M2-phase-2 steps 5–7 ───────────────────────────────────────────────────

namespace {
// A base slab (steps 1–2) to apply individual M2-phase-2 steps onto.
TopoDS_Shape phoneBase()
{
    auto spec = koocadcam::engine::PhoneFrontModel::defaultSpec();
    koocadcam::engine::PhoneFrontModel m;
    auto s = m.buildBase(spec).shape;
    return m.applyCornerRadius(s, spec).shape;
}
}  // namespace

// Step 5: side buttons cut material from the body and stay a valid solid.
TEST(PhoneFrontModel, SideButtonsCutBothSides)
{
    using namespace koocadcam;
    auto spec = engine::PhoneFrontModel::defaultSpec();
    const TopoDS_Shape base = phoneBase();
    engine::PhoneFrontModel m;
    const TopoDS_Shape out = m.addSideButtons(base, spec).shape;

    ASSERT_FALSE(out.IsNull());
    EXPECT_TRUE(BRepCheck_Analyzer(out).IsValid());
    EXPECT_LT(volumeOf(out), volumeOf(base)) << "side buttons must remove material";
    // Width is unchanged (pockets are shallow, well inside the body).
    EXPECT_NEAR(engine::prim::optimalBbox(out).dx(), 76.0, 0.5);
}

// Step 5: missing/empty side_buttons is a pass-through (no change).
TEST(PhoneFrontModel, SideButtonsAbsentIsNoOp)
{
    using namespace koocadcam;
    auto spec = engine::PhoneFrontModel::defaultSpec();
    spec.erase("side_buttons");
    const TopoDS_Shape base = phoneBase();
    engine::PhoneFrontModel m;
    const TopoDS_Shape out = m.addSideButtons(base, spec).shape;
    EXPECT_NEAR(volumeOf(out), volumeOf(base), 1e-6);
}

// Step 6: the USB-C port cuts an opening on the bottom (−Y) edge.
TEST(PhoneFrontModel, PortHoleOnBottomEdge)
{
    using namespace koocadcam;
    auto spec = engine::PhoneFrontModel::defaultSpec();
    const TopoDS_Shape base = phoneBase();
    engine::PhoneFrontModel m;
    const TopoDS_Shape out = m.addPortHole(base, spec).shape;

    ASSERT_FALSE(out.IsNull());
    EXPECT_TRUE(BRepCheck_Analyzer(out).IsValid());
    EXPECT_LT(volumeOf(out), volumeOf(base)) << "port must remove material";
    // The bottom edge stays at y = -80 (the cut opens the face, not extends it).
    EXPECT_NEAR(engine::prim::optimalBbox(out).yMin, -80.0, 0.5);
}

// Step 7: deco rings cut shallow annular grooves around the rear cameras.
TEST(PhoneFrontModel, DecoRingsCutGrooves)
{
    using namespace koocadcam;
    auto spec = engine::PhoneFrontModel::defaultSpec();
    // Apply against a body that already has the camera holes (step 4).
    engine::PhoneFrontModel m;
    auto s = m.buildBase(spec).shape;
    s = m.applyCornerRadius(s, spec).shape;
    s = m.addCameraHoles(s, spec).shape;
    const double vBefore = volumeOf(s);
    const TopoDS_Shape out = m.addCameraDecoRings(s, spec).shape;

    ASSERT_FALSE(out.IsNull());
    EXPECT_TRUE(BRepCheck_Analyzer(out).IsValid());
    EXPECT_LT(volumeOf(out), vBefore) << "deco rings must remove material";
}

// DFM-018: a deco-ring whose annular land is below the min wall warns.
TEST(PhoneFrontModel, DFM018FiresOnThinDecoRingLand)
{
    using namespace koocadcam;
    auto spec = engine::PhoneFrontModel::defaultSpec();
    spec["camera_deco_rings"] = nlohmann::json::array({
        nlohmann::json{ { "offset_x_mm", -22.0 }, { "offset_y_mm", -55.0 },
                        { "outer_dia_mm", 9.0 }, { "inner_dia_mm", 8.6 },  // land 0.2 < 0.4
                        { "depth_mm", 0.3 } } });
    std::vector<engine::BuildWarning> warnings;
    TopoDS_Shape shape = engine::PhoneFrontModel::buildAll(spec, warnings);
    ASSERT_FALSE(shape.IsNull());
    auto report = engine::PhoneFrontModel::runDFM(shape, spec);

    bool dfm018 = false;
    for (const auto& f : report.findings) if (f.code == "DFM-018") dfm018 = true;
    EXPECT_TRUE(dfm018) << "thin deco-ring land must raise DFM-018";
}

// Step 7: a ring whose inner_dia >= outer_dia is skipped with a warning.
TEST(PhoneFrontModel, DecoRingDegenerateIsSkipped)
{
    using namespace koocadcam;
    auto spec = engine::PhoneFrontModel::defaultSpec();
    spec["camera_deco_rings"] = nlohmann::json::array({
        nlohmann::json{ { "offset_x_mm", -22.0 }, { "offset_y_mm", -55.0 },
                        { "outer_dia_mm", 8.0 }, { "inner_dia_mm", 9.0 },
                        { "depth_mm", 0.3 } } });
    const TopoDS_Shape base = phoneBase();
    engine::PhoneFrontModel m;
    auto r = m.addCameraDecoRings(base, spec);
    EXPECT_NEAR(volumeOf(r.shape), volumeOf(base), 1e-6) << "degenerate ring left no groove";
    bool warned = false;
    for (const auto& w : r.warnings) if (w.code == "W_DECO_RING") warned = true;
    EXPECT_TRUE(warned) << "degenerate ring must warn";
}
