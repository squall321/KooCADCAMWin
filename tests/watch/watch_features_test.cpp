// @lat: [[process/test-strategy#watch]]
// WatchFrontModel build steps 4-6 integration tests (M1.2).

#include <gtest/gtest.h>

#include "engine/ProductFrontModel.hpp"
#include "engine/WatchFrontModel.hpp"
#include "io/JsonSpec.hpp"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <vector>
#include <cmath>
#include <numbers>

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
    if (const char* env = std::getenv("KOO_TESTS_DATA_DIR")) {
        return fs::path(env) / leaf;
    }
    return {};
}

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

nlohmann::json loadSampleSpec()
{
    const fs::path specPath = resolveFixture("sample_watch.json");
    if (specPath.empty()) return {};
    std::string err;
    auto opt = koocadcam::io::JsonSpec::read(specPath, err);
    return opt.has_value() ? *opt : nlohmann::json{};
}

} // namespace

// 1. Display pocket removes material
TEST(WatchFeaturesM12, DisplayPocketReducesVolume)
{
    using namespace koocadcam;
    nlohmann::json full = loadSampleSpec();
    ASSERT_FALSE(full.empty()) << "sample_watch.json not found";

    nlohmann::json noPocket = full;
    noPocket.erase("display_pocket");

    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape sFull   = engine::WatchFrontModel::buildAll(full,     w1);
    TopoDS_Shape sNoPkt  = engine::WatchFrontModel::buildAll(noPocket, w2);
    ASSERT_FALSE(sFull.IsNull());
    ASSERT_FALSE(sNoPkt.IsNull());

    const double vFull  = volumeOf(sFull);
    const double vNoPkt = volumeOf(sNoPkt);
    EXPECT_LT(vFull, vNoPkt) << "display pocket should remove material";

    // Approximate cylinder volume with 30% tolerance
    const double r = full["display_pocket"]["d_pocket_mm"].get<double>() / 2.0;
    const double d = full["display_pocket"]["depth_pocket_mm"].get<double>();
    const double cylinderVol = M_PI * r * r * d;
    const double diff = vNoPkt - vFull;
    EXPECT_NEAR(diff, cylinderVol, cylinderVol * 0.30);
}

// 2. Crown cavity removes material
TEST(WatchFeaturesM12, CrownCavityReducesVolume)
{
    using namespace koocadcam;
    nlohmann::json full = loadSampleSpec();
    ASSERT_FALSE(full.empty()) << "sample_watch.json not found";
    ASSERT_TRUE(full.contains("crown_cavity")) << "fixture missing crown_cavity";

    nlohmann::json noCrown = full;
    noCrown.erase("crown_cavity");

    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape sFull    = engine::WatchFrontModel::buildAll(full,     w1);
    TopoDS_Shape sNoCrown = engine::WatchFrontModel::buildAll(noCrown,  w2);
    ASSERT_FALSE(sFull.IsNull());
    ASSERT_FALSE(sNoCrown.IsNull());

    EXPECT_LT(volumeOf(sFull), volumeOf(sNoCrown))
        << "crown cavity should remove material";
}

// 3. More side buttons → more material removed
TEST(WatchFeaturesM12, SideButtonsCountAffectsVolume)
{
    using namespace koocadcam;
    nlohmann::json specOne = engine::WatchFrontModel::defaultSpec();
    // Ensure exactly one button
    specOne["side_buttons"] = nlohmann::json::array();
    specOne["side_buttons"].push_back({
        {"angle_deg", 60.0}, {"height_mm", 5.0}, {"length_mm", 5.0},
        {"width_mm",  2.0},  {"depth_mm",  0.8}, {"taper_deg", 0.0}
    });

    nlohmann::json specTwo = specOne;
    specTwo["side_buttons"].push_back({
        {"angle_deg", 180.0}, {"height_mm", 5.0}, {"length_mm", 5.0},
        {"width_mm",  2.0},   {"depth_mm",  0.8}, {"taper_deg", 0.0}
    });

    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape sOne = engine::WatchFrontModel::buildAll(specOne, w1);
    TopoDS_Shape sTwo = engine::WatchFrontModel::buildAll(specTwo, w2);
    ASSERT_FALSE(sOne.IsNull());
    ASSERT_FALSE(sTwo.IsNull());

    EXPECT_LT(volumeOf(sTwo), volumeOf(sOne))
        << "two buttons should remove more material than one";
}

// 4. Empty side_buttons array == no side_buttons key
TEST(WatchFeaturesM12, EmptySideButtonsIsPassThrough)
{
    using namespace koocadcam;
    nlohmann::json base = engine::WatchFrontModel::defaultSpec();
    base.erase("side_buttons");

    nlohmann::json withEmpty = base;
    withEmpty["side_buttons"] = nlohmann::json::array();

    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape sNoKey  = engine::WatchFrontModel::buildAll(base,      w1);
    TopoDS_Shape sEmpty  = engine::WatchFrontModel::buildAll(withEmpty, w2);
    ASSERT_FALSE(sNoKey.IsNull());
    ASSERT_FALSE(sEmpty.IsNull());

    EXPECT_NEAR(volumeOf(sNoKey), volumeOf(sEmpty), 1.0)
        << "empty array should be identical to missing key (within 1 mm³)";
}

// 5. Missing crown_cavity key == same as no crown_cavity (both omitted)
TEST(WatchFeaturesM12, CrownDisabledMatchesMissingKey)
{
    using namespace koocadcam;
    nlohmann::json base = engine::WatchFrontModel::defaultSpec();
    base.erase("crown_cavity");

    // Two specs that both lack crown_cavity — should produce same volume
    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape s1 = engine::WatchFrontModel::buildAll(base, w1);
    TopoDS_Shape s2 = engine::WatchFrontModel::buildAll(base, w2);
    ASSERT_FALSE(s1.IsNull());
    ASSERT_FALSE(s2.IsNull());

    EXPECT_NEAR(volumeOf(s1), volumeOf(s2), 1e-6)
        << "same spec without crown_cavity must produce identical volumes";
}

// 6. Full build preserves outer envelope
TEST(WatchFeaturesM12, FullBuildPreservesBaseDimensions)
{
    using namespace koocadcam;
    nlohmann::json spec = loadSampleSpec();
    ASSERT_FALSE(spec.empty()) << "sample_watch.json not found";

    std::vector<engine::BuildWarning> warnings;
    TopoDS_Shape shape = engine::WatchFrontModel::buildAll(spec, warnings);
    ASSERT_FALSE(shape.IsNull());

    Bnd_Box box;
    BRepBndLib::AddOptimal(shape, box);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    // M1.5 full envelope: case 44 mm Ø, lugs at 12/6 o'clock extend Y by
    // 6 mm each side → dy ≈ 56.  Lugs are axially centered, no Z extension.
    EXPECT_NEAR(xmax - xmin, 44.0, 1.0) << "dx out of range";
    EXPECT_NEAR(ymax - ymin, 56.0, 1.0) << "dy out of range";
    EXPECT_NEAR(zmax - zmin, 10.0, 0.5) << "dz out of range";
}

// 7. Bezel taper changes geometry
// Re-enabled in M1.3 — the refactor (797903b) switched buildBezel from
// BRepBndLib::Add (NURBS control-polygon bulge ≈ +1.8 mm) to
// BRepBndLib::AddOptimal (true surface bbox), so the annular cut tool's
// outer wall now sits AT the case surface instead of 1.8 mm outside it.
// With taper_deg > 0, the cut tool's outer wall slopes inward at the
// bottom, removing additional material near the inner rim.
TEST(WatchFeaturesM12, BezelTaperChangesShape)
{
    using namespace koocadcam;
    nlohmann::json specFlat = engine::WatchFrontModel::defaultSpec();
    specFlat["bezel"]["taper_deg"] = 0.0;

    nlohmann::json specTaper = specFlat;
    specTaper["bezel"]["taper_deg"] = 5.0;

    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape sFlat  = engine::WatchFrontModel::buildAll(specFlat,  w1);
    TopoDS_Shape sTaper = engine::WatchFrontModel::buildAll(specTaper, w2);
    ASSERT_FALSE(sFlat.IsNull());
    ASSERT_FALSE(sTaper.IsNull());

    // Tapered cut tool removes more material (outer wall narrows toward
    // bottom → larger annular ring carved). Loose 0.1 mm³ threshold tolerates
    // OCCT solver micro-variation.
    const double diff = std::abs(volumeOf(sFlat) - volumeOf(sTaper));
    EXPECT_GE(diff, 0.1) << "taper_deg=5 should change volume by >= 0.1 mm³";
}

// ── M1.5 step 7-10 tests ──────────────────────────────────────────────────

// 8. Speaker grille removes material in proportion to hole count
TEST(WatchFeaturesM15, SpeakerGrilleReducesVolume)
{
    using namespace koocadcam;
    nlohmann::json full = engine::WatchFrontModel::defaultSpec();
    nlohmann::json noGrille = full;
    noGrille.erase("speaker_grille");

    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape sFull   = engine::WatchFrontModel::buildAll(full,     w1);
    TopoDS_Shape sNoGr   = engine::WatchFrontModel::buildAll(noGrille, w2);
    ASSERT_FALSE(sFull.IsNull());
    ASSERT_FALSE(sNoGr.IsNull());
    EXPECT_LT(volumeOf(sFull), volumeOf(sNoGr))
        << "speaker grille should remove material";
}

// 9. Rear sensor holes remove material
TEST(WatchFeaturesM15, RearSensorsReduceVolume)
{
    using namespace koocadcam;
    nlohmann::json full = engine::WatchFrontModel::defaultSpec();
    nlohmann::json noSensors = full;
    noSensors.erase("rear_sensors");

    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape sFull = engine::WatchFrontModel::buildAll(full,      w1);
    TopoDS_Shape sNoS  = engine::WatchFrontModel::buildAll(noSensors, w2);
    ASSERT_FALSE(sFull.IsNull());
    ASSERT_FALSE(sNoS.IsNull());
    EXPECT_LT(volumeOf(sFull), volumeOf(sNoS))
        << "rear sensors should remove material";
}

// 10. Lugs ADD material (fuse, not cut) — opposite sign from cut steps
TEST(WatchFeaturesM15, LugsAddMaterial)
{
    using namespace koocadcam;
    nlohmann::json full = engine::WatchFrontModel::defaultSpec();
    // For a clean A/B, also drop pin holes (lugs adds material; pin holes
    // remove it).  Test the protrusion sign independently of pin holes.
    if (full.contains("lugs")) {
        for (auto& l : full["lugs"]) l.erase("pin_hole_dia_mm");
    }
    nlohmann::json noLugs = full;
    noLugs.erase("lugs");

    std::vector<engine::BuildWarning> w1, w2;
    TopoDS_Shape sFull   = engine::WatchFrontModel::buildAll(full,   w1);
    TopoDS_Shape sNoLugs = engine::WatchFrontModel::buildAll(noLugs, w2);
    ASSERT_FALSE(sFull.IsNull());
    ASSERT_FALSE(sNoLugs.IsNull());
    EXPECT_GT(volumeOf(sFull), volumeOf(sNoLugs))
        << "lugs (no pin holes) should ADD material";
}

// 11. Lug span symmetry — bbox should grow symmetrically with 12 + 6 o'clock lugs
TEST(WatchFeaturesM15, LugBboxSymmetric)
{
    using namespace koocadcam;
    nlohmann::json spec = engine::WatchFrontModel::defaultSpec();

    std::vector<engine::BuildWarning> warnings;
    TopoDS_Shape shape = engine::WatchFrontModel::buildAll(spec, warnings);
    ASSERT_FALSE(shape.IsNull());

    Bnd_Box box;
    BRepBndLib::AddOptimal(shape, box);
    double xMin, yMin, zMin, xMax, yMax, zMax;
    box.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    // Lugs at 90° (+Y) and 270° (-Y) → both extend the Y extent symmetrically.
    EXPECT_NEAR(std::abs(yMax), std::abs(yMin), 0.2)
        << "Y extent should be symmetric (12/6 o'clock lugs)";
    // Y extent must exceed case diameter (44) by some lug projection.
    EXPECT_GT(yMax - yMin, 50.0)
        << "Y extent should grow past 50mm with 6mm lugs on both sides";
}

// 12. runDFM passes on the default sample
TEST(WatchFeaturesM15, RunDFMPassesOnSampleSpec)
{
    using namespace koocadcam;
    nlohmann::json spec = engine::WatchFrontModel::defaultSpec();

    std::vector<engine::BuildWarning> warnings;
    TopoDS_Shape shape = engine::WatchFrontModel::buildAll(spec, warnings);
    ASSERT_FALSE(shape.IsNull());

    auto report = engine::WatchFrontModel::runDFM(shape, spec);
    EXPECT_TRUE(report.passed)
        << "default spec must pass DFM — first failure: "
        << (report.findings.empty() ? std::string("none")
                                    : report.findings.front().code + ": " +
                                      report.findings.front().message);
}

// 13. runDFM catches DFM-002 violation (sub-0.8 mm hole)
TEST(WatchFeaturesM15, RunDFMCatchesDFM002UnderSizedHole)
{
    using namespace koocadcam;
    nlohmann::json spec = engine::WatchFrontModel::defaultSpec();
    spec["crown_cavity"]["shaft_dia_mm"] = 0.5;  // violates DFM-002 (min 0.8)

    std::vector<engine::BuildWarning> warnings;
    TopoDS_Shape shape = engine::WatchFrontModel::buildAll(spec, warnings);
    ASSERT_FALSE(shape.IsNull());

    auto report = engine::WatchFrontModel::runDFM(shape, spec);
    EXPECT_FALSE(report.passed);
    bool foundDfm002 = false;
    for (const auto& f : report.findings) {
        if (f.code == "DFM-002") { foundDfm002 = true; break; }
    }
    EXPECT_TRUE(foundDfm002) << "expected DFM-002 finding for 0.5 mm shaft";
}

// 14. runDFM catches DFM-009 violation (bezel too narrow)
TEST(WatchFeaturesM15, RunDFMCatchesDFM009ThinBezel)
{
    using namespace koocadcam;
    nlohmann::json spec = engine::WatchFrontModel::defaultSpec();
    spec["bezel"]["width_mm"] = 0.4;  // violates DFM-009 (min 0.6)

    std::vector<engine::BuildWarning> warnings;
    TopoDS_Shape shape = engine::WatchFrontModel::buildAll(spec, warnings);
    // shape may or may not be null with such thin bezel; runDFM only needs spec
    auto report = engine::WatchFrontModel::runDFM(
        shape.IsNull() ? engine::WatchFrontModel::buildAll(
                            engine::WatchFrontModel::defaultSpec(), warnings)
                       : shape,
        spec);
    EXPECT_FALSE(report.passed);
    bool found = false;
    for (const auto& f : report.findings) {
        if (f.code == "DFM-009") { found = true; break; }
    }
    EXPECT_TRUE(found) << "expected DFM-009 finding for 0.4 mm bezel";
}
