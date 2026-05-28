// @lat: [[process/test-strategy#skill round-trip]]
//
// co_extrusion — multi-layer continuous extrusion (metadata-only).
//
// Cases (5):
//   1. apply handles input — geometric no-op + signature carries layer stack.
//   2. validate rejects bad input (< 2 layers).
//   3. signature records kind + layer_count + layers list with materials.
//   4. recognize() replays metadata.
//   5. specific assertion: total_thickness_mm == sum of per-layer thicknesses.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/co_extrusion.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

}  // namespace

// ─── 1. Apply handles input — geometric no-op ──────────────────────────────
TEST(SkillCoExtrusion, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(60.0, 5.0, 3.0, "abs");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::co_extrusion::Input in;
    in.layers = {
        { "abs",   1.0 },
        { "tpe",   0.5 },
        { "petg",  1.5 },
    };

    auto out = skill::co_extrusion::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("co_extrusion"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input (< 2 layers) ────────────────────────────
TEST(SkillCoExtrusion, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 5.0, 3.0);

    skill::co_extrusion::Input single;
    single.layers = { { "abs", 1.0 } };

    auto r = skill::co_extrusion::validate(*stock, single);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-COEX-COUNT"));
    EXPECT_THROW(skill::co_extrusion::apply(*stock, single),
                 skill::SkillError);

    // Empty material on one layer.
    skill::co_extrusion::Input emptyMat;
    emptyMat.layers = {
        { "abs", 1.0 },
        { "",    0.5 },        // empty material
    };
    auto r2 = skill::co_extrusion::validate(*stock, emptyMat);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-COEX-MATERIAL"));

    // Zero thickness.
    skill::co_extrusion::Input zeroThk;
    zeroThk.layers = {
        { "abs", 1.0 },
        { "tpe", 0.0 },        // zero thickness
    };
    auto r3 = skill::co_extrusion::validate(*stock, zeroThk);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-COEX-THICKNESS"));
}

// ─── 3. Signature records kind + layer_count + materials ───────────────────
TEST(SkillCoExtrusion, SignatureRecordsKindAndMaterials)
{
    auto stock = skill::createCuboidStock(60.0, 5.0, 3.0, "abs");

    skill::co_extrusion::Input in;
    in.layers = {
        { "abs",      1.0 },
        { "evoh",     0.1 },
        { "polyethylene", 0.8 },
    };

    auto out = skill::co_extrusion::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("co_extrusion"));
    EXPECT_EQ(out.signature.pattern["layer_count"].get<int>(), 3);

    const auto layersJ = out.signature.pattern["layers"];
    ASSERT_EQ(layersJ.size(), 3u);
    EXPECT_EQ(layersJ[0]["material"].get<std::string>(), std::string("abs"));
    EXPECT_EQ(layersJ[1]["material"].get<std::string>(), std::string("evoh"));
    EXPECT_EQ(layersJ[2]["material"].get<std::string>(),
              std::string("polyethylene"));
    EXPECT_NEAR(layersJ[1]["thickness_mm"].get<double>(), 0.1, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_multilayer"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("co_extrusion_die"));
}

// ─── 4. Recognize via metadata replay ──────────────────────────────────────
TEST(SkillCoExtrusion, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 5.0, 3.0);

    skill::co_extrusion::Input in;
    in.layers = {
        { "pet",      0.3 },
        { "tie",      0.05 },
        { "evoh",     0.05 },
        { "tie",      0.05 },
        { "pet",      0.3 },
    };

    auto out   = skill::co_extrusion::apply(*stock, in);
    auto cands = skill::co_extrusion::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_GT(cands[0].confidence, 0.99);
    const auto layersJ = cands[0].recovered_params["layers"];
    ASSERT_EQ(layersJ.size(), 5u);
    EXPECT_EQ(layersJ[2]["material"].get<std::string>(), std::string("evoh"));

    // Stripped metadata → no recognition.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::co_extrusion::recognize(raw).empty());
}

// ─── 5. Specific assertion: total_thickness_mm = Σ layer thicknesses ───────
TEST(SkillCoExtrusion, TotalThicknessMatchesSum)
{
    auto stock = skill::createCuboidStock(60.0, 5.0, 3.0);

    skill::co_extrusion::Input in;
    in.layers = {
        { "abs",   1.0 },
        { "tpe",   0.5 },
        { "petg",  1.5 },
        { "tpu",   0.25 },
    };

    auto out = skill::co_extrusion::apply(*stock, in);
    const double total = out.signature.pattern["total_thickness_mm"].get<double>();
    EXPECT_NEAR(total, 1.0 + 0.5 + 1.5 + 0.25, 1e-9);

    // layer_count matches input size.
    EXPECT_EQ(out.signature.pattern["layer_count"].get<int>(), 4);
}
