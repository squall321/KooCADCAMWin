// @lat: [[process/test-strategy#skill round-trip]]
//
// powder_blend — mix powders by mass fraction (no geometric change).
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput (empty list, fraction sum off, empty material)
//   3. SignatureRecordsKindAndKeyParams (component array)
//   4. RecognizeMetadataReplay
//   5. component_count matches input

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/powder_blend.hpp"

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

}  // namespace

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillPowderBlend, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);
    const double volBefore   = volumeOf(stock->shape());
    const int    facesBefore = stock->faceCount();
    const int    edgesBefore = stock->edgeCount();

    skill::powder_blend::Input in;
    in.components = {
        { "iron_pure",          0.85 },
        { "carbon_graphite",    0.10 },
        { "copper_pure",        0.05 },
    };

    auto out = skill::powder_blend::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_EQ(out.workpiece->faceCount(), facesBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgesBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillPowderBlend, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    // Empty component list.
    skill::powder_blend::Input emptyList;
    auto r1 = skill::powder_blend::validate(*stock, emptyList);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::powder_blend::apply(*stock, emptyList),
                 skill::SkillError);

    // Fractions don't sum to 1.0.
    skill::powder_blend::Input badSum;
    badSum.components = {
        { "iron_pure",   0.50 },
        { "copper_pure", 0.30 },     // sum = 0.80
    };
    auto r2 = skill::powder_blend::validate(*stock, badSum);
    EXPECT_FALSE(r2.passed);

    // Empty material string.
    skill::powder_blend::Input emptyMat;
    emptyMat.components = {
        { "",            0.50 },
        { "copper_pure", 0.50 },
    };
    auto r3 = skill::powder_blend::validate(*stock, emptyMat);
    EXPECT_FALSE(r3.passed);

    // Negative fraction.
    skill::powder_blend::Input negFrac;
    negFrac.components = {
        { "iron_pure",   -0.10 },
        { "copper_pure",  1.10 },     // sum = 1.0 but negative entry
    };
    auto r4 = skill::powder_blend::validate(*stock, negFrac);
    EXPECT_FALSE(r4.passed);
}

// ─── 3. Signature records kind + component array ─────────────────────────
TEST(SkillPowderBlend, SignatureRecordsKindAndComponents)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::powder_blend::Input in;
    in.components = {
        { "iron_pure",          0.945 },
        { "carbon_graphite",    0.005 },
        { "copper_pure",        0.050 },
    };

    auto out = skill::powder_blend::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("powder_blend"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("powder_blend"));
    EXPECT_EQ(out.signature.pattern["process_family"].get<std::string>(),
              std::string("powder_metallurgy"));
    ASSERT_TRUE(out.signature.pattern["components"].is_array());
    ASSERT_EQ(out.signature.pattern["components"].size(), 3u);
    EXPECT_EQ(out.signature.pattern["components"][0]["material"].get<std::string>(),
              std::string("iron_pure"));
    EXPECT_NEAR(out.signature.pattern["components"][0]["fraction"].get<double>(),
                0.945, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("v_blender"));
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillPowderBlend, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::powder_blend::Input in;
    in.components = {
        { "stainless_316l", 0.95 },
        { "nickel_pure",    0.05 },
    };

    auto out   = skill::powder_blend::apply(*stock, in);
    auto cands = skill::powder_blend::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("powder_blend"));
    EXPECT_GT(cands[0].confidence, 0.99);
    ASSERT_TRUE(cands[0].recovered_params["components"].is_array());
    EXPECT_EQ(cands[0].recovered_params["components"][0]["material"].get<std::string>(),
              std::string("stainless_316l"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::powder_blend::recognize(raw).empty());
}

// ─── 5. Component count matches input ────────────────────────────────────
TEST(SkillPowderBlend, ComponentCountMatchesInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::powder_blend::Input in;
    in.components = {
        { "iron_pure",          0.80 },
        { "nickel_pure",        0.10 },
        { "copper_pure",        0.05 },
        { "carbon_graphite",    0.05 },
    };

    auto out = skill::powder_blend::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["component_count"].get<int>(), 4);
    EXPECT_EQ(out.signature.pattern["components"].size(), 4u);
}
