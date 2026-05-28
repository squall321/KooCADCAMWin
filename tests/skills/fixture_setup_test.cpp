// @lat: [[process/test-strategy#skill round-trip]]
//
// fixture_setup skill — 5-case sweep.  Metadata-only, geometry no-op.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/fixture_setup.hpp"

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

// ─── 1. Apply: geometry unchanged + signature added ───────────────────────
TEST(SkillFixtureSetup, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::fixture_setup::Input in;
    in.fixture_type    = "vise";
    in.clamp_locations = { { 0.0, 0.0, 0.0 }, { 40.0, 40.0, 0.0 } };
    in.work_offsets    = { "G54" };

    auto out = skill::fixture_setup::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("fixture_setup"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate: empty clamp_locations rejected ──────────────────────────
TEST(SkillFixtureSetup, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fixture_setup::Input in;
    in.fixture_type    = "vise";
    in.clamp_locations = {};                  // empty → error
    in.work_offsets    = { "G54" };

    auto r = skill::fixture_setup::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::fixture_setup::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records fixture_type + clamp_locations + work_offsets ──
TEST(SkillFixtureSetup, SignatureRecordsKindAndKeys)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fixture_setup::Input in;
    in.fixture_type    = "three_jaw_chuck";
    in.clamp_locations = { { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 }, { 7.0, 8.0, 9.0 } };
    in.work_offsets    = { "G54", "G55" };

    auto out = skill::fixture_setup::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("fixture_setup"));
    EXPECT_EQ(out.signature.pattern["fixture_type"].get<std::string>(),
              std::string("three_jaw_chuck"));
    EXPECT_EQ(out.signature.pattern["clamp_locations"].size(), 3u);
    EXPECT_EQ(out.signature.pattern["work_offsets"].size(), 2u);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFixtureSetup, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fixture_setup::Input in;
    in.fixture_type    = "magnetic_chuck";
    in.clamp_locations = { { 0.0, 0.0, 0.0 } };
    in.work_offsets    = { "G54" };
    auto out = skill::fixture_setup::apply(*stock, in);

    auto cands = skill::fixture_setup::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["fixture_type"].get<std::string>(),
              std::string("magnetic_chuck"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::fixture_setup::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific assertion: nonstandard work_offset emits warning, OK ─────
TEST(SkillFixtureSetup, NonstandardOffsetIsWarningNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fixture_setup::Input in;
    in.fixture_type    = "vise";
    in.clamp_locations = { { 0.0, 0.0, 0.0 } };
    in.work_offsets    = { "G99" };           // bad form → warning

    auto r = skill::fixture_setup::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "FX-OFFSET-FORM"));
    EXPECT_NO_THROW(skill::fixture_setup::apply(*stock, in));
}
