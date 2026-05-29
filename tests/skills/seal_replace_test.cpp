// @lat: [[process/test-strategy#skill round-trip]]
//
// seal_replace skill — 5-case maintenance sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/seal_replace.hpp"

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

skill::seal_replace::Input goodInput()
{
    skill::seal_replace::Input in;
    in.seal_type        = "O-ring";
    in.seal_id          = "AS568-214";
    in.leak_test_passed = true;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ─────────────────────────────────
TEST(SkillSealReplace, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::seal_replace::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("seal_replace"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ───────────────────────────────────────
TEST(SkillSealReplace, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.seal_id = "";                       // empty
    auto r = skill::seal_replace::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::seal_replace::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.seal_type = "";                    // empty type
    auto r2 = skill::seal_replace::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillSealReplace, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.seal_type        = "lip";
    in.seal_id          = "TC-25-40-7";
    in.leak_test_passed = true;

    auto out = skill::seal_replace::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("seal_replace"));
    EXPECT_EQ(out.signature.pattern["seal_type"].get<std::string>(),
              std::string("lip"));
    EXPECT_EQ(out.signature.pattern["seal_id"].get<std::string>(),
              std::string("TC-25-40-7"));
    EXPECT_TRUE(out.signature.pattern["leak_test_passed"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillSealReplace, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::seal_replace::apply(*stock, goodInput());

    auto cands = skill::seal_replace::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["seal_id"].get<std::string>(),
              std::string("AS568-214"));

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::seal_replace::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "seal_replace cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: failed leak test BLOCKS apply ──────────────────────────
TEST(SkillSealReplace, FailedLeakTestBlocksApply)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.leak_test_passed = false;           // leak detected post-install

    auto r = skill::seal_replace::validate(*stock, in);
    EXPECT_FALSE(r.passed)
        << "failed leak test MUST block the maintenance record from closing";
    EXPECT_TRUE(hasFinding(r, "DFM-SEAL-LEAK"));
    EXPECT_THROW(skill::seal_replace::apply(*stock, in), skill::SkillError);
}
