// @lat: [[process/test-strategy#skill round-trip]]
//
// peel_test skill — adhesive peel-test no-op sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/peel_test.hpp"

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

skill::peel_test::Input goodInput()
{
    skill::peel_test::Input in;
    in.peel_angle_deg  = 90.0;
    in.peel_force_n_mm = 1.5;
    in.adhesive_type   = "PSA";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ─────────────────────────────────
TEST(SkillPeelTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::peel_test::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("peel_test"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (angle out of (0,180]) ────────────────
TEST(SkillPeelTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.peel_angle_deg = 200.0;             // invalid

    auto r = skill::peel_test::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PEEL-ANGLE"));
    EXPECT_THROW(skill::peel_test::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.peel_force_n_mm = -0.1;
    auto r2 = skill::peel_test::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. Signature records kind + angle/force/adhesive ────────────────────
TEST(SkillPeelTest, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.peel_angle_deg  = 180.0;
    in.peel_force_n_mm = 3.2;
    in.adhesive_type   = "epoxy";

    auto out = skill::peel_test::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("peel_test"));
    EXPECT_NEAR(out.signature.pattern["peel_angle_deg"].get<double>(),  180.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["peel_force_n_mm"].get<double>(),   3.2, 1e-9);
    EXPECT_EQ(out.signature.pattern["adhesive_type"].get<std::string>(),
              std::string("epoxy"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.pattern["test_standard"].get<std::string>(),
              std::string("ASTM_D1876"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPeelTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::peel_test::apply(*stock, goodInput());

    auto cands = skill::peel_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["peel_angle_deg"].get<double>(),
                90.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::peel_test::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "peel_test cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: weaker peel force ⇒ bond_quality = "weak" ──────────────
TEST(SkillPeelTest, BondQualityReflectsPeelForce)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto weakIn = goodInput();
    weakIn.peel_force_n_mm = 0.2;
    auto weakOut = skill::peel_test::apply(*stock, weakIn);
    EXPECT_EQ(weakOut.signature.pattern["bond_quality"].get<std::string>(),
              std::string("weak"));

    auto strongIn = goodInput();
    strongIn.peel_force_n_mm = 8.0;
    auto strongOut = skill::peel_test::apply(*stock, strongIn);
    EXPECT_EQ(strongOut.signature.pattern["bond_quality"].get<std::string>(),
              std::string("strong"));

    auto nominalIn = goodInput();
    nominalIn.peel_force_n_mm = 2.0;
    auto nominalOut = skill::peel_test::apply(*stock, nominalIn);
    EXPECT_EQ(nominalOut.signature.pattern["bond_quality"].get<std::string>(),
              std::string("nominal"));
}
