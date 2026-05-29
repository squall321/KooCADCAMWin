// @lat: [[process/test-strategy#skill round-trip]]
//
// base_plate_install skill — 5-case sweep covering identity-geometry
// synthesis, DFM rejection of thin plate / low bolt count, metadata-only
// recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/base_plate_install.hpp"

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

skill::base_plate_install::Input goodInput()
{
    skill::base_plate_install::Input in;
    in.plate_thick_mm = 25.0;
    in.bolt_count     = 4;
    in.anchor_dia_mm  = 24.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillBasePlateInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::base_plate_install::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("base_plate_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (plate 5 mm < 10 mm minimum) ───────────
TEST(SkillBasePlateInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.plate_thick_mm = 5.0;
    auto r = skill::base_plate_install::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-BP-THK"));
    EXPECT_THROW(skill::base_plate_install::apply(*stock, in), skill::SkillError);

    // bolt count too low
    auto in2 = goodInput();
    in2.bolt_count = 1;
    auto r2 = skill::base_plate_install::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-BP-COUNT"));
}

// ─── 3. Signature records kind + plate_thick/bolt_count/anchor_dia ───────
TEST(SkillBasePlateInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.plate_thick_mm = 32.0;
    in.bolt_count     = 8;
    in.anchor_dia_mm  = 30.0;

    auto out = skill::base_plate_install::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("base_plate_install"));
    EXPECT_NEAR(out.signature.pattern["plate_thick_mm"].get<double>(),
                32.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["bolt_count"].get<int>(), 8);
    EXPECT_NEAR(out.signature.pattern["anchor_dia_mm"].get<double>(),
                30.0, 1e-9);
    EXPECT_TRUE(out.signature.pattern["symmetric"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillBasePlateInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::base_plate_install::apply(*stock, goodInput());

    auto cands = skill::base_plate_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["bolt_count"].get<int>(), 4);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::base_plate_install::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "base_plate_install cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: more bolts ⇒ longer torque cycle time, odd count → info
TEST(SkillBasePlateInstall, BoltCountScalesCycleTimeAndOddInfo)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto fewIn = goodInput();
    fewIn.bolt_count = 4;
    auto fewOut = skill::base_plate_install::apply(*stock, fewIn);

    auto manyIn = goodInput();
    manyIn.bolt_count = 12;
    auto manyOut = skill::base_plate_install::apply(*stock, manyIn);

    EXPECT_GT(manyOut.signature.tooling.est_cycle_time_s,
              fewOut.signature.tooling.est_cycle_time_s);
    EXPECT_NEAR(manyOut.signature.tooling.est_cycle_time_s, 30.0 * 12.0, 1e-9);

    // Odd count emits info-level finding but does NOT block.
    auto oddIn = goodInput();
    oddIn.bolt_count = 5;
    auto oddR = skill::base_plate_install::validate(*stock, oddIn);
    EXPECT_TRUE(oddR.passed);
    EXPECT_TRUE(hasFinding(oddR, "DFM-BP-COUNT"));
}
