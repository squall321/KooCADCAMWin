// @lat: [[process/test-strategy#skill round-trip]]
//
// sleeper_install skill — 5-case sweep covering identity-geometry
// synthesis, DFM rejection of out-of-range spacing, metadata-only
// recognition, and track-length computation.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sleeper_install.hpp"

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

skill::sleeper_install::Input goodInput()
{
    skill::sleeper_install::Input in;
    in.sleeper_type = "concrete";
    in.spacing_mm   = 600.0;
    in.count        = 100;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillSleeperInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::sleeper_install::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("sleeper_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (spacing too tight / too wide) ─────────
TEST(SkillSleeperInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.spacing_mm = 300.0;            // < 450 mm
    auto r = skill::sleeper_install::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-SL-SPACING"));
    EXPECT_THROW(skill::sleeper_install::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.spacing_mm = 1200.0;          // > 900 mm
    auto r2 = skill::sleeper_install::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-SL-SPACING"));

    auto in3 = goodInput();
    in3.count = 0;                    // count < 1
    auto r3 = skill::sleeper_install::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + type/spacing/count ───────────────────────
TEST(SkillSleeperInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.sleeper_type = "wood";
    in.spacing_mm   = 650.0;
    in.count        = 50;
    auto out = skill::sleeper_install::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("sleeper_install"));
    EXPECT_EQ(out.signature.pattern["sleeper_type"].get<std::string>(),
              std::string("wood"));
    EXPECT_NEAR(out.signature.pattern["spacing_mm"].get<double>(),
                650.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["count"].get<int>(), 50);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSleeperInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::sleeper_install::apply(*stock, goodInput());

    auto cands = skill::sleeper_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["sleeper_type"].get<std::string>(),
              std::string("concrete"));
    EXPECT_NEAR(cands[0].recovered_params["spacing_mm"].get<double>(),
                600.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::sleeper_install::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "sleeper_install cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: track_length_mm = spacing × (count - 1) ─────────────────
TEST(SkillSleeperInstall, TrackLengthIsSpacingTimesCountMinusOne)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.spacing_mm = 600.0;
    in.count      = 101;             // 100 spans
    auto out = skill::sleeper_install::apply(*stock, in);

    EXPECT_NEAR(out.signature.pattern["track_length_mm"].get<double>(),
                600.0 * 100.0, 1e-6);
}
