// @lat: [[process/test-strategy#skill round-trip]]
//
// drivetrain_install — 5-case sweep covering identity-geometry synthesis,
// drivetrain-type / id DFM gates, signature, metadata replay, and the
// AWD vs non-AWD cycle-time differentiation.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drivetrain_install.hpp"

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

// ── 1. Apply: geometry COMPLETELY unchanged ──────────────────────────────
TEST(SkillDrivetrainInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::drivetrain_install::Input in;
    in.drivetrain_type = "FWD";
    in.engine_id       = "E-1600-D";
    in.transmission_id = "T-6MT-B";

    auto out = skill::drivetrain_install::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("drivetrain_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: bad drivetrain_type, empty ids rejected ──────────────────────
TEST(SkillDrivetrainInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::drivetrain_install::Input bad;
    bad.drivetrain_type = "TURBOJET";        // invalid
    bad.engine_id       = "E-1";
    bad.transmission_id = "T-1";
    EXPECT_FALSE(skill::drivetrain_install::validate(*stock, bad).passed);
    EXPECT_THROW(skill::drivetrain_install::apply(*stock, bad),
                 skill::SkillError);

    bad.drivetrain_type = "FWD";
    bad.engine_id       = "";                 // invalid
    EXPECT_FALSE(skill::drivetrain_install::validate(*stock, bad).passed);

    bad.engine_id       = "E-1";
    bad.transmission_id = "";                 // invalid
    EXPECT_FALSE(skill::drivetrain_install::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillDrivetrainInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::drivetrain_install::Input in;
    in.drivetrain_type = "AWD";
    in.engine_id       = "E-3500-V6";
    in.transmission_id = "T-8AT-Z";

    auto out = skill::drivetrain_install::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("drivetrain_install"));
    EXPECT_EQ(out.signature.pattern["drivetrain_type"].get<std::string>(),
              std::string("AWD"));
    EXPECT_EQ(out.signature.pattern["engine_id"].get<std::string>(),
              std::string("E-3500-V6"));
    EXPECT_EQ(out.signature.pattern["transmission_id"].get<std::string>(),
              std::string("T-8AT-Z"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("drivetrain_marriage_station"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillDrivetrainInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::drivetrain_install::Input in;
    in.drivetrain_type = "RWD";
    in.engine_id       = "E-5000-V8";
    in.transmission_id = "T-10AT-X";

    auto out = skill::drivetrain_install::apply(*stock, in);
    auto cands = skill::drivetrain_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["drivetrain_type"].get<std::string>(),
              std::string("RWD"));
    EXPECT_EQ(cands[0].recovered_params["engine_id"].get<std::string>(),
              std::string("E-5000-V8"));

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::drivetrain_install::recognize(raw).empty());
}

// ── 5. AWD has longer cycle than FWD/RWD (skill-specific) ────────────────
TEST(SkillDrivetrainInstall, AwdCycleTimeExceedsFwd)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::drivetrain_install::Input fwd;
    fwd.drivetrain_type = "FWD";
    fwd.engine_id       = "E-1";
    fwd.transmission_id = "T-1";
    auto outFwd = skill::drivetrain_install::apply(*stock, fwd);
    EXPECT_NEAR(outFwd.signature.tooling.est_cycle_time_s, 120.0, 1e-9);

    skill::drivetrain_install::Input awd;
    awd.drivetrain_type = "AWD";
    awd.engine_id       = "E-1";
    awd.transmission_id = "T-1";
    auto outAwd = skill::drivetrain_install::apply(*stock, awd);
    EXPECT_NEAR(outAwd.signature.tooling.est_cycle_time_s, 180.0, 1e-9);

    EXPECT_GT(outAwd.signature.tooling.est_cycle_time_s,
              outFwd.signature.tooling.est_cycle_time_s);
}
