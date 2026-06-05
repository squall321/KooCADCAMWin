// @lat: [[process/test-strategy#skill round-trip]]
//
// anchor_chain_locker_pipe_compound (slice 16) — REAL geometric verification.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/anchor_chain_locker_pipe_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

skill::anchor_chain_locker_pipe_compound::Input goodInput()
{
    skill::anchor_chain_locker_pipe_compound::Input in;
    in.entry_face                = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm               = 250.0;
    in.center_y_mm               = 250.0;
    in.axis_dir                  = gp_Dir(0, 0, 1);
    in.pipe_outer_dia_mm         = 250.0;
    in.pipe_wall_thickness_mm    = 12.0;
    in.pipe_length_mm            = 80.0;
    in.grating_ring_position_z_mm = 40.0;
    in.grating_ring_dia_mm       = 280.0;
    in.grating_ring_depth_mm     = 6.0;
    return in;
}

}  // namespace

TEST(SkillAnchorChainLockerPipe, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(500.0, 500.0, 80.0);
    ASSERT_FALSE(stock->shape().IsNull());

    const double volBefore = volumeOf(stock->shape());
    const int facesBefore  = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::anchor_chain_locker_pipe_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    const double removed  = volBefore - volAfter;

    const double rPipe = in.pipe_outer_dia_mm / 2.0;
    const double vBore = M_PI * rPipe * rPipe * in.pipe_length_mm;
    EXPECT_GT(removed, vBore * 0.9);
    EXPECT_GT(out.workpiece->faceCount(), facesBefore);
}

TEST(SkillAnchorChainLockerPipe, ValidateRejectsGratingTooNarrow)
{
    auto stock = skill::createCuboidStock(500.0, 500.0, 80.0);

    auto in = goodInput();
    in.grating_ring_dia_mm = in.pipe_outer_dia_mm * 0.5;  // < pipe OD

    auto r = skill::anchor_chain_locker_pipe_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-GRATING"));
    EXPECT_THROW(skill::anchor_chain_locker_pipe_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillAnchorChainLockerPipe, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(500.0, 500.0, 80.0);

    auto in  = goodInput();
    auto out = skill::anchor_chain_locker_pipe_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("anchor_chain_locker_pipe_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("marine_feature_type", std::string{}),
              std::string("chain_locker_pipe"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 2);
}

TEST(SkillAnchorChainLockerPipe, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(500.0, 500.0, 80.0);

    auto in  = goodInput();
    auto out = skill::anchor_chain_locker_pipe_compound::apply(*stock, in);
    auto cands = skill::anchor_chain_locker_pipe_compound::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    bool ok = false;
    for (const auto& c : cands) {
        if (c.confidence >= 0.8) { ok = true; break; }
    }
    EXPECT_TRUE(ok);
}

TEST(SkillAnchorChainLockerPipe, ValidateRejectsWallTooThin)
{
    auto stock = skill::createCuboidStock(500.0, 500.0, 80.0);

    auto in = goodInput();
    in.pipe_wall_thickness_mm = 1.0;  // < 3 mm

    auto r = skill::anchor_chain_locker_pipe_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-MARINE-PLATE"));
}
