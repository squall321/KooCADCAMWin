// @lat: [[process/test-strategy#skill round-trip]]
//
// profile_milling — outer-shell removal leaving an inset island.
//
// Tests:
//   1. Apply removes the expected annular volume.
//   2. DFM rejects sub-min wall thickness (DFM-001 analog).
//   3. DFM rejects offset that would collapse the inset.
//   4. Recognize recovers the offset and depth from the resulting topology.
//   5. Edge case: deeper-than-thickness depth is rejected.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/profile_milling.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}
}  // namespace

// ─── 1. Basic synthesis works ─────────────────────────────────────────────
TEST(SkillProfileMilling, ApplyRemovesOuterShell)
{
    // 60 × 40 × 10 stock.  Profile in by 2 mm on each side, 3 mm deep.
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::profile_milling::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir     = gp_Dir(0, 0, -1);
    in.offset_in_mm = 2.0;
    in.depth_mm     = 3.0;

    auto out = skill::profile_milling::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected removed = (outer − inset) × depth
    //                  = (60·40 − 56·36) × 3 = (2400 − 2016) × 3 = 1152 mm³.
    const double approx = (60.0 * 40.0 - 56.0 * 36.0) * 3.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.05);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("profile_milling"));
}

// ─── 2. DFM rejects sub-min offset ───────────────────────────────────────
TEST(SkillProfileMilling, ValidateRejectsSubMinOffset)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::profile_milling::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.offset_in_mm = 0.2;             // < 0.4 mm
    in.depth_mm     = 3.0;

    auto r = skill::profile_milling::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-001") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::profile_milling::apply(*stock, in), skill::SkillError);
}

// ─── 3. DFM rejects offset that would collapse the inset ─────────────────
TEST(SkillProfileMilling, ValidateRejectsCollapsedInset)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    skill::profile_milling::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.offset_in_mm = 12.0;            // 2·12 = 24 > 20 bbox extent
    in.depth_mm     = 3.0;

    auto r = skill::profile_milling::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::profile_milling::apply(*stock, in), skill::SkillError);
}

// ─── 4. Recognize recovers offset and depth ──────────────────────────────
TEST(SkillProfileMilling, RecognizeRecoversOffsetAndDepth)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::profile_milling::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.offset_in_mm = 2.0;
    in.depth_mm     = 3.0;

    auto out = skill::profile_milling::apply(*stock, in);
    auto cands = skill::profile_milling::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty()) << "expected at least one profile candidate";

    const auto& c = cands.front();
    EXPECT_NEAR(c.recovered_params["offset_in_mm"].get<double>(), 2.0, 0.1);
    EXPECT_NEAR(c.recovered_params["depth_mm"].get<double>(),     3.0, 0.1);
}

// ─── 5. Edge case: depth > workpiece thickness rejected ──────────────────
TEST(SkillProfileMilling, ValidateRejectsExcessiveDepth)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 5.0);

    skill::profile_milling::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.offset_in_mm = 1.0;
    in.depth_mm     = 8.0;             // > 5 mm thickness

    auto r = skill::profile_milling::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::profile_milling::apply(*stock, in), skill::SkillError);
}
