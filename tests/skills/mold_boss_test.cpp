// @lat: [[process/test-strategy#skill round-trip]]
//
// mold_boss — cylindrical post on a planar face, optionally hollow.
//
// Cases (6):
//   1. Solid boss fuses onto stock and adds π r² h volume.
//   2. Hollow boss adds (outer - inner) ring volume.
//   3. DFM rejects too-small outer diameter (< 1.5 mm).
//   4. DFM rejects too-thin hollow wall.
//   5. recognize replays history.
//   6. Topology recognize works on a freshly built boss (no history) —
//      finds a cylindrical face.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mold_boss.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

}  // namespace

// ─── 1. Solid boss adds volume ────────────────────────────────────────────
TEST(SkillMoldBoss, ApplyAddsSolidBossVolume)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const double stockVol = volumeOf(stock->shape());

    skill::mold_boss::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 20.0;
    in.position_y_mm = 15.0;
    in.height_mm     = 5.0;
    in.outer_dia_mm  = 6.0;
    in.inner_dia_mm  = 0.0;     // solid
    in.draft_angle_deg = 1.0;

    auto out = skill::mold_boss::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double added = volumeOf(out.workpiece->shape()) - stockVol;
    // Slight taper from 1° draft reduces vol a bit — use cone formula:
    // V = (π h / 3)(r1² + r1 r2 + r2²)
    const double r1 = 3.0;
    const double r2 = std::max(0.1, r1 - 5.0 * std::tan(1.0 * M_PI / 180.0));
    const double approxAdded = (M_PI * 5.0 / 3.0) * (r1 * r1 + r1 * r2 + r2 * r2);
    EXPECT_GT(added, 0.0);
    EXPECT_NEAR(added, approxAdded, approxAdded * 0.10);

    EXPECT_EQ(out.signature.skill_id, std::string("mold_boss"));
    EXPECT_EQ(out.signature.pattern["is_hollow"].get<bool>(), false);
}

// ─── 2. Hollow boss adds ring volume ──────────────────────────────────────
TEST(SkillMoldBoss, ApplyHollowBossRingVolume)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const double stockVol = volumeOf(stock->shape());

    skill::mold_boss::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 15.0;
    in.height_mm     = 5.0;
    in.outer_dia_mm  = 8.0;
    in.inner_dia_mm  = 4.0;
    in.draft_angle_deg = 1.0;

    auto out = skill::mold_boss::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double added = volumeOf(out.workpiece->shape()) - stockVol;
    // Outer (with draft) minus straight inner cylinder × cutDepth (height - floor).
    // Just verify added > 0 and < solid boss equivalent.
    EXPECT_GT(added, 0.0);
    const double solidEquiv = M_PI * 4.0 * 4.0 * 5.0;
    EXPECT_LT(added, solidEquiv);

    EXPECT_EQ(out.signature.pattern["is_hollow"].get<bool>(), true);
}

// ─── 3. DFM rejects undersized outer dia ──────────────────────────────────
TEST(SkillMoldBoss, ValidateRejectsTooSmallOuter)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::mold_boss::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 15.0;
    in.height_mm     = 3.0;
    in.outer_dia_mm  = 1.0;       // < 1.5
    in.inner_dia_mm  = 0.0;

    auto r = skill::mold_boss::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BOSS-DIA") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::mold_boss::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects too-thin wall on hollow boss ──────────────────────────
TEST(SkillMoldBoss, ValidateRejectsThinHollowWall)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::mold_boss::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 15.0;
    in.height_mm     = 4.0;
    in.outer_dia_mm  = 3.0;
    in.inner_dia_mm  = 2.5;     // wall = 0.5 < 1.2

    auto r = skill::mold_boss::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BOSS-WALL") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 5. Recognize replays history ─────────────────────────────────────────
TEST(SkillMoldBoss, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::mold_boss::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 15.0;
    in.position_y_mm = 12.0;
    in.height_mm     = 4.5;
    in.outer_dia_mm  = 5.0;
    in.inner_dia_mm  = 0.0;
    in.draft_angle_deg = 1.0;

    auto out = skill::mold_boss::apply(*stock, in);
    auto cands = skill::mold_boss::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.7);
    EXPECT_NEAR(c.recovered_params["outer_dia_mm"].get<double>(), 5.0, 0.05);
    EXPECT_NEAR(c.recovered_params["height_mm"].get<double>(), 4.5, 0.05);
    EXPECT_NEAR(c.recovered_params["position_x_mm"].get<double>(), 15.0, 0.05);
}

// ─── 6. Topology recognize finds cylindrical face on history-less wp ──────
TEST(SkillMoldBoss, RecognizeFindsCylindricalFaceWithoutHistory)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::mold_boss::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 15.0;
    in.height_mm     = 4.0;
    in.outer_dia_mm  = 5.0;
    in.inner_dia_mm  = 0.0;
    in.draft_angle_deg = 0.0;    // straight cylinder

    auto out = skill::mold_boss::apply(*stock, in);

    // Reconstruct a workpiece with NO feature history but the same shape.
    auto wpNoHist = std::make_shared<skill::Workpiece>(
        out.workpiece->shape(), out.workpiece->material());
    auto cands = skill::mold_boss::recognize(*wpNoHist);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands.front().recovered_params["outer_dia_mm"].get<double>(),
                5.0, 0.1);
}
