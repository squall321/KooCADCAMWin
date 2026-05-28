// @lat: [[process/test-strategy#skill round-trip]]
//
// drill_lathe — drilling along the rotation axis from the right end.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_lathe.hpp"

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

// ─── 1. apply removes material ────────────────────────────────────────────
TEST(SkillDrillLathe, ApplyRemovesMaterial)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);   // Ø40 × 30
    const double volBefore = volumeOf(stock->shape());

    skill::drill_lathe::Input in;
    in.diameter_mm = 8.0;
    in.depth_mm    = 15.0;

    auto out = skill::drill_lathe::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected removed volume ≈ π r² d = π · 16 · 15 ≈ 754 mm³
    const double approx = M_PI * 16.0 * 15.0;
    const double diff   = volBefore - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.05);
}

// ─── 2. DFM rejects depth > stock length ──────────────────────────────────
TEST(SkillDrillLathe, ValidateRejectsOverDepth)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::drill_lathe::Input in;
    in.diameter_mm = 8.0;
    in.depth_mm    = 35.0;     // > stock length of 30

    auto r = skill::drill_lathe::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LATHE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::drill_lathe::apply(*stock, in), skill::SkillError);
}

// ─── 3. signature records kind ────────────────────────────────────────────
TEST(SkillDrillLathe, SignatureRecordsKind)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::drill_lathe::Input in;
    in.diameter_mm = 8.0;
    in.depth_mm    = 15.0;

    auto out = skill::drill_lathe::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("drill_lathe"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("drill_lathe"));
    EXPECT_GE(out.signature.pattern["cylindrical_face_count"].get<int>(), 1);
    EXPECT_FALSE(out.signature.pattern["through_hole"].get<bool>());
}

// ─── 4. recognize — geometric, then metadata replay if features absent ───
TEST(SkillDrillLathe, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::drill_lathe::Input in;
    in.diameter_mm = 8.0;
    in.depth_mm    = 15.0;

    auto out = skill::drill_lathe::apply(*stock, in);
    auto cands = skill::drill_lathe::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool matched = false;
    for (const auto& c : cands) {
        const double dia = c.recovered_params["diameter_mm"].get<double>();
        const double dep = c.recovered_params["depth_mm"].get<double>();
        if (std::abs(dia - in.diameter_mm) < 0.2 &&
            std::abs(dep - in.depth_mm)    < 0.2) {
            EXPECT_GT(c.confidence, 0.8);
            matched = true;
            break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 5. specific — through-hole flagged when depth ≈ length ───────────────
TEST(SkillDrillLathe, ThroughHoleDetected)
{
    auto stock = skill::createCylindricalStock(40.0, 30.0);

    skill::drill_lathe::Input in;
    in.diameter_mm = 6.0;
    in.depth_mm    = 30.0;     // exactly the stock length
    auto out = skill::drill_lathe::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["through_hole"].get<bool>());
}
