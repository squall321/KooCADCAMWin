// @lat: [[process/test-strategy#skill round-trip]]
//
// investment_cast — lost-wax investment-casting process skill (no
// geometric change).
//
// Cases (6):
//   1. Apply is a geometric no-op.
//   2. Signature records investment-cast metadata + alias "lost_wax".
//   3. DFM-CAST-WALL rejects wall_thickness < 1.5 mm.
//   4. DFM-CAST-SHRINK rejects shrink_pct outside [0.5, 3.0].
//   5. DFM accepts 1.5 mm walls (sand-cast would reject at this thickness).
//   6. Recognize replays history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/investment_cast.hpp"

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

// ─── 1. Apply: no geometric change ─────────────────────────────────────────
TEST(SkillInvestmentCast, ApplyIsGeometricNoOp)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    const double volBefore = volumeOf(stock->shape());

    skill::investment_cast::Input in;
    in.material               = "stainless_steel";
    in.wall_thickness_mm      = 2.0;
    in.wax_pattern_shrink_pct = 1.5;

    auto out = skill::investment_cast::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_EQ(out.workpiece->faceCount(), stock->faceCount());

    EXPECT_EQ(out.signature.skill_id, std::string("investment_cast"));
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_process_only"].get<bool>());
}

// ─── 2. Signature carries investment-cast metadata + alias ─────────────────
TEST(SkillInvestmentCast, SignatureRecordsLostWaxAlias)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::investment_cast::Input in;
    in.material               = "titanium";
    in.wall_thickness_mm      = 2.5;
    in.wax_pattern_shrink_pct = 2.0;

    auto out = skill::investment_cast::apply(*stock, in);

    EXPECT_EQ(out.signature.params["material"].get<std::string>(),
              std::string("titanium"));
    EXPECT_NEAR(out.signature.params["wall_thickness_mm"].get<double>(), 2.5, 1e-9);
    EXPECT_NEAR(out.signature.params["wax_pattern_shrink_pct"].get<double>(),
                2.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["process_alias"].get<std::string>(),
              std::string("lost_wax"));
    EXPECT_EQ(out.signature.pattern["process_subtype"].get<std::string>(),
              std::string("investment_cast"));
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("investment_cast_shell"));
}

// ─── 3. DFM rejects sub-1.5 mm wall ────────────────────────────────────────
TEST(SkillInvestmentCast, ValidateRejectsTooThinWall)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::investment_cast::Input in;
    in.material               = "stainless_steel";
    in.wall_thickness_mm      = 1.0;       // < 1.5 mm
    in.wax_pattern_shrink_pct = 1.5;

    auto r = skill::investment_cast::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CAST-WALL" && f.severity == "error") {
            found = true;
            break;
        }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::investment_cast::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects shrink_pct outside [0.5, 3.0] ──────────────────────────
TEST(SkillInvestmentCast, ValidateRejectsBadShrink)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::investment_cast::Input tooLow;
    tooLow.material               = "stainless_steel";
    tooLow.wall_thickness_mm      = 2.0;
    tooLow.wax_pattern_shrink_pct = 0.2;       // < 0.5
    {
        auto r = skill::investment_cast::validate(*stock, tooLow);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-CAST-SHRINK") { found = true; break; }
        EXPECT_TRUE(found);
    }

    skill::investment_cast::Input tooHigh = tooLow;
    tooHigh.wax_pattern_shrink_pct = 5.0;      // > 3.0
    {
        auto r = skill::investment_cast::validate(*stock, tooHigh);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-CAST-SHRINK") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ─── 5. Investment cast accepts 1.5 mm walls (vs sand cast minimum 3 mm) ───
TEST(SkillInvestmentCast, AcceptsFineWalls)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::investment_cast::Input in;
    in.material               = "stainless_steel";
    in.wall_thickness_mm      = 1.5;       // = minimum, allowed
    in.wax_pattern_shrink_pct = 1.5;

    auto r = skill::investment_cast::validate(*stock, in);
    EXPECT_TRUE(r.passed) << "1.5 mm wall must be acceptable for investment cast";

    auto out = skill::investment_cast::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("investment_cast"));
}

// ─── 6. Recognize replays the casting metadata ─────────────────────────────
TEST(SkillInvestmentCast, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::investment_cast::Input in;
    in.material               = "cobalt_chrome";
    in.wall_thickness_mm      = 2.2;
    in.wax_pattern_shrink_pct = 1.8;

    auto out   = skill::investment_cast::apply(*stock, in);
    auto cands = skill::investment_cast::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_GT(cands[0].confidence, 0.99);
    EXPECT_EQ(cands[0].recovered_params["material"].get<std::string>(),
              std::string("cobalt_chrome"));
    EXPECT_NEAR(cands[0].recovered_params["wall_thickness_mm"].get<double>(),
                2.2, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["wax_pattern_shrink_pct"].get<double>(),
                1.8, 1e-9);
}
