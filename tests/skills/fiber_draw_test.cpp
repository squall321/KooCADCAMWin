// @lat: [[process/test-strategy#skill round-trip]]
//
// fiber_draw skill — preform-to-fiber draw tower.  Covers:
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput  (draw_temp_c too low → DFM error)
//   3. SignatureRecordsKind+params
//   4. RecognizeMetadataReplay
//   5. Specific assertion — coating_applied flag set for valid speed

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/fiber_draw.hpp"

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

}  // namespace

// ─── 1. Apply: clones geometry unchanged ──────────────────────────────────
TEST(SkillFiberDraw, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(125.0, 500.0, "aluminum_6061");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::fiber_draw::Input in;
    in.final_dia_um     = 125.0;
    in.draw_temp_c      = 2050.0;
    in.draw_speed_m_min = 1200.0;

    auto out = skill::fiber_draw::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("fiber_draw"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input (draw_temp_c too low) ──────────────────
TEST(SkillFiberDraw, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(125.0, 500.0);

    skill::fiber_draw::Input in;
    in.final_dia_um     = 125.0;
    in.draw_temp_c      = 1500.0;     // way below silica softening
    in.draw_speed_m_min = 1200.0;

    auto r = skill::fiber_draw::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-DRAW-TEMP-LOW"));
    EXPECT_THROW(skill::fiber_draw::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + params ───────────────────────────────────
TEST(SkillFiberDraw, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(125.0, 500.0);

    skill::fiber_draw::Input in;
    in.final_dia_um     = 250.0;
    in.draw_temp_c      = 2080.0;
    in.draw_speed_m_min = 1500.0;

    auto out = skill::fiber_draw::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("fiber_draw"));
    EXPECT_NEAR(out.signature.pattern["final_dia_um"].get<double>(),
                250.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["draw_temp_c"].get<double>(),
                2080.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["draw_speed_m_min"].get<double>(),
                1500.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("draw_tower"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFiberDraw, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(125.0, 500.0);

    skill::fiber_draw::Input in;
    in.final_dia_um     = 125.0;
    in.draw_temp_c      = 2050.0;
    in.draw_speed_m_min = 1200.0;
    auto out = skill::fiber_draw::apply(*stock, in);

    auto cands = skill::fiber_draw::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["final_dia_um"].get<double>(),
                125.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::fiber_draw::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "fiber_draw cannot be recognized geometrically";
}

// ─── 5. Specific assertion — coating_applied flag true at standard speed ──
TEST(SkillFiberDraw, PatternFlagsInlineCoatingAtStandardSpeed)
{
    auto stock = skill::createCylindricalStock(125.0, 500.0);

    skill::fiber_draw::Input in;
    in.final_dia_um     = 125.0;
    in.draw_temp_c      = 2050.0;
    in.draw_speed_m_min = 1200.0;     // squarely in coater window

    auto out = skill::fiber_draw::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["coating_applied"].get<bool>());

    // Very slow draw should disable inline coating flag.
    skill::fiber_draw::Input slow = in;
    slow.draw_speed_m_min = 20.0;
    auto out2 = skill::fiber_draw::apply(*stock, slow);
    EXPECT_FALSE(out2.signature.pattern["coating_applied"].get<bool>());
}
