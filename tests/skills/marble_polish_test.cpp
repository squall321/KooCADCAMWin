// @lat: [[process/test-strategy#skill round-trip]]
//
// marble_polish skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection on non-positive area, metadata-only recognition, and
// non-monotonic grit progression surfacing as a warning (non-blocking).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/marble_polish.hpp"

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

skill::marble_polish::Input goodInput()
{
    skill::marble_polish::Input in;
    in.surface_area_m2         = 4.0;
    in.polish_grit_progression = { 60, 120, 220, 400, 800, 1500, 3000 };
    in.final_gloss_units       = 90.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillMarblePolish, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(2000.0, 1000.0, 30.0, "marble");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::marble_polish::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("marble_polish"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (area = 0) ────────────────────────────
TEST(SkillMarblePolish, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(2000.0, 1000.0, 30.0, "marble");

    auto in = goodInput();
    in.surface_area_m2 = 0.0;

    auto r = skill::marble_polish::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::marble_polish::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + area/grit/gloss ─────────────────────────
TEST(SkillMarblePolish, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(2000.0, 1000.0, 30.0, "marble");

    auto in = goodInput();
    in.surface_area_m2         = 6.5;
    in.polish_grit_progression = { 100, 220, 600, 1500, 3000 };
    in.final_gloss_units       = 88.0;

    auto out = skill::marble_polish::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("marble_polish"));
    EXPECT_NEAR(out.signature.pattern["surface_area_m2"].get<double>(),
                6.5, 1e-9);
    EXPECT_EQ(out.signature.pattern["polish_grit_progression"].size(), 5u);
    EXPECT_NEAR(out.signature.pattern["final_gloss_units"].get<double>(),
                88.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["pass_count"].get<std::size_t>(), 5u);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillMarblePolish, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(2000.0, 1000.0, 30.0, "marble");
    auto out = skill::marble_polish::apply(*stock, goodInput());

    auto cands = skill::marble_polish::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["surface_area_m2"].get<double>(),
                4.0, 1e-9);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::marble_polish::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "marble_polish cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: non-monotonic grit ladder triggers warning ──────────────
TEST(SkillMarblePolish, NonMonotonicGritEmitsWarning)
{
    auto stock = skill::createCuboidStock(2000.0, 1000.0, 30.0, "marble");

    auto in = goodInput();
    in.polish_grit_progression = { 100, 220, 120, 1500 };   // 120 < 220
    in.final_gloss_units       = 70.0;     // keep within usual GU band

    auto r = skill::marble_polish::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "non-monotonic grit ladder should warn but not block";
    EXPECT_TRUE(hasFinding(r, "DFM-POLISH-GRIT-ORDER"));
    EXPECT_NO_THROW(skill::marble_polish::apply(*stock, in));
}
